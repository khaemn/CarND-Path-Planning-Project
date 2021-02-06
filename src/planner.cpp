#include "planner.h"

#include "spline.h"

Planner::Planner(const RoadMap &map, const PlannerConstParams &params)
  : map_(map)
  , params_(params)
{
  clear_obstacles();
}

Planner::State Planner::state() const
{
  return state_;
}

void Planner::process_telemetry(const nlohmann::json &telemetry)
{
  clear_trajectory();

  parse_ego(telemetry);

  parse_prev_path(telemetry);

  parse_obstacles(telemetry);

  update_allowed_speed(); // Prevents collisions with obstacles ahead

  // Sensor Fusion Data, a list of all other cars on the same side
  //   of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  choose_next_state();

  generate_trajectory(telemetry);

  std::cout << "Desired spd " << desired_speed_ms_ << ", allowed spd " << allowed_now_speed_ms_
            << ", now in lane " << current_lane_ << ", slowed: " << is_slowed_down_by_obstacle_ahead
            << std::endl;
  std::cout << "Ego spd " << ego_.speed_ms << ", heading " << rad2deg(ego_.yaw) << ", s " << ego_.s
            << ", d " << ego_.d << std::endl;
}

const std::vector<float> &Planner::x_trajectory_points() const
{
  return trajectory_x_;
}

const std::vector<float> &Planner::y_trajectory_points() const
{
  return trajectory_y_;
}

void Planner::parse_ego(const nlohmann::json &telemetry)
{
  // Main car's localization Data
  ego_.x         = telemetry["x"];
  ego_.y         = telemetry["y"];
  ego_.s         = telemetry["s"];
  ego_.d         = telemetry["d"];
  ego_.yaw       = deg2rad(telemetry["yaw"]);
  ego_.speed_ms = double(telemetry["speed"]) / 3.6;

  update_current_lane();
}

void Planner::parse_prev_path(const nlohmann::json &telemetry)
{
  // Previous path data given to the Planner
  auto previous_path_x = telemetry["previous_path_x"];
  auto previous_path_y = telemetry["previous_path_y"];

  if (previous_path_x.size() != previous_path_y.size())
  {
    std::cout << "Error: previous path x size " << previous_path_x.size()
              << " != previous path y size " << previous_path_y.size() << std::endl;
  }

  for (size_t i{0}; i < previous_path_x.size(); ++i)
  {
    trajectory_x_.push_back(previous_path_x[i]);
    trajectory_y_.push_back(previous_path_y[i]);
  }

  // Previous path's end s and d values
  end_path_s_ = telemetry["end_path_s"];
  end_path_d_ = telemetry["end_path_d"];
}

void Planner::parse_obstacles(const nlohmann::json &telemetry)
{
  auto sensor_fusion = telemetry["sensor_fusion"];
  clear_obstacles();
  for (size_t i{0}; i < sensor_fusion.size(); ++i)
  {
    auto       obj = sensor_fusion[i];
    RoadObject obstacle{obj[0], obj[1], obj[2], obj[3],
                        obj[4], obj[5], obj[6], distance(ego_.x, ego_.y, obj[1], obj[2])};
    const auto its_lane = lane_num_of(obstacle.d);
    if (its_lane < 0 || its_lane > int(params_.num_lanes))
    {
      // skip this obstacle, it is out of our road;
      continue;
    }
    if (obstacle.s > ego_.s)
    {
      obstacles_ahead_[size_t(its_lane)].insert(obstacle);
    }
    else
    {
      obstacles_behind_[size_t(its_lane)].insert(obstacle);
    }
  }
}

void Planner::update_allowed_speed()
{
  is_slowed_down_by_obstacle_ahead = false;
  if (obstacles_ahead_.at(size_t(current_lane_)).empty())
  {
    allowed_now_speed_ms_ = desired_speed_ms_;
    return;
  }
  // Avoid collision
  const RoadObject &closest_ahead = *obstacles_ahead_.at(size_t(current_lane_)).begin();
  std::cout << "Obst: " << closest_ahead.id << " "
            << closest_ahead.distance_to_ccp << " s: " << closest_ahead.s
            << " d: " << closest_ahead.d << std::endl;

  if (closest_ahead.distance_to_ccp < params_.min_gap_lon)
  {
    // Full brake if there is no safe gap ahead
    allowed_now_speed_ms_ = 0.;
    is_slowed_down_by_obstacle_ahead = true;
    return;
  }

  if (closest_ahead.distance_to_ccp > params_.safe_gap_lon)
  {
    // Speed up if the lane is free
    allowed_now_speed_ms_ = desired_speed_ms_;
    return;
  }
  is_slowed_down_by_obstacle_ahead = true;

  // Slow down if there's an obstacle ahead at a safe distance
  const auto obst_heading        = std::atan(closest_ahead.vy / closest_ahead.vx);
  const auto full_obst_speed     = sqrt(pow(closest_ahead.vx, 2) + pow(closest_ahead.vy, 2));
  const auto angle_diff          = obst_heading - ego_.yaw;
  const auto obst_parallel_speed = cos(angle_diff) * full_obst_speed;
  std::cout << "Closest speed to me " << obst_parallel_speed;

  const auto coeff =
      (closest_ahead.distance_to_ccp - params_.min_gap_lon) / (params_.safe_gap_lon * 0.9);
  allowed_now_speed_ms_ = std::min(desired_speed_ms_, obst_parallel_speed * 0.98);
}

void Planner::update_current_lane()
{
  current_lane_ = lane_num_of(ego_.d);
}

void Planner::generate_keep_lane()
{
  // Generates the best possible keep lane trajectory.
  // Ensures safe stop before an obstacle, safe distance
  // to a moving obstacle ahead or a maximum allowed speed
  // on a free lane.
  static constexpr double s_step_m    = 30.;
  static constexpr size_t steps_ahead = 3;

  // We need (at least) 2 reference points in order to make the
  // path trajectory continuation tangent to the existing path.
  double ref_x(ego_.x), ref_y(ego_.y), prev_ref_x(ego_.x - cos(ego_.yaw)),
      prev_ref_y(ego_.y - sin(ego_.yaw)), ref_yaw(ego_.yaw);

  // Previous path is now stored in the trajectory array
  const auto prev_path_len = trajectory_x_.size();

  // if there are at least 2 pts in the prev path, the reference position
  // for the path should be at the end of the previous one; otherwise,
  // the ref position would be calculated from the CCP.
  double current_dist_inc = dist_inc_at_const_speed(ego_.speed_ms);
  if (prev_path_len >= 2)
  {
    ref_x            = double(trajectory_x_.at(prev_path_len - 1));
    ref_y            = double(trajectory_y_.at(prev_path_len - 1));
    prev_ref_x       = double(trajectory_x_.at(prev_path_len - 2));
    prev_ref_y       = double(trajectory_y_.at(prev_path_len - 2));
    ref_yaw          = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
    current_dist_inc = distance(ref_x, ref_y, prev_ref_x, prev_ref_y);
  }

  auto         ref_s_d = getFrenet(ref_x, ref_y, ref_yaw, map_.x, map_.y);
  const double ref_s   = ref_s_d.at(0);
  const double ref_d   = ref_s_d.at(1);
  //  const double d_center_offset = lane_center_d(current_lane_) - ref_d;
  //  const double d_step_m = d_center_offset / steps_ahead;

  vector<double> spline_keypts_x{prev_ref_x, ref_x};
  vector<double> spline_keypts_y{prev_ref_y, ref_y};

  // Add 'steps_ahead' more points along the lane, each next pt
  // closer to the lane center and further ahead, in map coords
  const auto future_lane = detect_optimal_lane();
  for (size_t i{1}; i <= steps_ahead; i++)
  {
    vector<double> coords =
        getXY(ref_s + i * s_step_m, lane_center_d(future_lane), map_.s, map_.x, map_.y);
    spline_keypts_x.push_back(coords[0]);
    spline_keypts_y.push_back(coords[1]);
  }

  // Shift car reference angle to 0 degrees for each pt
  const double sin_angle_diff = sin(0. - ref_yaw);
  const double cos_angle_diff = cos(0. - ref_yaw);
  for (size_t i{0}; i < spline_keypts_x.size(); ++i)
  {
    const double x = spline_keypts_x.at(i) - ref_x;
    const double y = spline_keypts_y.at(i) - ref_y;

    spline_keypts_x[i] = x * cos_angle_diff - y * sin_angle_diff;
    spline_keypts_y[i] = x * sin_angle_diff + y * cos_angle_diff;
  }
  const double current_x_in_car_cs = spline_keypts_x[1];
  tk::spline   spline;
  spline.set_points(spline_keypts_x, spline_keypts_y);


  const double allowed_distance_inc = dist_inc_at_const_speed(allowed_now_speed_ms_);

  double       dist_inc_for_next_step = current_dist_inc;
  const double sin_yaw                = sin(ref_yaw);
  const double cos_yaw                = cos(ref_yaw);
  const double accel_step             = dist_inc_delta_at_accel(params_.comfort_longitudinal_accel);
  double       prev_x_car             = current_x_in_car_cs;
  for (size_t i{1}; i <= params_.trajectory_length_pts - prev_path_len; ++i)
  {
    // With each step I change the desired traveled distance in a way
    // to approach the desired/allowed speed and keep a comfort accel/deccel.
    if (dist_inc_for_next_step < allowed_distance_inc)
    {
      dist_inc_for_next_step = std::min(allowed_distance_inc, dist_inc_for_next_step + accel_step);
    }
    else
    {
      dist_inc_for_next_step =
          std::max(allowed_distance_inc, dist_inc_for_next_step - 1.5 * accel_step);
    }
    const double next_x_car = prev_x_car + dist_inc_for_next_step;
    prev_x_car              = next_x_car;
    const double next_y_car = spline(next_x_car);

    // Rotate this point back to the map coord system
    const double next_x = next_x_car * cos_yaw - next_y_car * sin_yaw + ref_x;
    const double next_y = next_x_car * sin_yaw + next_y_car * cos_yaw + ref_y;

    trajectory_x_.push_back(float(next_x));
    trajectory_y_.push_back(float(next_y));
  }
}

int Planner::detect_optimal_lane()
{
  if (not is_slowed_down_by_obstacle_ahead)
  {
    return current_lane_;
  }
  auto      future_lane    = current_lane_;
  const int leftmost_lane  = std::max(0, current_lane_ - 1);
  const int rightmost_lane = std::min(int(params_.num_lanes - 1), current_lane_ + 1);

  for (int l{leftmost_lane}; l <= rightmost_lane; l++)
  {
    bool        is_ahead_clear(false);
    bool        is_behind_clear(false);
    const auto &ahead_check_lst = obstacles_ahead_.at(l);
    if (ahead_check_lst.empty() || ahead_check_lst.begin()->distance_to_ccp > params_.safe_gap_lon)
    {
      is_ahead_clear = true;
    }
    const auto &behind_check_lst = obstacles_ahead_.at(l);
    if (behind_check_lst.empty() || behind_check_lst.begin()->distance_to_ccp > params_.min_gap_lon)
    {
      is_behind_clear = true;
    }
    if (is_ahead_clear && is_behind_clear)
    {
      return l;
    }
  }

  return future_lane;
}

void Planner::clear_trajectory()
{
  trajectory_x_.clear();
  trajectory_y_.clear();
}

void Planner::clear_prev_path()
{
  end_path_d_ = ego_.d;
  end_path_s_ = ego_.s;
}

void Planner::clear_obstacles()
{
  obstacles_ahead_.clear();
  obstacles_behind_.clear();

  for (size_t i{0}; i < params_.num_lanes; ++i)
  {
    obstacles_ahead_.push_back({});
    obstacles_behind_.push_back({});
  }
}

double Planner::lane_center_d(int lane) const
{
    return params_.lane_width_m * (lane + 0.5);
}

int Planner::lane_num_of(double d)
{
    return int(std::floor(d / params_.lane_width_m));
}

void Planner::choose_next_state()
{
  state_ = State::KeepLane;
}

void Planner::generate_trajectory(const nlohmann::json &telemetry)
{
  // Jerk and accel values according to
  // https://repositories.lib.utexas.edu/bitstream/handle/2152/20856/cats_rr_40.pdf
  switch (state_)
  {
  case State::KeepLane:
    generate_keep_lane();
    break;
  default:
    break;
  }
}

double Planner::desired_speed_kmh() const
{
  return desired_speed_ms_ * 3.6;
}

double Planner::desired_speed_ms() const
{
  return desired_speed_ms_;
}

double Planner::dist_inc_at_const_speed(double speed_ms) const
{
  return speed_ms * params_.timestep_seconds;
}

double Planner::dist_inc_delta_at_accel(double accel) const
{
  return accel * params_.timestep_seconds * params_.timestep_seconds;
}

void Planner::set_desired_speed_kmh(double desired_speed)
{
  desired_speed_ms_ = desired_speed / 3.6;
}
