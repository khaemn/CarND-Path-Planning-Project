#include "planner.h"
#include "spline.h"

Planner::Planner(const RoadMap &map, const PlannerConstParams &params)
  : map_(map)
  , params_(params)
{}

Planner::State Planner::state() const
{
  return state_;
}

void Planner::process_telemetry(const nlohmann::json &telemetry)
{
  parse_ego(telemetry);

  parse_prev_path(telemetry);
  // Sensor Fusion Data, a list of all other cars on the same side
  //   of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  choose_next_state();

  generate_trajectory(telemetry);
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
  ego_.x     = telemetry["x"];
  ego_.y     = telemetry["y"];
  ego_.s     = telemetry["s"];
  ego_.d     = telemetry["d"];
  ego_.yaw   = telemetry["yaw"];
  ego_.speed = telemetry["speed"];

  update_current_lane();
}

void Planner::parse_prev_path(const nlohmann::json &telemetry)
{
  clear_prev_path();

  // Previous path data given to the Planner
  auto previous_path_x = telemetry["previous_path_x"];
  auto previous_path_y = telemetry["previous_path_y"];

  if (previous_path_x.size() != previous_path_y.size())
  {
    std::cout << "Error: previous path x size " << previous_path_x.size()
              << " != previous path y size " << previous_path_y.size() << std::endl;
  }
  std::copy(previous_path_x.begin(), previous_path_x.end(), prev_path_x_.begin());
  std::copy(previous_path_y.begin(), previous_path_y.end(), prev_path_y_.begin());

  // Previous path's end s and d values
  end_path_s_ = telemetry["end_path_s"];
  end_path_d_ = telemetry["end_path_d"];
}

void Planner::update_current_lane()
{
  current_lane_ = int(std::floor(ego_.d / params_.lane_width_m));
}

void Planner::generate_simple_keep_lane(const nlohmann::json &telemetry)
{
  static constexpr double s_step_m = 30.;
  static constexpr size_t steps_ahead = 3;
  clear_trajectory();

  const double dist_inc =
      (desired_speed_ms() * params_.trajectory_time_sec()) / params_.trajectory_length_pts;

  // We need (at least) 2 reference points in order to make the
  // path trajectory continuation tangent to the existing path.
  double ref_x(ego_.x), ref_y(ego_.y), prev_ref_x(ego_.x - cos(ego_.yaw)),
      prev_ref_y(ego_.y - sin(ego_.yaw)), ref_yaw(ego_.yaw);

  const auto prev_path_len = prev_path_x_.size();

  // if there are at least 2 pts in the prev path, the reference position
  // for the path should be at the end of the previous one; otherwise,
  // the ref position would be calculated from the CCP.
  if (prev_path_len >= 2)
  {
    ref_x      = prev_path_x_.at(prev_path_len - 1);
    ref_y      = prev_path_y_.at(prev_path_len - 1);
    prev_ref_x = prev_path_x_.at(prev_path_len - 2);
    prev_ref_y = prev_path_y_.at(prev_path_len - 2);
    ref_yaw    = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
  }

  auto ref_s_d = getFrenet(ref_x, ref_y, ref_yaw, map_.x, map_.y);
  const double ref_s = ref_s_d.at(0);
  const double ref_d = ref_s_d.at(1);
  const double d_center_offset = lane_center_d(current_lane_) - ref_d;
  const double d_step_m = d_center_offset / steps_ahead;

  vector<double> spline_keypts_x{prev_ref_x, ref_x};
  vector<double> spline_keypts_y{prev_ref_y, ref_y};

  // Add 'steps_ahead' more points along the lane, each next pt
  // closer to the lane center and further ahead, in map coords
  for (size_t i{1}; i <= steps_ahead; i++) {
    vector<double> coords =
        getXY(ref_s + i * s_step_m, ref_d + i * d_step_m, map_.s, map_.x, map_.y);
    spline_keypts_x.push_back(coords[0]);
    spline_keypts_y.push_back(coords[1]);
  }

  for (size_t i{0}; i < params_.trajectory_length_pts; ++i)
  {
    const double next_s = ego_.s + (i + 1) * dist_inc;
    const double next_d = lane_center_d(current_lane_);

    const auto map_coords = getXY(next_s, next_d, map_.s, map_.x, map_.y);
    trajectory_x_.push_back(float(map_coords[0]));
    trajectory_y_.push_back(float(map_coords[1]));
  }
}

void Planner::generate_best_keep_lane(const nlohmann::json &telemetry)
{
  // Generates the best possible keep lane trajectory.
  // Ensures safe stop before an obstacle, safe distance
  // to a moving obstacle ahead or a maximum allowed speed
  // on a free lane.

  // DUMMY
  generate_simple_keep_lane(telemetry);
}

void Planner::clear_trajectory()
{
  trajectory_x_.clear();
  trajectory_y_.clear();
}

void Planner::clear_prev_path()
{
  prev_path_x_.clear();
  prev_path_y_.clear();
  end_path_d_ = ego_.d;
  end_path_s_ = ego_.s;
}

double Planner::lane_center_d(int lane) const
{
  return params_.lane_width_m * (lane + 0.5);
}

void Planner::choose_next_state()
{
  state_ = State::KeepLane;
}

void Planner::generate_trajectory(const nlohmann::json &telemetry)
{
  switch (state_)
  {
  case State::KeepLane:
    generate_best_keep_lane(telemetry);
    break;
  default:
    clear_trajectory();
    break;
  }
}

double Planner::desired_speed_kmh() const
{
  return desired_speed_kmh_;
}

double Planner::desired_speed_ms() const
{
  return desired_speed_kmh_ / 3.6;
}

void Planner::set_desired_speed_kmh(double desired_speed)
{
  desired_speed_kmh_ = desired_speed;
}
