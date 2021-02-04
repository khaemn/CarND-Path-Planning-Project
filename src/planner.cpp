#include "planner.h"

Planner::Planner(const RoadMap &map, double timestep_seconds, size_t trajectory_length_pts)
  : map_(map)
  , timestep_sec_(timestep_seconds)
  , trajectory_length_pts_(trajectory_length_pts)
{}

Planner::State Planner::state() const
{
  return state_;
}

void Planner::process_telemetry(const nlohmann::json &telemetry)
{
  parse_ego(telemetry);

  // Previous path data given to the Planner
  auto previous_path_x = telemetry["previous_path_x"];
  auto previous_path_y = telemetry["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = telemetry["end_path_s"];
  double end_path_d = telemetry["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side
  //   of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  choose_next_state();

  generate_trajectory();
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
}

void Planner::generate_keep_lane()
{
  // TODO: correct impl! Now it is just a dummy.
  clear_trajectory();
  double dist_inc = 0.3;
  for (int i{0}; i < 50; ++i)
  {
    const double next_s = ego_.s + (i + 1) * dist_inc;
    const double next_d = 6;

    const auto map_coords = getXY(next_s, next_d, map_.s, map_.x, map_.y);
    trajectory_x_.push_back(float(map_coords[0]));
    trajectory_y_.push_back(float(map_coords[1]));
  }
}

void Planner::clear_trajectory()
{
  trajectory_x_.clear();
  trajectory_y_.clear();
}

void Planner::choose_next_state()
{
  state_ = State::KeepLane;
}

void Planner::generate_trajectory()
{
  switch (state_)
  {
  case State::KeepLane:
    generate_keep_lane();
    break;
  default:
    clear_trajectory();
    break;
  }
}

double Planner::desired_speed() const
{
  return desired_speed_kmh_;
}

void Planner::set_desired_speed_kmh(double desired_speed)
{
  desired_speed_kmh_ = desired_speed;
}
