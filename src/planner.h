#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <iostream>
#include <string>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

struct RoadMap
{
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct PlannerConstParams
{
  double timestep_seconds{0.02};
  size_t trajectory_length_pts{50};
  double lane_width_m{4.};
  size_t num_lanes{3};
  double max_planning_s{30};
  double max_planning_d{lane_width_m * 2};
  double comfort_longitudinal_accel{1.7};  // m per second squared

  double trajectory_time_sec() const
  {
    return trajectory_length_pts * timestep_seconds;
  }
};

struct RoadObject
{
};

struct Car
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed_kmh;
};

class Planner
{
public:
  enum class State
  {
    KeepLane,
    PrepareChangeLaneLeft,
    PrepareChangeLaneRight,
    ChangeLaneLeft,
    ChangeLaneRight,
    Invalid
  };

  explicit Planner(const RoadMap &map, const PlannerConstParams &params);

  State state() const;

  void process_telemetry(const nlohmann::json &telemetry);

  const std::vector<float> &x_trajectory_points() const;
  const std::vector<float> &y_trajectory_points() const;

  double desired_speed_kmh() const;
  double desired_speed_ms() const;

  double dist_inc_at_const_speed(double speed_ms) const;
  double dist_inc_delta_at_accel(double accel) const;

  void set_desired_speed_kmh(double desired_speed_kmh);

private:
  void parse_ego(const nlohmann::json &telemetry);
  void parse_prev_path(const nlohmann::json &telemetry);
  void update_allowed_speed(const nlohmann::json &telemetry);
  void update_current_lane();
  void choose_next_state();
  void generate_trajectory(const nlohmann::json &telemetry);

  // Trajectory generators
  void generate_simple_keep_lane();
  void generate_best_keep_lane();

  // Utility
  void   clear_trajectory();
  void   clear_prev_path();
  double lane_center_d(int lane) const;

private:
  State         state_{State::Invalid};
  vector<float> trajectory_x_;
  vector<float> trajectory_y_;
  Car           ego_;
  // According to requirements, the road map is immutable.
  const RoadMap      map_;
  PlannerConstParams params_;
  double             desired_speed_ms_{0.};
  double             allowed_now_speed_ms_{0.};

  double end_path_d_{0.};
  double end_path_s_{0.};

  // 0 for the leftmost lane, params_.num_lanes - 1 for the rightmost
  int current_lane_{0};
};
