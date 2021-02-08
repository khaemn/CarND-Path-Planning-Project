#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <set>

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
  double comfort_longitudinal_accel{3/*2.5*/};  // m per second squared
  double ego_length_m{5.0};
  double ego_width_m{3.0};
  double safe_gap_lon{6/*7*/ * ego_length_m};
  double min_gap_lon{1.5 * ego_length_m};
  double safe_gap_lat{1.2 * ego_width_m};
  double min_gap_lat{ego_width_m};
  int preferred_lane{1};
  double trajectory_time_sec() const
  {
    return trajectory_length_pts * timestep_seconds;
  }
};

struct RoadObject
{
  const int    id;
  const double x;
  const double y;
  const double vx;
  const double vy;
  const double s;
  const double d;
  const double distance_to_ccp;
};
inline bool operator<(const RoadObject &l, const RoadObject &r)
{
  return l.distance_to_ccp < r.distance_to_ccp;
}

struct Car
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed_ms;
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
  void parse_obstacles(const nlohmann::json &telemetry);
  void update_allowed_speed();
  void update_current_lane();
  void choose_next_state();
  void generate_trajectory(const nlohmann::json &telemetry);

  // Trajectory generators
  void generate_simple_keep_lane();
  void generate_keep_lane();
  int decide_best_lane();

  // Utility
  void   clear_trajectory();
  void   clear_prev_path();
  void   clear_obstacles();
  double lane_center_d(int lane) const;
  int lane_num_of(double d);
  double obstacle_speed_along_my_heading(const RoadObject& object);

private:
  State         state_{State::Invalid};
  vector<float> trajectory_x_;
  vector<float> trajectory_y_;
  Car           ego_;
  vector<std::set<RoadObject>> obstacles_ahead_;
  vector<std::set<RoadObject>> obstacles_behind_;
  // According to requirements, the road map is immutable.
  const RoadMap      map_;
  PlannerConstParams params_;
  double             desired_speed_ms_{0.};
  double             allowed_now_speed_ms_{0.};

  double end_path_d_{0.};
  double end_path_s_{0.};

  // 0 for the leftmost lane, params_.num_lanes - 1 for the rightmost
  int current_lane_{0};
  bool is_slowed_down_by_obstacle_ahead{false};
};
