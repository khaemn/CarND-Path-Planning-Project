#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <iostream>
#include <set>
#include <vector>

/// A convenient wrapper around several data vectors,
/// holding the waypoint coordinates in different coordinate
/// systems
struct RoadMap
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

struct PlannerParams
{
  // Planning algorithm parameters
  double timestep_seconds{0.02};
  size_t trajectory_length_pts{50};
  size_t min_time_between_lane_changes_sec{2};

  // Highway parameters
  size_t total_lanes{3};
  int    preferred_lane{1};
  double lane_width_m{4.};

  // Ego car drivetrain parameters
  double comfort_longitudinal_accel_ms2{4.};
  double max_braking_deceleration_ms2{8.};
  double min_possible_speed_ms{0.05};

  // Ego car approximate size
  double ego_length_m{5.0};
  double ego_width_m{3.0};

  // Longitudinal (along 's' Frenet axis) gaps
  double safe_gap_lon{8 * ego_length_m};
  double min_gap_lon{2 * ego_length_m};
};

/// A wrapper for the rest of cars or other objects on the road
struct RoadObject
{
  RoadObject() = default;
  RoadObject(int id, double x, double y, double vx, double vy, double s, double d, double dist_ccp)
    : id(id)
    , x(x)
    , y(y)
    , vx(vx)
    , vy(vy)
    , s(s)
    , d(d)
    , distance_to_ccp(dist_ccp)
  {}
  int    id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double distance_to_ccp;
};
inline bool operator<(const RoadObject &l, const RoadObject &r)
{
  return l.distance_to_ccp < r.distance_to_ccp;
}

/// A wrapper for the ego car object
struct Car
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed_ms;
};

/// Implements the behavior planning, e.g. trajectory points
/// generation according to the road surrounding and given parameters.
class Planner
{
public:
  explicit Planner(const RoadMap &map, const PlannerParams &params);

  /// Main planner interface, updates the internal state and
  /// generates new trajectory.
  void process_telemetry(const nlohmann::json &telemetry);

  /// Retrievers for the new trajectory points.
  const std::vector<float> &x_trajectory_points() const;
  const std::vector<float> &y_trajectory_points() const;

  /// Sets the speed the planner will try to keep the car at
  void set_desired_speed_kmh(double desired_speed_kmh);

private:
  void parse_ego(const nlohmann::json &telemetry);
  void parse_previous_path(const nlohmann::json &telemetry);
  void parse_obstacles(const nlohmann::json &telemetry);
  void update_allowed_speed();
  void update_current_lane();
  void update_lane_change_counter();

  void generate_trajectory();
  int  choose_best_lane();

  // Utility
  void   clear_trajectory();
  void   alarm_crop_trajectory();
  void   clear_obstacles();
  double dist_inc_at_speed(double speed_ms) const;
  double dist_inc_delta_at_accel(double accel) const;
  double lane_center_d(int lane) const;
  int    lane_num_of(double d);
  double obstacle_speed(const RoadObject &object);

  inline double speed_factor() const;
  inline bool   was_recent_lane_change() const;

private:
  vector<float> trajectory_x_;
  vector<float> trajectory_y_;
  Car           ego_;

  // According to requirements, the road map is immutable.
  const RoadMap map_;
  PlannerParams params_;

  vector<std::set<RoadObject>> obstacles_ahead_;
  vector<std::set<RoadObject>> obstacles_behind_;

  double end_path_d_{0.};
  double end_path_s_{0.};

  double desired_speed_ms_{0.};
  double allowed_now_speed_ms_{0.};

  int    current_lane_{1};  // 0 for the leftmost lane
  int    future_lane_{current_lane_};
  size_t lane_change_counter_{0};
  bool   is_slowed_down_by_obstacle_ahead{false};
};
