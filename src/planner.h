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
  double speed;
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

  explicit Planner(const RoadMap &map, double timestep_seconds, size_t trajectory_length_pts);

  State state() const;

  void process_telemetry(const nlohmann::json &telemetry);

  const std::vector<float> &x_trajectory_points() const;
  const std::vector<float> &y_trajectory_points() const;

  double desired_speed() const;
  void   set_desired_speed_kmh(double desired_speed);

private:
  void parse_ego(const nlohmann::json &telemetry);
  void choose_next_state();
  void generate_trajectory();

  // Trajectory generators
  void generate_keep_lane();

  // Utility
  void clear_trajectory();

private:
  State              state_{State::Invalid};
  std::vector<float> trajectory_x_;
  std::vector<float> trajectory_y_;
  Car                ego_;
  // According to requirements, the road map is immutable.
  const RoadMap map_;
  const double timestep_sec_;
  const size_t trajectory_length_pts_;
  double        desired_speed_kmh_{0.};
};
