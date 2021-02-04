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

  State state() const;

  void process(nlohmann::json data);

  const std::vector<float>& x_trajectory_points() const;
  const std::vector<float>& y_trajectory_points() const;

private:
  State state_{State::Invalid};
  std::vector<float> x_;
  std::vector<float> y_;
};
