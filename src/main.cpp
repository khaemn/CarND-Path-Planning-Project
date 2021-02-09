#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  RoadMap road_map;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  if (not in_map_.is_open())
  {
    std::cout << "Can not open map file at " << map_file_ << ", impossible to operate without it."
              << std::endl;
    std::exit(1);
  }

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    road_map.x.push_back(x);
    road_map.y.push_back(y);
    road_map.s.push_back(s);
    road_map.dx.push_back(d_x);
    road_map.dy.push_back(d_y);
  }

  PlannerParams params;

  Planner planner(road_map, params);
  planner.set_desired_speed_kmh(78.5);

  h.onMessage([&planner]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    const bool is_message_correct = length && length > 2 && data[0] == '4' && data[1] == '2';
    if (not is_message_correct)
    {
      return;
    }

    auto s = hasData(data);

    if (s.empty())
    {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    auto j = json::parse(s);

    string event_type = j[0].get<string>();

    if (event_type != "telemetry")
    {
      return;
    }

    // This call performs all the job with trajectory planning.
    planner.process_telemetry(j[1]);

    json msgJson;

    msgJson["next_x"] = planner.x_trajectory_points();
    msgJson["next_y"] = planner.y_trajectory_points();

    auto msg = "42[\"control\"," + msgJson.dump() + "]";

    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
