#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "spline.h"
#include "json.hpp"
#include "helpers.h"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

class PathPlanner {
private:
  int lane;
  int num_lanes;
  double dist_inc;

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

public:
  PathPlanner(const std::string & map_file) {
    lane      = 1;
    num_lanes = 3;
    dist_inc  = 0.3;

    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);
    std::string line;
    while (getline(in_map_, line)) {
      std::istringstream iss(line);
      double x, y;
      float  s, d_x, d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map_waypoints_x.push_back(x);
      map_waypoints_y.push_back(y);
      map_waypoints_s.push_back(s);
      map_waypoints_dx.push_back(d_x);
      map_waypoints_dy.push_back(d_y);
    }
  }

  ~PathPlanner() {}

  nlohmann::json parse(const std::string & data) {
    auto j = nlohmann::json::parse(data);
    // Main car's localization Data
    double car_x     = j[1]["x"];
    double car_y     = j[1]["y"];
    double car_s     = j[1]["s"];
    double car_d     = j[1]["d"];
    double car_yaw   = j[1]["yaw"];
    double car_speed = j[1]["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = j[1]["previous_path_x"];
    auto previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s    = j[1]["end_path_s"];
    double end_path_d    = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    auto sensor_fusion   = j[1]["sensor_fusion"];

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    for (int i = 0; i < 50; ++i) {
      double next_s = car_s + (i + 1) * dist_inc;
      double next_d = 6;
      std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s,
                                     map_waypoints_x, map_waypoints_y);
      next_x_vals.push_back(xy[0]);
      next_y_vals.push_back(xy[1]);
    }

    nlohmann::json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    return msgJson;
  }
};

#endif
