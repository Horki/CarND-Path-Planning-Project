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
#include <tuple>

class PathPlanner {
private:
  int lane;
  double ref_val;
  const int num_lanes;
  const size_t points;
  const double speed_change;
  const double max_speed;
  const double time_step;
  const double miles_to_meters;

  double ref_x;
  double ref_y;
  double ref_yaw;

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  std::tuple<std::vector<double>, std::vector<double>>
  pts(double car_s, double car_x, double car_y, double car_yaw,
      const std::vector<double> & previous_path_x,
      const std::vector<double> & previous_path_y) {
    // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
    // Later we will interpolate those waypoints with a spline and fill it in
    // with more points that control speed
    int prev_size = previous_path_x.size();
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    // Reference x, y, yaw states
    ref_x   = car_x;
    ref_y   = car_y;
    ref_yaw = deg2rad(car_yaw);

    // If previous size is almost empty, use the car as starting reference
    if (prev_size < 2) {
      // Use two points that make the path tangent to the car
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      // X
      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);
      // Y
      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    } else {
      // Use previous path's end point as starting reference
      // Redefine reference state as previous path end point
      ref_x             = previous_path_x[prev_size - 1];
      ref_y             = previous_path_y[prev_size - 1];
      double prev_ref_x = previous_path_x[prev_size - 2];
      double prev_ref_y = previous_path_y[prev_size - 2];
      ref_yaw           = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
      // Use two points to make the path tangent to the previous path's end point
      // X
      ptsx.push_back(prev_ref_x);
      ptsx.push_back(ref_x);
      // Y
      ptsy.push_back(prev_ref_y);
      ptsy.push_back(ref_y);
    }
    // In freenet add evenly 30m spaced points ahead of the starting reference
    std::vector<std::vector<double>> next_wps;
    for (double deg = 30.0; deg <= 90.0; deg += 30.0) {
      next_wps.push_back(getXY(car_s + deg, // 30, 60, 90
                               (2 + 4 * lane),
                               map_waypoints_s,
                               map_waypoints_x,
                               map_waypoints_y));
    }

    for (const auto & next_wp : next_wps) {
      ptsx.push_back(next_wp[0]);
      ptsy.push_back(next_wp[1]);
    }

    for (size_t i = 0; i < ptsx.size(); ++i) {
      // Shift car reference angle to 0 degrees
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
      ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }
    return std::make_tuple(ptsx, ptsy);
  }

  void check(std::vector<std::vector<double>> const & sensor_fusion,
             int prev_size,
             double car_s) {
    bool same_lane_clear  = true;
    bool left_lane_clear  = lane != 0;
    bool right_lane_clear = lane != (num_lanes - 1);

    for (size_t i = 0; i < sensor_fusion.size(); ++i) {
      // Car is in my lane
      float d = sensor_fusion[i][6];
      bool in_same_lane = d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2);
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
      double check_car_s = sensor_fusion[i][5];
      check_car_s += (double) prev_size * time_step * check_speed;
      if (in_same_lane) {
        // check values greater than mine and s gap
        if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
          // Do some logic
          same_lane_clear = false;
        }
      } else {
        bool in_left_lane  = d < (4 * lane);
        bool getting_close = (check_car_s > car_s - 10)
          && (check_car_s - car_s < 30);
        if (in_left_lane && getting_close) {
          left_lane_clear = false;
        } else {
          if (getting_close) right_lane_clear = false;
        }
      }
    }
    if (same_lane_clear && ref_val < max_speed) {
      ref_val += speed_change;
      std::cout << "accelerate: " << ref_val << " mph\n";
    } else if (left_lane_clear) {
      --lane;
      std::cout << "moving to left lane\n";
    } else if (right_lane_clear) {
      ++lane;
      std::cout << "moving to right lane\n";
    } else {
      ref_val -= speed_change;
      std::cout << "slow: " << ref_val << " mph\n";
    }
  }
public:
  PathPlanner(const std::string & map_file) : points(50),
                                              num_lanes(3),
                                              speed_change(0.224),
                                              max_speed(49.5),
                                              time_step(0.02),
                                              miles_to_meters(2.24) {
    lane      = 1; // start lane
    ref_val   = 0.0; // mph, start speed
    std::cout << "Path Planner on!\n";
    std::cout << "Start speed: " << ref_val << " mph\n";
    std::cout << "Start lane: " << lane << std::endl;
    std::cout << "Max speed: " << max_speed << std::endl;

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
    int prev_size        = previous_path_x.size();
    if (prev_size > 0) { car_s = end_path_s; }

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"];

    check(sensor_fusion, prev_size, car_s);

    std::tuple<std::vector<double>, std::vector<double>> result =
      pts(car_s, car_x, car_y, car_yaw, previous_path_x, previous_path_y);
    std::vector<double> pts_x = std::get<0>(result);
    std::vector<double> pts_y = std::get<1>(result);
    // Create a Spline
    tk::spline sp;
    // Set (x, y) points to spline
    sp.set_points(pts_x, pts_y);

    // Define the actual (x, y) points we will use for the planner
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    // Start with all previous path points from last time
    for (size_t i = 0; i < previous_path_x.size(); ++i) {
      next_x_vals.push_back(previous_path_x.at(i));
      next_y_vals.push_back(previous_path_y.at(i));
    }

    // Calculate how to breakup spline points so that we travel at our
    // desired reference velocity
    double target_x    = 30.0;
    double target_y    = sp(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
    double x_add_on    = 0.0;

    // Fill up the rest of our path planner after filling it with previous
    // points, here we will always output 50 points
    for (size_t i = 1; i <= points - previous_path_x.size(); ++i) {
      double N       = target_dist / (time_step * ref_val / miles_to_meters);
      double x_point = x_add_on + target_x / N;
      double y_point = sp(x_point);
      x_add_on       = x_point;
      double x_ref   = x_point;
      double y_ref   = y_point;
      // Rotate back to normal after rotating it earlier
      x_point  = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point  = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
      x_point += ref_x;
      y_point += ref_y;
      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }

    nlohmann::json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    return msgJson;
  }
};

#endif
