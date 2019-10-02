#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <fstream>
#include <cassert>

#include "helpers.h"
#include "json.hpp"
#include "spline.h"

static const double MAX_SPEED = 49.5;
static const double MAX_SPEED_CHANGE = .224; // About 5 m/s^2 accelleration
static const double MPH_TO_METERS = 2.24;

static const int NUM_LANES = 3; // FYI: Lanes are indexed at 0.
static const double LANE_WIDTH = 4.; // in meters, useful for d part of Frenet coordinates
static const double HALF_LANE_WIDTH = LANE_WIDTH / 2.; // to avoid having to compute /2 everytime.

static const int NUM_POINTS = 50; // Number of points to use in path
static const double TARGET_DISTANCE = 30.; // How far to look ahead with path calc.
static const double SIMULATOR_TIME_STEP = .02; // Num seconds between each point that the simulator


class Data {
private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
public:
  Data(std::string const & filename) {
    std::ifstream in_map_(filename.c_str(), std::ifstream::in);
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
    assert(map_waypoints_x.size()  == 181);
    assert(map_waypoints_y.size()  == 181);
    assert(map_waypoints_s.size()  == 181);
    assert(map_waypoints_dx.size() == 181);
    assert(map_waypoints_dy.size() == 181);
  }
  std::vector<double> get_s() { return map_waypoints_s; }
  std::vector<double> get_x() { return map_waypoints_x; }
  std::vector<double> get_y() { return map_waypoints_y; }
};

void determine_lane_and_velocity(nlohmann::json const & data, int &lane, double &ref_velocity);
std::tuple<std::vector<double>, std::vector<double>>
generate_trajectory_for_lane(nlohmann::json const & data,
                             Data & map,
                             const int lane,
                             const double ref_velocity);

nlohmann::json parse_data(Data & map, std::string const & data, int & lane, double & ref_val) {
  auto j = nlohmann::json::parse(data);

  determine_lane_and_velocity(j[1], lane, ref_val);
  std::tuple<std::vector<double>, std::vector<double>> result =
    generate_trajectory_for_lane(j[1], map, lane, ref_val);
  std::vector<double> next_x = std::get<0>(result);
  std::vector<double> next_y = std::get<1>(result);

  nlohmann::json msgJson;
  msgJson["next_x"] = next_x;
  msgJson["next_y"] = next_y;

  return msgJson;
}

void determine_lane_and_velocity(nlohmann::json const & data,
                                 int & lane,
                                 double & ref_velocity) {
  double car_s = data["s"];
  auto previous_path_x = data["previous_path_x"];
  auto previous_path_y = data["previous_path_y"];
  double end_path_s = data["end_path_s"];

  auto sensor_fusion = data["sensor_fusion"]; // format is [ id, x, y, vx, vy, s, d]

  int prev_size = previous_path_x.size();

  double last_s = prev_size > 0 ? end_path_s : car_s;

  bool same_lane_clear  = true;
  bool left_lane_clear  = lane != 0;
  bool right_lane_clear = lane != 3 - 1;

  for (auto cur_sense : sensor_fusion) {
    float d = cur_sense[6];
    double v_x = cur_sense[3];
    double v_y = cur_sense[4];
    double check_speed = sqrt(v_x * v_x + v_y * v_y);
    double check_car_s = cur_sense[5];
    check_car_s += (double) prev_size * 0.02 * check_speed;
    bool in_same_lane = d < (4.0 + 4.0 * lane) && d > 4.0 * lane;
    if (in_same_lane) {
      bool getting_close = check_car_s > last_s && (check_car_s - last_s) < 30.0;
      if (getting_close) {
        same_lane_clear = false;
      }
    } else {
      bool in_left_lane  = d < 4.0 * lane;

      bool getting_close = check_car_s > last_s - 10.0
                          && check_car_s - last_s < 30.0;
      if (in_left_lane) {
        if (getting_close) {
          left_lane_clear = false;
        }
      } else {
        if (getting_close) {
          right_lane_clear = false;
        }
      }
    }
  }

  if (same_lane_clear) {
    if (ref_velocity < 49.5) {
      ref_velocity += 0.224;
      std::cout << "accelerate: " << ref_velocity << " mph\n";
    } else {
      ref_velocity -= 0.224;
      std::cout << "slow in same lane: " << ref_velocity << " mph\n";
    }
  } else if (left_lane_clear) {
    --lane;
    std::cout << "moving to left lane\n";
  } else if (right_lane_clear) {
    ++lane;
    std::cout << "moving to right lane\n";
  } else {
    ref_velocity -= 0.224;
    std::cout << "slow: " << ref_velocity << " mph\n";
  }
}

std::tuple<std::vector<double>, std::vector<double>>
generate_trajectory_for_lane(nlohmann::json const & data,
                             Data & map,
                             const int lane,
                             const double ref_velocity) {

  // Main car's localization Data
  double car_x = data["x"];
  double car_y = data["y"];
  double car_s = data["s"];
  double car_yaw = data["yaw"];

  // Previous path data given to the Planner
  auto previous_path_x = data["previous_path_x"];
  auto previous_path_y = data["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = data["end_path_s"];
  int prev_size = previous_path_x.size();
  double last_s = prev_size > 0 ? end_path_s : car_s;
  // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
  // Later we will interpolate those waypoints with a spline and fill it in
  // with more points that control speed
  std::vector<double> pts_x;
  std::vector<double> pts_y;

  // Reference x, y, yaw states
  double ref_x;
  double ref_y;
  double ref_yaw;

  // If previous size is almost empty, use the car as starting reference
  if (prev_size < 2) {
    ref_x   = car_x;
    ref_y   = car_y;
    ref_yaw = deg2rad(car_yaw);
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(car_x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);
    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y);
  }

  std::vector<std::vector<double>> next_wps;
  for (double deg = 30.0; deg <= 90.0; deg += 30.0) {
    next_wps.push_back(getXY(last_s + deg, // 30, 60, 90
                              (2.0 + 4.0 * lane),
                              map.get_s(),
                              map.get_x(),
                              map.get_y()));
  }

  for (const auto & next_wp : next_wps) {
    pts_x.push_back(next_wp[0]);
    pts_y.push_back(next_wp[1]);
  }
  // Transform to local car coordinates
  for (int i = 0; i < pts_x.size(); ++i) {
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  tk::spline sp;
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
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;
  for (int i = 1; i <= (50 - prev_size); ++i) {
    double N = target_dist / (0.02 * ref_velocity / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = sp(x_point);
    x_add_on = x_point;
    double local_x_ref = x_point;
    double local_y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = local_x_ref * cos(ref_yaw) - local_y_ref * sin(ref_yaw);
    y_point = local_x_ref * sin(ref_yaw) + local_y_ref * cos(ref_yaw);

    // Very poor naming from Q&A, x_ref looks a lot like ref_x, was stuck on that for a little!
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return std::make_tuple(next_x_vals, next_y_vals);
}
