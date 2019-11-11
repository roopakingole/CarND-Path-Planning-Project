#ifndef VEHICLE_H
#define VEHICLE_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "helpers.h"
#include "json.hpp"
#include "spline.h"



// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class Vehicle {
//Date
private:
	  vector<double> map_waypoints_x;
	  vector<double> map_waypoints_y;
	  vector<double> map_waypoints_s;
	  vector<double> map_waypoints_dx;
	  vector<double> map_waypoints_dy;
	  int lane = 1;
	  double ref_vel = 0.0; //mph

      // Main car's localization Data
      double car_x;
      double car_y;
      double car_s;
      double car_d;
      double car_yaw;
      double car_speed;

      // Previous path data given to the Planner
      vector<double> previous_path_x;
      vector<double> previous_path_y;
      // Previous path's end s and d values
      double end_path_s;
      double end_path_d;

      // Sensor Fusion Data, a list of all other cars on the same side of the road.
      vector<vector<double>> sensor_fusion;

      int prev_size;
      double speed_diff;
      const double MAX_SPEED = 49.5;
      const double MAX_ACC = .224;

      bool too_close = false;
      bool car_ahead = false;
      bool car_left = false;
      bool car_righ = false;

	  vector<double> next_x_vals;
	  vector<double> next_y_vals;

//Methods
public:
	Vehicle();
	Vehicle(  vector<double> map_wp_x,
	  vector<double> map_wp_y,
	  vector<double> map_wp_s,
	  vector<double> map_wp_dx,
	  vector<double> map_wp_dy);
	~Vehicle();
	void processTelemetry(json j);
	void setTelemetryData( double x, double y, double s ,double d,double yaw, double spd, double end_s, double end_d, vector<double> path_x, vector<double> path_y, vector<vector<double>> sf);
	void processSensorFusionData(void);
	void behaviorPlan(void);
	void generateTrajectory(void);
	vector<double> get_next_x(void) { return next_x_vals; }
	vector<double> get_next_y(void) { return next_y_vals; }
private:

};

#endif
