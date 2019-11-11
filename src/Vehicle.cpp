#include "Vehicle.h"

extern vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);
extern double deg2rad(double x);

Vehicle::Vehicle() {

}

Vehicle::Vehicle(  vector<double> map_wp_x,
		  vector<double> map_wp_y,
		  vector<double> map_wp_s,
		  vector<double> map_wp_dx,
		  vector<double> map_wp_dy) {
	 map_waypoints_x = map_wp_x;
	 map_waypoints_y = map_wp_y;
	 map_waypoints_s = map_wp_s;
	 map_waypoints_dx = map_wp_dx;
	 map_waypoints_dy = map_wp_dy;
	 speed_diff = 0;

#ifdef _DEBUG
	 printf("%d %d %d %d %d\n", map_wp_x.size(), map_wp_y.size(), map_wp_s.size(), map_wp_dx.size(), map_wp_dy.size());
	 printf("%d %d %d %d %d\n", map_waypoints_x.size(), map_waypoints_y.size(), map_waypoints_s.size(), map_waypoints_dx.size(), map_waypoints_dy.size());
#endif
}

Vehicle::~Vehicle() {

}

void Vehicle::setTelemetryData( double x, double y, double s ,double d,double yaw, double spd, double end_s, double end_d, vector<double> path_x, vector<double> path_y, vector<vector<double>> sf) {
	car_x = x;
	car_y = y;
	car_s = s;
	car_d = d;
	car_yaw = yaw;
	car_speed = spd;
	end_path_s = end_s;
	end_path_d = end_d;

	previous_path_x.clear();
	previous_path_y.clear();
	sensor_fusion.clear();

	previous_path_x = path_x;
	previous_path_y = path_y;
	sensor_fusion = sf;

#ifdef _DEBUG
    std::cout << " x:" << car_x
			 << " y:" << car_y
			 << " s:" << car_s
			 << " d:" << car_d
			 << " yaw:" << car_yaw
			 << " spd:" << car_speed
			 << " pXsz:" << previous_path_x.size()
			 << " pYsz:" << previous_path_y.size()
			 << " SFsz:" << sensor_fusion.size() << std::endl;
#endif
	prev_size = previous_path_x.size();

    if(prev_size > 0)
    {
    	car_s = end_path_s;
    }

}


void Vehicle::processTelemetry(json j) {
	previous_path_x.clear();
	previous_path_y.clear();
	sensor_fusion.clear();

	// Ego car localization Data
	car_x = j[1]["x"];
	car_y = j[1]["y"];
	car_s = j[1]["s"];
	car_d = j[1]["d"];
	car_yaw = j[1]["yaw"];
	car_speed = j[1]["speed"];

    // Previous path data given to the Planner
	auto path_x = j[1]["previous_path_x"];
	for(int i = 0; i < path_x.size(); i++) { previous_path_x.push_back(path_x[i]); }
	auto path_y = j[1]["previous_path_y"];
	for(int i = 0; i < path_y.size(); i++) { previous_path_y.push_back(path_x[i]); }

	// Previous path's end s and d values
	end_path_s = j[1]["end_path_s"];
	end_path_d = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
	auto sFusion = j[1]["sensor_fusion"];
	for(int i = 0; i < sFusion.size(); i++) { sensor_fusion.push_back(sFusion[i]); }

	prev_size = previous_path_x.size();

    if(prev_size > 0)
    {
    	car_s = end_path_s;
    }

    std::cout << " x:" << car_x
			 << " y:" << car_y
			 << " s:" << car_s
			 << " d:" << car_d
			 << " yaw:" << car_yaw
			 << " spd:" << car_speed
			 << " pXsz:" << previous_path_x.size()
			 << " pYsz:" << previous_path_y.size()
			 << " SFsz:" << sensor_fusion.size() << std::endl;

}

void Vehicle::processSensorFusionData(void) {
	car_ahead = false;
	car_left = false;
	car_righ = false;

    //Process all the vehicles reported by Sensor Fusion.
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
  	  //find which lane car is riding
  	  float d = sensor_fusion[i][6];
        int car_lane = -1;
        // is it on the same lane we are
        if ( d > 0 && d < 4 ) {
          car_lane = 0;
        } else if ( d > 4 && d < 8 ) {
          car_lane = 1;
        } else if ( d > 8 && d < 12 ) {
          car_lane = 2;
        }
        if (car_lane < 0) {
          continue;
        }

        // Find car speed.
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // Estimate car s position after executing previous trajectory.
        check_car_s += ((double)prev_size*0.02*check_speed);

        if ( car_lane == lane ) {
          // Car in our lane.
          car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;        // Is car's s ahead of ego's s AND distance between the two is < 30m
        } else if ( car_lane - lane == -1 ) {
          // Car left
          car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;    // Is car's s between +-30m of ego's s.
        } else if ( car_lane - lane == 1 ) {
          // Car right
          car_righ |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;    // Is car's s between +-30m of ego's s.
        }

    }
#ifdef _DEBUG
    std::cout << " ahead:" << car_ahead
    		  << " left:"  << car_left
			  << " right:" << car_righ
			  << std::endl;
#endif
}

void Vehicle::behaviorPlan(void) {

	// Define behavior of Ego vehicle based on localize information.
	speed_diff = 0;

	//Simplistic Lane state machine
	//First check if lane change possible without reducing the speed.
    if ( car_ahead ) { // Is Car ahead of Ego vehicle
    	//Lane shift
        if ( !car_left && lane > 0 ) { // if there is no car in left lane
        	lane--; // Change lane left.
      } else if ( !car_righ && lane != 2 ){
        // if there is no car right and there is a right lane.
    	  lane++; // Change lane right.
      } else {
    	//or slow down
        speed_diff -= MAX_ACC;
      }
    } else {
      if ( lane != 1 ) { // if we are not on the center lane.
        if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
          lane = 1; // Back to center.
        }
      }
      if ( ref_vel < MAX_SPEED ) {
        speed_diff += MAX_ACC;
      }
    }
}

void Vehicle::generateTrajectory(void){

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

  	next_x_vals.clear();
  	next_y_vals.clear();

    // Do I have have previous points
    if ( prev_size < 2 ) {
        // There are not too many...
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        // Use the last two points.
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Setting up target points in the future.
    vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Making coordinates to local car coordinates.
    for ( int i = 0; i < ptsx.size(); i++ ) {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
      ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // Create the spline.
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // Output path points from previous path for continuity.
    for ( int i = 0; i < prev_size; i++ ) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate distance y position on 30 m ahead.
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;

    for( int i = 1; i < 50 - prev_size; i++ ) {
      ref_vel += speed_diff;
      if ( ref_vel > MAX_SPEED ) {
        ref_vel = MAX_SPEED;
      } else if ( ref_vel < MAX_ACC ) {
        ref_vel = MAX_ACC;
      }

      double N = target_dist/(0.02*ref_vel/2.24);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }

}
