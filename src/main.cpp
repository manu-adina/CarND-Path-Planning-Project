#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using namespace std;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Starting ego_car_lane of the vehicle. '1' is a middle ego_car_lane.
  int ego_car_lane = 1;

  // Velocity
  double ref_vel = 0.0;
  double speed_step = .224;
	const double max_speed = 49.5;

  h.onMessage([&ref_vel, &speed_step, &max_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ego_car_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if(prev_size > 0) {
            car_s = end_path_s;
          }

          /** A lot of the starting code was taken from the Udacity's video tutorial. 
           * Source: https://youtu.be/7sI3VHFPP0w
           * The ideas for lane changes were taken from a blog "Path planning project — Udacity’s self-driving car nanodegree"
           * Source: https://medium.com/intro-to-artificial-intelligence/path-planning-project-udacitys-self-driving-car-nanodegree-be1f531cc4f7
           * Source Author: Dhanoop Karunakaran
          **/

          // Prediction. 
          bool neighbour_car_ahead = false;
          bool neighbour_car_left = false;
          bool neighbour_car_right = false;

          for(int i = 0; i < sensor_fusion.size(); i++) {

            float d = sensor_fusion[i][6];
            
            int neighbour_car_lane = -1;

            // Checking on what lane the neighbour car is.
            if(d > 0 && d < 4) {
              neighbour_car_lane = 0; // Left lane = 0
            } else if(d > 4 && d < 8) {
              neighbour_car_lane = 1; // Middle lane = 1
            } else if(d > 8 and d < 12) {
              neighbour_car_lane = 2; // Right lane = 2
            }

            // Neighbouring car's velocity components.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double neighbour_car_s = sensor_fusion[i][5];

            // Predict the neighboring car's position in the future.
            neighbour_car_s += ((double)prev_size*0.02*check_speed);

            // Decide which move to execute.
            if(neighbour_car_lane == ego_car_lane) {
              // Neighbour car is on the same lane as ego car, and will be too close.
              neighbour_car_ahead |= neighbour_car_s > car_s && (neighbour_car_s - car_s) < 30;										
            } else if((neighbour_car_lane - ego_car_lane) == -1) {
              // Neighbour car is on the left lane and check if it's within 30m
              neighbour_car_left |= (car_s + 30) > neighbour_car_s  && (car_s - 30) < neighbour_car_s;
            } else if((neighbour_car_lane - ego_car_lane) == 1) {
              // Neighbour car is on the right lane, and check if it's within 30m
              neighbour_car_right |= (car_s + 30) > neighbour_car_s  && (car_s - 30) < neighbour_car_s;
            }
          }

          // Using the information on where the neightbour car is, decide on the action.
					if(neighbour_car_ahead) {
            // lane change left, if there isn't a neighbour car to the left
						if(!neighbour_car_left && ego_car_lane > 0) {
							ego_car_lane--;
            // lane change right, if there isn't a neighbour car to the right
						} else if(!neighbour_car_right && ego_car_lane !=2) {
							ego_car_lane++;
            // Slow down if there is a car ahead and cant change left or right
						} else {
							ref_vel -= speed_step;
						}
          // Speed up if there are no cars ahead
					} else if(ref_vel < max_speed) {
						ref_vel += speed_step;
					}

          // The code was provided by Udacity's Tutorial, as referenced earlier.
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);

            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            ptsx.push_back(prev_ref_x);
            ptsy.push_back(prev_ref_y);

            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);

          }

          vector<double> next_wp_0 = getXY(car_s + 30, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_1 = getXY(car_s + 60, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_2 = getXY(car_s + 90, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp_0[0]);
          ptsy.push_back(next_wp_0[1]);

          ptsx.push_back(next_wp_1[0]);
          ptsy.push_back(next_wp_1[1]);

          ptsx.push_back(next_wp_2[0]);
          ptsy.push_back(next_wp_2[1]);

          for(int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

          double x_add_on = 0;

          for(int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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