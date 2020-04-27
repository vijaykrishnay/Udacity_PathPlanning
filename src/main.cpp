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

#define mps_to_mph 2.23
#define speed_limit 49
#define acc_limit 9.5
#define dt 0.02
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double pos_x_prev;
          double pos_y_prev;
          double speed_prev_mps;

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            
            pos_x_prev = pos_x - cos(angle);
            pos_y_prev = pos_y - sin(angle);
            speed_prev_mps = car_speed / mps_to_mph;
            // std::cout<<"path size=0"<<std::endl;
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            pos_x_prev = previous_path_x[path_size-2];
            pos_y_prev = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y_prev, pos_x-pos_x_prev);
            speed_prev_mps = distance(pos_x, pos_y, pos_x_prev, pos_y_prev) / dt;
            // std::cout<<"path size>0"<<std::endl;
          }

          // get car position and lane
          vector<double> s_d = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          double lane_d = s_d[1];
          if ((lane_d > 0) & (lane_d <= 4.)){
            lane_d = 2.;
          }else if ((lane_d > 4) & (lane_d <= 8.)){
            lane_d = 6.;
          }
          else if ((lane_d > 8) & (lane_d <= 12.)){
            lane_d = 10.;
          }

          // calculate distance increments for path
          double dist_inc = speed_limit * dt / mps_to_mph;
          vector <double> dist_inc_list;
          for (int i = 0; i < 50-path_size; ++i) {
            // ADJUST DIST INC FOR SMOOTH ACCEL
            double acc_proj = (speed_limit/mps_to_mph - speed_prev_mps)/dt;
            if (acc_proj > acc_limit){
              // s = ut + 1/2at^2
              dist_inc = speed_prev_mps * dt + 0.5 * acc_limit * pow(dt, 2);
              std::cout<<"car speed adjusted for acc"<<speed_prev_mps<<std::endl;
            }

            // ADJUST DIST INC TO AVOID COLLISIONS IN SAME LANE
            double max_dist_in_lane = dist_inc;
            double ego_i_s = s_d[0];
            for (int j=0; j<sensor_fusion.size(); j++){
              double car_j_s = sensor_fusion[j][5];
              double car_j_d = sensor_fusion[j][6];
              double vx_j = sensor_fusion[j][3];
              double vy_j = sensor_fusion[j][4];
              double car_j_v = pow(pow(vx_j, 2) + pow(vy_j, 2), 0.5);
              if ((car_j_d > lane_d - 2.) & ((car_j_d < lane_d + 2.))){
                if (((ego_i_s + dist_inc) > (car_j_s + car_j_v * dt * path_size - 50.)) & 
                      ((ego_i_s + dist_inc) < (car_j_s + car_j_v * dt * path_size))){
                  if ((dist_inc/dt) > car_j_v){
                    dist_inc = std::max(speed_prev_mps * dt - 0.5 * acc_limit * pow(dt, 2), car_j_v * dt);
                    std::cout<<"car j d"<<car_j_d<<std::endl;
                    std::cout<<"lane d"<<lane_d<<std::endl;
                    std::cout<<"car j s"<<car_j_s<<std::endl;
                    std::cout<<"ego s"<<ego_i_s<<std::endl;
                    std::cout<<"car j v"<<car_j_v<<std::endl;
                  }
                }
              }
            }

            speed_prev_mps = dist_inc / dt;
            ego_i_s += dist_inc;
            dist_inc_list.push_back(dist_inc);
            // std::cout<<"dist inc "<<dist_inc<<std::endl;
          }

          
          vector<double> xy_15 = getXY(s_d[0]+15, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> xy_30 = getXY(s_d[0]+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> xy_45 = getXY(s_d[0]+45, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> xpts;
          vector<double> ypts;
          xpts.push_back(pos_x_prev);
          xpts.push_back(pos_x);          
          xpts.push_back(xy_15[0]);
          xpts.push_back(xy_30[0]);
          xpts.push_back(xy_45[0]);
          ypts.push_back(pos_y_prev);
          ypts.push_back(pos_y);
          ypts.push_back(xy_15[1]);
          ypts.push_back(xy_30[1]);
          ypts.push_back(xy_45[1]);
          
          int npts = xpts.size();
          // Convert to local coordinate system. This prevents multiple y values for the same x?(for short distances)
          for(int i=0; i<npts; i++){
            double shift_x = xpts[i] - pos_x;
            double shift_y = ypts[i] - pos_y;
            xpts[i] = (shift_x*cos(0. - angle) - shift_y*sin(0. - angle));
            ypts[i] = (shift_x*sin(0. - angle) + shift_y*cos(0. - angle));
            // std::cout<<"x "<<xpts[i]<<std::endl;
            // std::cout<<"y "<<ypts[i]<<std::endl;
          }

          tk::spline s;
          s.set_points(xpts, ypts);
          double dist = distance(xpts[1], ypts[1], xpts[npts-1], ypts[npts-1]);
          
          // std::cout<<"car speed="<<speed_prev_mps<<std::endl;

          for (int i = 0; i < 50-path_size; ++i) {
            double dx_i = (i+1)*(dist_inc_list[i]/dist) * (xpts[npts-1] - xpts[1]);
            double x_i = xpts[1]+dx_i;
            double y_i = s(x_i);

            double x_i_glob = pos_x + x_i*cos(angle) - y_i*sin(angle);
            double y_i_glob = pos_y + x_i*sin(angle) + y_i*cos(angle);
            next_x_vals.push_back(x_i_glob);
            next_y_vals.push_back(y_i_glob);
          }
          dist_inc_list.clear();
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