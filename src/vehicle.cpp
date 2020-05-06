#include "vehicle.h"
#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include "spline.h"
#include "helpers.h"
#include "json.hpp"

using std::string;
using std::vector;
using nlohmann::json;

/**
 * Constructor.
 */
Vehicle::Vehicle() {}
SensorFusion::SensorFusion(vec_dbl data_){
    data = data_;
};


/**
 * Destructor.
 */
Vehicle::~Vehicle() {}

void Vehicle::update_position(const json& data){
    curr_x = data["x"];
    curr_y = data["y"];
    curr_s = data["s"];
    curr_d = data["d"];
    curr_yaw = data["yaw"];
    curr_speed = data["speed"];
    end_path_s = data["end_path_s"];
    end_path_d = data["end_path_d"];
    previous_path_x = data["previous_path_x"].get<vec_dbl>();
    previous_path_y = data["previous_path_y"].get<vec_dbl>();
    sensor_data.clear();
    for(int i=0; i<data["sensor_fusion"].size(); ++i){
        SensorFusion sf(data["sensor_fusion"][i].get<vec_dbl>());
        sensor_data.push_back(sf);
    }

    path_size = previous_path_x.size();
    // std::cout<<"path size "<<path_size<<std::endl;
    // // Cap path size to add at leas 5 points per update. Fewer points may result in weird lane change behavior
    // if (path_size > (NUM_POINTS - 5)){
    //   path_size = NUM_POINTS - 5;
    // }

    // CALCULATE END, END PREV POSITIONS
    if (path_size == 0) {
      end_x = curr_x;
      end_y = curr_y;
      end_angle = deg2rad(curr_yaw);
      end_path_s = curr_s;
      end_path_d = curr_d;
      
      end_x_prev = end_x - cos(end_angle);
      end_y_prev = end_y - sin(end_angle);
      end_speed_prev_mps = curr_speed / MPS_TO_MPH;
      // std::cout<<"path size=0"<<std::endl;
    } else {
      end_x = previous_path_x[path_size-1];
      end_y = previous_path_y[path_size-1];

      end_x_prev = previous_path_x[path_size-2];
      end_y_prev = previous_path_y[path_size-2];
      end_angle = atan2(end_y-end_y_prev, end_x-end_x_prev);
      end_speed_prev_mps = distance(end_x, end_y, end_x_prev, end_y_prev) / dt;
      // std::cout<<"path size: "<<path_size<<std::endl;
    }

    // CALCULATE LANES
    curr_lane = infer_lane(curr_d);
    end_lane = infer_lane(end_path_d);
    // std::cout<<"curr lane = "<<curr_lane<<std::endl;
}

void Vehicle::plan_new_path(vec_dbl map_waypoints_x, vec_dbl map_waypoints_y, vec_dbl map_waypoints_s){
    next_path_x.clear();
    next_path_y.clear();

    for (int i = 0; i < path_size; ++i) {
      next_path_x.push_back(previous_path_x[i]);
      next_path_y.push_back(previous_path_y[i]);
    }

    vec2d_dbl dist_inc_lists;
    vec_dbl dist_tot_list;
    int lane_new;

    // For each lane, get distance x, y list and use lane with max dist covered.
    for (int lane=0; lane<3; lane++){
      vec_dbl dist_inc_list = calc_dist_inc(lane);
      
      double dist_tot = 0.;
      for (int k=0; k<dist_inc_list.size(); k++){
        dist_tot += dist_inc_list[k];
      }
      // SET DISTANCE TOTAL TO 0 IF ANOTHER CAR IS IN THE LANE IN RANGE
      if (cars_in_lane(lane)){
        dist_tot = 0.;
      }

      dist_tot_list.push_back(dist_tot);
      dist_inc_lists.push_back(dist_inc_list);
      // std::cout<<"Lane: "<<lane<<std::endl;
      // std::cout<<"Distance total: "<<dist_tot<<std::endl;
    }

    // Decide what lane to use. Default to current lane
    vec_dbl dist_inc_list;
    lane_new = end_lane;
    // ONLY CHECK FOR LANE CHANGE IF CAR LANE IS THE SAME AS POS LANE (CAR POS AT END OF PREVIOUS PATH)
    // IF END OF PREVIOUS PATH IN NEW LANE. USE THE NEW LANE. WE DO  NOT WANT TO CHANGE MULTIPLE LANES WITHIN THE WINDOW SELECTED.
    if (curr_lane == end_lane){
      if (lane_new == 0){
        if (dist_tot_list[1] >= 1.1 * dist_tot_list[0]){
          lane_new = 1;
        }
      }else if (lane_new == 1){
        if ((dist_tot_list[1] < 0.9 * dist_tot_list[0]) | (dist_tot_list[1] < 0.9 * dist_tot_list[2])){
          if (dist_tot_list[2] >= dist_tot_list[0]){
            lane_new = 2;
          }
          else{
            lane_new = 0;
          }
        }
      }else if (lane_new == 2){
        if (dist_tot_list[1] >= 1.1 * dist_tot_list[2]){
          lane_new = 1;
        }
      }
    }

    dist_inc_list = dist_inc_lists[lane_new];
    vec_dbl xpts, ypts;
    xpts.push_back(end_x_prev);
    xpts.push_back(end_x);
    ypts.push_back(end_y_prev);
    ypts.push_back(end_y);

    vec_dbl s_list {30., 60., 90.};

    // vec_dbl xy_5 = getXY(s_d[0]+5, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    for (double s : s_list){
      vec_dbl xy_d = getXY(end_path_s+s, calc_lane_d(lane_new), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      xpts.push_back(xy_d[0]);
      ypts.push_back(xy_d[1]);
    }

    int npts = xpts.size();
    // Convert to local coordinate system. This prevents multiple y values for the same x?(for short distances)
    for(int i=0; i<npts; i++){
      double shift_x = xpts[i] - end_x;
      double shift_y = ypts[i] - end_y;
      xpts[i] = (shift_x*cos(0. - end_angle) - shift_y*sin(0. - end_angle));
      ypts[i] = (shift_x*sin(0. - end_angle) + shift_y*cos(0. - end_angle));
    }

    tk::spline s;
    s.set_points(xpts, ypts);
    double dist = distance(xpts[1], ypts[1], xpts[2], ypts[2]); // this should be equal to first distance in d_list

    // std::cout<<"car speed="<<end_speed_prev_mps<<std::endl;
    int i;
    for (i = 0; i < (NUM_POINTS - path_size); ++i) {
      double dx_i = (i+1)*(dist_inc_list[i]/dist) * (xpts[2] - xpts[1]);
      double x_i = xpts[1]+dx_i;
      double y_i = s(x_i);

      double x_i_glob = end_x + x_i*cos(end_angle) - y_i*sin(end_angle);
      double y_i_glob = end_y + x_i*sin(end_angle) + y_i*cos(end_angle);
      next_path_x.push_back(x_i_glob);
      next_path_y.push_back(y_i_glob);
      // std::cout<<"next x "<<x_i_glob<<std::endl;
      // std::cout<<"next y "<<y_i_glob<<std::endl;
    }
}

vec_dbl Vehicle::calc_dist_inc(int lane){
  vector <double> dist_inc_list;
  double ego_i_s = end_path_s;
  double end_speed_prev_mps_tmp = end_speed_prev_mps;
  for (int i = 0; i < (3 * NUM_POINTS); ++i) {

    double dist_inc = DIST_INC_MAX;
    // ADJUST DIST INC FOR SMOOTH ACCEL
    double acc_proj = (SPEED_LIMIT/MPS_TO_MPH - end_speed_prev_mps)/dt;
    if (acc_proj > ACC_LIMIT){
      // s = ut + 1/2at^2
      dist_inc = end_speed_prev_mps_tmp * dt + 0.5 * ACC_LIMIT * pow(dt, 2);
      // std::cout<<"car speed adjusted for acc"<<end_speed_prev_mps<<std::endl;
    }

    // ADJUST DIST INC TO AVOID COLLISIONS IN SAME LANE
    double max_dist_in_lane = dist_inc;
    
    for (int j=0; j<sensor_data.size(); j++){
      double car_j_s = sensor_data[j].get_s();
      double car_j_d = sensor_data[j].get_d();
      double car_j_v = sensor_data[j].get_v();

      if (infer_lane(car_j_d) == lane){
        if (((ego_i_s + dist_inc) > (car_j_s + car_j_v * dt * path_size - SLOWDOWN_DIST)) & ((ego_i_s + dist_inc) < (car_j_s + car_j_v * dt * path_size))){
          if ((dist_inc/dt) > car_j_v){
            dist_inc = std::max(end_speed_prev_mps_tmp * dt - 0.5 * ACC_LIMIT * pow(dt, 2), car_j_v * dt);
          }
        }
      }
    }

    end_speed_prev_mps_tmp = dist_inc / dt;
    ego_i_s += dist_inc;
    dist_inc_list.push_back(dist_inc);
    // std::cout<<"dist inc "<<dist_inc<<std::endl;
  }
  return dist_inc_list;
}

// Check if EGO will be within 10m of other vehicles in the same lane after path_size steps 
bool Vehicle::cars_in_lane(int lane){
  for (int j=0; j<sensor_data.size(); j++){
    double car_j_s = sensor_data[j].get_s();
    double car_j_d = sensor_data[j].get_d();
    double car_j_v = sensor_data[j].get_v();
    
    if (infer_lane(car_j_d) == lane){
      if (((end_path_s) < (car_j_s + car_j_v * dt * path_size + MIN_LANECHANGE_DIST_REAR)) &
            ((end_path_s) > (car_j_s + car_j_v * dt * path_size - MIN_LANECHANGE_DIST_FRONT))){
        return true;
      }
    }
  }
  return false;
}

vec_dbl Vehicle::get_next_path_x(){
  return next_path_x;
}

vec_dbl Vehicle::get_next_path_y(){
  return next_path_y;
}
