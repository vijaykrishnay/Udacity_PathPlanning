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

Vehicle::Vehicle(vec_dbl map_waypoints_x_, vec_dbl map_waypoints_y_, vec_dbl map_waypoints_s_){
  map_waypoints_x = map_waypoints_x_;
  map_waypoints_y = map_waypoints_y_;
  map_waypoints_s = map_waypoints_s_;
}

SensorFusion::SensorFusion(vec_dbl data_){
    data = data_;
};


/**
 * Destructor.
 */
Vehicle::~Vehicle() {}

void Vehicle::calc_end_positions(){
  if (path_size < 3) {
    end_x = curr_x;
    end_y = curr_y;
    end_angle = deg2rad(curr_yaw);
    end_path_s = curr_s;
    end_path_d = curr_d;
    
    end_x_prev = end_x - cos(end_angle) * dt;
    end_y_prev = end_y - sin(end_angle) * dt;
    end_speed_prev_mps = curr_speed / MPS_TO_MPH;
    // std::cout<<"path size=0"<<std::endl;
  } else {
    end_x = previous_path_x[path_size-1];
    end_y = previous_path_y[path_size-1];

    end_x_prev = previous_path_x[path_size-2];
    end_y_prev = previous_path_y[path_size-2];
    end_angle = atan2(end_y - end_y_prev, end_x - end_x_prev);
    end_speed_prev_mps = distance(end_x, end_y, end_x_prev, end_y_prev) / dt;
    // std::cout<<"path size: "<<path_size<<std::endl;
  }
}

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
    
    // CALCULATE END, END PREV POSITIONS
    calc_end_positions();

    // CALCULATE LANES
    curr_lane = infer_lane(curr_d);
    end_lane = infer_lane(end_path_d);
    // std::cout<<"curr lane = "<<curr_lane<<std::endl;
}

void Vehicle::plan_new_path(int &lane_new){
    next_path_x.clear();
    next_path_y.clear();

    for (int i = 0; i < path_size; ++i) {
      next_path_x.push_back(previous_path_x[i]);
      next_path_y.push_back(previous_path_y[i]);
    }

    vec2d_dbl dist_inc_lists;
    vec_dbl dist_tot_list;

    // For each lane, get distance x, y list and use lane with max dist covered.
    for (int lane=0; lane<3; lane++){
      vec_dbl dist_inc_list = calc_dist_inc(lane, lane!=end_lane);
      
      double dist_tot = 0.;
      for (int k=0; k<dist_inc_list.size(); k++){
        dist_tot += dist_inc_list[k];
      }
      // SET DISTANCE TOTAL TO 0 IF ANOTHER CAR IS IN THE LANE IN RANGE
      if (lane!=end_lane){
        if (cars_in_lane(lane)){
          dist_tot = 0.;
        }        
      }      

      dist_tot_list.push_back(dist_tot);
      dist_inc_lists.push_back(dist_inc_list);
      // std::cout<<"Lane: "<<lane<<std::endl;
      // std::cout<<"Distance total: "<<dist_tot<<std::endl;
    }

    // Decide what lane to use. Default to current lane
    vec_dbl dist_inc_list;

    if (lane_new < 0){
      lane_new = end_lane;
    }
    // ONLY CHECK FOR LANE CHANGE IF CAR LANE IS THE SAME AS NEW LANE (from previous iteration)
    // IF END OF PREVIOUS PATH IN NEW LANE. USE THE NEW LANE. WE DO  NOT WANT TO CHANGE MULTIPLE LANES WITHIN THE WINDOW SELECTED.
    // std::cout<<"lane_new: "<<lane_new<<std::endl;
    if (curr_lane == lane_new){
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

    // vec_dbl xy_5 = getXY(s_d[0]+5, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    double new_d = calc_lane_d(lane_new);
    double lane_change_dist = end_speed_prev_mps * LANE_CHANGE_T * std::abs(new_d - end_path_d) / LANE_WIDTH;
    for (double s=20.; s < 100.; s+= 15){
      if (s < lane_change_dist){
        // this helps smoother lane changes?
        new_d = ((lane_change_dist - s) * end_path_d + s * new_d)/lane_change_dist;
      }
      vec_dbl xy_d = getXY(end_path_s+s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      xpts.push_back(xy_d[0]);
      ypts.push_back(xy_d[1]);
    }

    int npts = xpts.size();
    // Convert to local coordinate system. This prevents multiple y values for the same x?(for short distances)
    for(int i=0; i<npts; i++){
      vec_dbl xy_i_local = global_to_local(end_x, end_y, end_angle, xpts[i], ypts[i]);
      xpts[i] = xy_i_local[0];
      ypts[i] = xy_i_local[1];
    }

    tk::spline s;
    s.set_points(xpts, ypts);
    double dist = distance(xpts[1], ypts[1], xpts[2], ypts[2]); // this should be equal to first distance in d_list

    // std::cout<<"car speed="<<end_speed_prev_mps<<std::endl;
    int i;
    for (i = 0; i < (NUM_POINTS - path_size); ++i) {
      double dx_i = (i+1)*(dist_inc_list[i]/dist) * (xpts[2] - xpts[1]);
      double x_i = xpts[1]+dx_i;
      vec_dbl xy_global = local_to_global(end_x, end_y, end_angle, x_i, s(x_i));
      next_path_x.push_back(xy_global[0]);
      next_path_y.push_back(xy_global[1]);
      // std::cout<<"next x "<<xy_global[0]<<std::endl;
      // std::cout<<"next y "<<xy_global[1]<<std::endl;
    }
}

vec_dbl Vehicle::calc_dist_inc(int lane, bool is_change_lane){
  vector <double> dist_inc_list;
  double ego_i_s = end_path_s;
  double end_speed_prev_mps_tmp = end_speed_prev_mps;
  for (int i = 0; i < (3 * NUM_POINTS); ++i) {
    double dist_inc = DIST_INC_MAX;
    if ((is_change_lane) & (i < NUM_POINTS)){
      dist_inc *= 0.995;
    }
    // ADJUST DIST INC FOR SMOOTH ACCEL
    double acc_proj = (dist_inc/dt - end_speed_prev_mps)/dt;
    if (acc_proj > ACC_LIMIT){
      // s = ut + 1/2at^2
      dist_inc = end_speed_prev_mps_tmp * dt + 0.5 * ACC_LIMIT * pow(dt, 2);
      // std::cout<<"car speed adjusted for acc"<<end_speed_prev_mps<<std::endl;
    }

    // check for braking
    if (acc_proj < -ACC_LIMIT){
      // s = ut + 1/2at^2
      std::cout<<"car speed adjusted for braking: "<<dist_inc<<std::endl;
      // dist_inc = end_speed_prev_mps_tmp * dt - 0.5 * ACC_LIMIT * pow(dt, 2);
      std::cout<<"car speed adjusted for braking: "<<dist_inc<<std::endl;
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
    double end_car_j_s = car_j_s + car_j_v * dt * path_size;
    if (infer_lane(car_j_d) == lane){
      if ((end_path_s < (end_car_j_s + MIN_LANECHANGE_DIST_REAR)) & (end_path_s > (end_car_j_s - MIN_LANECHANGE_DIST_FRONT))){
        return true;
      }

      if ((curr_s < (car_j_s + MIN_LANECHANGE_DIST_REAR)) & (curr_s > (car_j_s - MIN_LANECHANGE_DIST_FRONT))){
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
