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

void Vehicle::compute_end_path_frenet(){
  vec_dbl sd_prev = getFrenet(end_x_prev, end_y_prev, end_angle, map_waypoints_x, map_waypoints_y);
  vec_dbl sd_prev2 = getFrenet(end_x_prev2, end_y_prev2, end_angle2, map_waypoints_x, map_waypoints_y);
  end_path_s_prev = sd_prev[0];
  end_path_d_prev = sd_prev[1];
  end_path_s_prev2 = sd_prev2[0];
  end_path_d_prev2 = sd_prev2[1];
  
  // Compute velocities, accels in frenet
  end_path_d_vel = (end_path_d - end_path_d_prev)/dt;
  end_path_d_vel_prev = (end_path_d_prev - end_path_d_prev2)/dt;
  end_path_d_acc = (end_path_d_vel - end_path_d_vel_prev)/dt;
  std::cout<<"end_path_s_prev: "<<end_path_s_prev<<", "<<end_path_s_prev2<<std::endl;

  end_path_s_vel = (end_path_s - end_path_s_prev)/dt;
  end_path_s_vel_prev = (end_path_s_prev - end_path_s_prev2)/dt;
  end_path_s_acc = (end_path_s_vel - end_path_s_vel_prev)/dt;
  std::cout<<"end_path_s_prev: "<<end_path_s_prev<<", "<<end_path_s_prev2<<std::endl;
  std::cout<<"end_path_s_vel: "<<end_path_s_vel<<std::endl;
}

void Vehicle::calc_end_positions(){
  if (path_size < 3) {
    end_x = curr_x;
    end_y = curr_y;
    end_angle = deg2rad(curr_yaw);
    end_angle2 = deg2rad(curr_yaw);
    end_path_s = curr_s;
    end_path_d = curr_d;
    
    end_x_prev = end_x - cos(end_angle) * dt;
    end_y_prev = end_y - sin(end_angle) * dt;
    end_x_prev2 = end_x_prev - cos(end_angle2) * dt;
    end_y_prev2 = end_y_prev - sin(end_angle2) * dt;
    end_speed_prev_mps = curr_speed / MPS_TO_MPH;
    // std::cout<<"path size=0"<<std::endl;
  } else {
    end_x = previous_path_x[path_size-1];
    end_y = previous_path_y[path_size-1];

    end_x_prev = previous_path_x[path_size-2];
    end_y_prev = previous_path_y[path_size-2];
    end_x_prev2 = previous_path_x[path_size-3];
    end_y_prev2 = previous_path_y[path_size-3];
    end_angle = atan2(end_y - end_y_prev, end_x - end_x_prev);
    end_angle2 = atan2(end_y_prev - end_y_prev2, end_x_prev - end_x_prev2);
    end_speed_prev_mps = distance(end_x, end_y, end_x_prev, end_y_prev) / dt;
    // std::cout<<"path size: "<<path_size<<std::endl;
  }
  // COMPUTE END PATH FRENET COORDS (REQUIRED FOR JMT())
  compute_end_path_frenet();
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

void Vehicle::get_JMT_paths(vec_int possible_lanes, vec2d_dbl &jmt_paths_s, vec2d_dbl &jmt_paths_d){
  vec_dbl start_jmt_d, end_jmt_d;
  vec_dbl start_jmt_s, end_jmt_s;
  start_jmt_d = {end_path_d, end_path_d_vel, end_path_d_acc};
  start_jmt_s = {end_path_s, end_path_s_vel, end_path_s_acc};
  std::cout<<"end_path_s_prev: "<<end_path_s_prev<<", "<<end_path_s_prev2<<std::endl;
  std::cout<<"end_path_s_vel: "<<end_path_s_vel<<std::endl;
  for(int lane: possible_lanes){
    for(int i=0; i<NUM_PATHS_PER_LANE; ++i){
      // std::cout<<"lane, path: "<<lane<<i<<std::endl;
      end_jmt_d = {calc_lane_d(lane), 0. ,0.}; // 0 lateral vel, acc at start, end
      jmt_paths_d.push_back(JMT(start_jmt_d, end_jmt_d, PATH_STEPS * dt));
      // vec_dbl tmp = JMT(start_jmt_d, end_jmt_d, PATH_STEPS * dt);
      // std::cout<<"JMT D(0)"<<evaluate_JMT_poly(tmp, 1)<<std::endl;

      double end_jmt_s_i, dist_step_i;
      dist_step_i = DIST_INC_MIN + ((double)i/(double)NUM_PATHS_PER_LANE) * (DIST_INC_MAX - DIST_INC_MIN);
      // if (lane == end_lane){
      //   dist_step_i = ((double)i/(double)NUM_PATHS_PER_LANE) * DIST_INC_MAX;
      // }else{
      //   dist_step_i = DIST_INC_MIN + ((double)i/(double)NUM_PATHS_PER_LANE) * (DIST_INC_MAX - DIST_INC_MIN);
      // }
      end_jmt_s_i = end_path_s + dist_step_i * PATH_STEPS;
      end_jmt_s = {end_jmt_s_i, dist_step_i/dt, 0.}; // 0 acc preferred at the end
      jmt_paths_s.push_back(JMT(start_jmt_s, end_jmt_s, PATH_STEPS * dt));
      // vec_dbl tmp2 = JMT(start_jmt_s, end_jmt_s, PATH_STEPS * dt);
      std::cout<<"S start: "<<start_jmt_s[0]<<std::endl;
      std::cout<<"dS start: "<<start_jmt_s[1]<<std::endl;
      std::cout<<"ddS start: "<<start_jmt_s[2]<<std::endl;
      std::cout<<"S end: "<<end_jmt_s[0]<<std::endl;
      std::cout<<"dS end: "<<end_jmt_s[1]<<std::endl;
      std::cout<<"ddS end: "<<end_jmt_s[2]<<std::endl;
      // for (int kk=0; kk<PATH_STEPS; ++kk){
      //   std::cout<<"JMT S: "<<evaluate_JMT_poly(tmp2, kk * dt)<<std::endl;
      // }
    }
  }
}

double Vehicle::max_accel_cost(vec_dbl path_s, vec_dbl path_d){
  double cost = 0.;
  double vel_i, vel_i_1, acc_i, cost_i;
  for(int i = 2; i < path_s.size(); ++i){
    // std::cout<<"path s: "<<path_s[i]<<std::endl;
    vel_i_1 = (path_s[i-1] - path_s[i-2])/dt;
    vel_i = (path_s[i] - path_s[i-1])/dt;
    acc_i = (vel_i - vel_i_1)/dt;
    cost_i = std::pow(std::abs(acc_i - ACC_LIMIT), 3);
    if (cost_i > cost){
      cost = cost_i;
    }
  }
  // std::cout<<"cost acc"<<cost<<std::endl;
  return cost;
}

double Vehicle::max_vel_cost(vec_dbl path_s, vec_dbl path_d){
  double cost = 0.;
  double vel_i, vel_i_1, cost_i;
  for(int i = 1; i < path_s.size(); ++i){
    vel_i = (path_s[i] - path_s[i-1])/dt;
    cost_i = std::pow(std::abs(vel_i - SPEED_LIMIT/MPS_TO_MPH), 3);
    // std::cout<<"vel i "<<vel_i<<std::endl;
    if (cost_i > cost){
      cost = cost_i;
    }
  }
  // std::cout<<"cost vel"<<cost<<std::endl;
  return cost;
}

double Vehicle::lane_change_cost(vec_dbl path_s, vec_dbl path_d){
  double count = 0;
  // count number of steps with CAR MORE THAN 1/4 OF LANE WIDTH OFF CENTER
  for(int i = 0; i < path_s.size(); ++i){
    if (std::abs(std::fmod(path_d[i], LANE_WIDTH) - LANE_WIDTH/2.) > LANE_WIDTH/4.){
      count += 1.;
    }
  }
  // std::cout<<"cost lane"<<count/path_s.size()<<std::endl;
  return count/path_s.size();
}

double Vehicle::dist_travel_cost(vec_dbl path_s, vec_dbl path_d){
  double cost = 1.;
  for(int i = 1; i < path_s.size(); ++i){
    cost -= (path_s[i] - path_s[i-1])/DIST_INC_MAX;
  }
  // std::cout<<"cost dist"<<cost<<std::endl;
  return cost;
}

double Vehicle::collision_cost(vec_dbl path_s, vec_dbl path_d){
  for(int i = 0; i < path_s.size(); ++i){
    if (i > NUM_POINTS){
      break; // Check only for NUM_POINTS
    }
    for(int j=0; j<sensor_data.size(); j++){
      double car_j_s = sensor_data[j].get_s();
      double car_j_d = sensor_data[j].get_d();
      double car_j_v = sensor_data[j].get_v();
      double car_j_i_s = car_j_s + car_j_v * dt * i;
      if ((std::abs(car_j_d - path_d[i]) < COLLISION_D) & (std::abs(car_j_i_s - path_s[i]) < COLLISION_S)){
        return 1.0;
      }
    }
  }
  return 0.;
}

double Vehicle::proximity_cost(vec_dbl path_s, vec_dbl path_d){
  double cost, cost_tmp;
  cost = 0.;
  for(int i = 0; i < path_s.size(); ++i){
    if (i > NUM_POINTS){
      break; // Check only for NUM_POINTS
    }
    for(int j=0; j<sensor_data.size(); j++){
      double car_j_s = sensor_data[j].get_s();
      double car_j_d = sensor_data[j].get_d();
      double car_j_v = sensor_data[j].get_v();
      double car_j_i_s = car_j_s + car_j_v * dt * i;
      if (std::abs(car_j_i_s - path_s[i]) > PROXIMITY_S){
        continue;
      }
      if (std::abs(car_j_d - path_d[i]) > PROXIMITY_D){
        continue;
      }
      cost_tmp = 0.5 * std::exp(-(car_j_i_s - path_s[i])/PROXIMITY_S);
      cost_tmp += 0.5 * std::exp(-(car_j_d - path_d[i])/PROXIMITY_D);
      cost = std::max(cost, cost_tmp);
    }
  }
  // std::cout<<"cost prox"<<cost<<std::endl;
  return cost;
}

double Vehicle::compute_path_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d){
  vec_dbl path_s, path_d;
  for(int i_step = 0; i_step < PATH_STEPS; ++i_step){
    path_s.push_back(evaluate_JMT_poly(jmt_path_s, i_step * dt));
    path_d.push_back(evaluate_JMT_poly(jmt_path_d, i_step * dt));
  }

  return max_accel_cost(path_s, path_d) * MAX_ACCEL_WT + max_vel_cost(path_s, path_d) * MAX_VEL_WT + 
          lane_change_cost(path_s, path_d) * LANE_CHANGE_WT + dist_travel_cost(path_s, path_d) * DIST_TRAVEL_WT + 
          collision_cost(path_s, path_d) * COLLISION_WT + proximity_cost(path_s, path_d) * PROXIMITY_WT;
}

void Vehicle::plan_new_path(int &lane_new){
  next_path_x.clear();
  next_path_y.clear();

  // REUSE REMAINING PATH FROM PREVIOUS ITERATION
  for(int i = 0; i < path_size; ++i) {
    next_path_x.push_back(previous_path_x[i]);
    next_path_y.push_back(previous_path_y[i]);
  }
  // std::cout<<"path_size "<<path_size<<std::endl;  

  // GET LIST OF POSSIBLE LANES GIVEN CURRENT LANE
  vec_int possible_lanes = get_possible_lanes(end_lane);
  // std::cout<<"possible lanes done!"<<std::endl;

  // GET N JMT PATHS PER POSSIBLE LANE
  vec2d_dbl jmt_paths_s, jmt_paths_d;
  get_JMT_paths(possible_lanes, jmt_paths_s, jmt_paths_d);
  // std::cout<<"JMT paths done!"<<std::endl;

  // COMPUTE COST PER PATH AND IDENTIFY MIN COST PATH
  double cost_min;
  int min_path_idx;
  for(int i = 0; i < jmt_paths_d.size(); ++i){
    double cost_i = compute_path_cost(jmt_paths_s[i], jmt_paths_d[i]);
    // std::cout<<"cost i: "<<cost_i<<std::endl;
    if (i==0){
      cost_min = cost_i;
    }

    if (cost_i < cost_min){
      min_path_idx = i;
      cost_min = cost_i;
    }
  }
  // std::cout<<"Compute costs done!"<<std::endl;

  // FIT SPLINE TO BEST PATH AND EXTRACT XY FROM SPLNE TO ADD TO PATH
  vec_dbl xpts, ypts;
  xpts.push_back(end_x_prev);
  xpts.push_back(end_x);
  ypts.push_back(end_y_prev);
  ypts.push_back(end_y);

  for(int i_step = 1; i_step < PATH_STEPS; ++i_step){
    double i_step_s = evaluate_JMT_poly(jmt_paths_s[min_path_idx], i_step * dt);
    double i_step_d = evaluate_JMT_poly(jmt_paths_d[min_path_idx], i_step * dt);
    std::cout<<"Step i s: "<<i_step_s<<std::endl;
    // std::cout<<"Step i d: "<<i_step_d<<std::endl;

    vec_dbl xy_d = getXY(i_step_s, i_step_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    xpts.push_back(xy_d[0]);
    ypts.push_back(xy_d[1]);
  }

  int npts = xpts.size();
  // Convert to local coordinate system. This prevents multiple y values for the same x?(for short distances)
  for(int i=0; i<npts; i++){
    vec_dbl xy_i_local = global_to_local(end_x, end_y, end_angle, xpts[i], ypts[i]);
    xpts[i] = xy_i_local[0];
    ypts[i] = xy_i_local[1];
    // std::cout<<"i x: "<<xpts[i]<<std::endl;
    // std::cout<<"i y: "<<ypts[i]<<std::endl;
  }

  tk::spline s;
  s.set_points(xpts, ypts);

  // std::cout<<"car speed="<<end_speed_prev_mps<<std::endl;
  int i;
  for(i = 0; i < (NUM_POINTS - path_size); ++i) {
    vec_dbl xy_global = local_to_global(end_x, end_y, end_angle, xpts[i+2], ypts[i+2]);
    next_path_x.push_back(xy_global[0]);
    next_path_y.push_back(xy_global[1]);
    // std::cout<<"next x "<<xy_global[0]<<std::endl;
    // std::cout<<"next y "<<xy_global[1]<<std::endl;
  }
}

// Check if EGO will be within 10m of other vehicles in the same lane after path_size steps 
bool Vehicle::cars_in_lane(int lane){
  for(int j=0; j<sensor_data.size(); j++){
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
