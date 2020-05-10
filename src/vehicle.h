#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <map>
#include <string>
#include <vector>
#include "helpers.h"
#include "json.hpp"

using std::string;
using std::vector;
using nlohmann::json;

const double MPS_TO_MPH = 2.23;
const double  SPEED_LIMIT = 49.;
const double ACC_LIMIT = 9.0; // 10% buffer to account for added distance during curved roads.
const int NUM_POINTS = 40;
const double dt = 0.02;
const double DIST_INC_MAX = SPEED_LIMIT * dt / MPS_TO_MPH;
const int NUM_LANES = 3;
const double LANE_WIDTH = 4.;
const double COLLISION_S = 8.;
const double COLLISION_D = 0.75 * LANE_WIDTH;
const double PROXIMITY_S = 12.;
const double PROXIMITY_D = 4.;
const double SLOWDOWN_DIST = 20.;
const double MIN_LANECHANGE_DIST_FRONT = 15.;
const double MIN_LANECHANGE_DIST_REAR = 12.;
const double DIST_INC_MIN = 0.5 * DIST_INC_MAX; //ONLY FOR LANE CHANGES
const int NUM_PATHS_PER_LANE = 5;
const int PATH_STEPS = (int) 2.5/dt;
const double MAX_ACCEL_WT = 10.;
const double MAX_VEL_WT = 10.;
const double LANE_CHANGE_WT = 5.;
const double DIST_TRAVEL_WT = 2.;
const double COLLISION_WT = 10.;
const double PROXIMITY_WT = 3.;

typedef vector<double> vec_dbl;
typedef vector<vec_dbl> vec2d_dbl;
typedef vector<int> vec_int;


static int infer_lane(double d_val){
  for (int lane=0; lane<NUM_LANES; lane++){
      if ((d_val > lane * LANE_WIDTH) & (d_val <= (lane + 1) * LANE_WIDTH)){
        return lane;
      }
    }
  return -1;
}

static vec_int get_possible_lanes(int lane){
  if (lane == 0){
    return {0, 1};
  }else if (lane==1){
    return {0, 1, 2};
  }
  return {1, 2};
}

static double calc_lane_d(int lane){
  return LANE_WIDTH * (lane + 0.5);
}

static double calc_magntude(double x, double y){
  return distance(x, y, 0., 0.);
}

struct SensorFusion{
    vec_dbl data;

    inline double get_id(){
      return data[0];
    }
    inline double get_x(){
      return data[1];
    }
    inline double get_y(){
      return data[2];
    }
    inline double get_vx(){
      return data[3];
    }
    inline double get_vy(){
      return data[4];
    }
    inline double get_s(){
      return data[5];
    }
    inline double get_d(){
      return data[6];
    }

    inline double get_v(){
      return calc_magntude(get_vx(), get_vy());
    }
    
    // constructor
    SensorFusion(vec_dbl data_);
};

class Vehicle {
  private:
    double curr_x, curr_y, curr_s, curr_d, curr_yaw, curr_speed;
    double end_x, end_y, end_path_s, end_path_d, end_angle, end_angle2;
    double end_x_prev, end_y_prev, end_speed_prev_mps, end_x_prev2, end_y_prev2;
    double end_path_s_prev, end_path_s_prev2, end_path_d_prev, end_path_d_prev2;
    double end_path_s_vel, end_path_s_vel_prev, end_path_d_vel, end_path_d_vel_prev;
    double end_path_s_acc, end_path_d_acc;
    int curr_lane, end_lane;
    int path_size;
    vec_dbl previous_path_x, previous_path_y;
    vec_dbl next_path_x, next_path_y;
    vector<SensorFusion> sensor_data;
    vec_dbl map_waypoints_x, map_waypoints_y, map_waypoints_s;

  public:
    // Constructors
    Vehicle();
    Vehicle(vec_dbl map_waypoints_x, vec_dbl map_waypoints_y, vec_dbl map_waypoints_s);
    // Vehicle(Waypoints);
    
    // Destructor
    virtual ~Vehicle();
    void calc_end_positions();
    void compute_end_path_frenet();
    void update_position(const json& data);
    void plan_new_path(int &lane_new);
    void get_JMT_paths(vec_int possible_lanes, vec2d_dbl &jmt_paths_s, vec2d_dbl &jmt_paths_d);
    double compute_path_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    vec_dbl calc_dist_inc(int lane, bool is_lane_change);
    bool cars_in_lane(int lane);
    vec_dbl get_next_path_x();
    vec_dbl get_next_path_y();
    double max_accel_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    double max_vel_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    double lane_change_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    double dist_travel_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    double collision_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
    double proximity_cost(vec_dbl jmt_path_s, vec_dbl jmt_path_d);
};
#endif  // VEHICLE_H_