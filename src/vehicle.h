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
const double SLOWDOWN_DIST = 20.;
const double MIN_LANECHANGE_DIST_FRONT = 15.;
const double MIN_LANECHANGE_DIST_REAR = 12.;

static int infer_lane(double d_val){
  for (int lane=0; lane<NUM_LANES; lane++){
      if ((d_val > lane * LANE_WIDTH) & (d_val <= (lane + 1) * LANE_WIDTH)){
        return lane;
      }
    }
  return -1;
}

static double calc_lane_d(int lane){
  return LANE_WIDTH * (lane + 0.5);
}

static double calc_magntude(double x, double y){
  return distance(x, y, 0., 0.);
}

typedef vector<double> vec_dbl;
typedef vector<vec_dbl> vec2d_dbl;

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
    double end_x, end_y, end_path_s, end_path_d, end_angle;
    double end_x_prev, end_y_prev, end_speed_prev_mps;
    int curr_lane, end_lane;
    int path_size;
    vec_dbl previous_path_x, previous_path_y;
    vec_dbl next_path_x, next_path_y;
    vector<SensorFusion> sensor_data;

  public:
    // Constructors
    Vehicle();
    // Vehicle(Waypoints);
    
    // Destructor
    virtual ~Vehicle();
    // void update_position(double x_, double y_, double s_, double d_, double yaw_, double speed_, 
    //             vec_dbl prev_path_x, vec_dbl prev_path_y, double end_s, 
    //             double end_d, vec2d_dbl sensor_fusion);
    void update_position(const json& data);
    
    void plan_new_path(vec_dbl map_waypoints_x, vec_dbl map_waypoints_y, vec_dbl map_waypoints_s, int &lane_new);
    vec_dbl calc_dist_inc(int lane, bool is_lane_change);
    bool cars_in_lane(int lane);
    vec_dbl get_next_path_x();
    vec_dbl get_next_path_y();
};
#endif  // VEHICLE_H_