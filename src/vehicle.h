#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

struct SensorFusion{
    int id;
    double x, y, vx, vy, s, d;
};

class Vehicle {
  private:
    double curr_x, curr_y, curr_s, curr_d, curr_yaw, curr_speed;
    double end_x, end_y, end_path_s, end_path_d, end_angle;
    double end_x_prev, end_y_prev, end_speed_prev_mps;
    int curr_lane, end_lane;
    int path_size;
    vector<double> previous_path_x, previous_path_y;
    vector<double> next_path_x, next_path_y;
    vector<SensorFusion> sensor_data;

  public:
    // Constructors
    Vehicle();
    // Vehicle(Waypoints);
    
    // Destructor
    virtual ~Vehicle();

    void update_position(double x_, double y_, double s_, double d_, double yaw_, double speed_, 
                vector<double> prev_path_x, vector<double> prev_path_y, double end_s, 
                double end_d, vector<vector<double>> sensor_fusion);
    
    void plan_new_path(vector<double>map_waypoints_x, vector<double>map_waypoints_y, vector<double>map_waypoints_s);
    vector<double> calc_dist_inc(int lane);
    bool cars_in_lane(int lane);
    vector<double> get_next_path_x();
    vector<double> get_next_path_y();
};
#endif  // VEHICLE_H_