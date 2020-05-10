#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
static string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
static constexpr double pi() { return M_PI; }
static double deg2rad(double x) { return x * pi() / 180; }
static double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
static double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
static int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
static vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
static vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

static vector<double> JMT(vector<double> start, vector<double> end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in steps, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd m(3, 3);
  VectorXd coeff(3);
  VectorXd y(3);
  m(0, 0) = pow(T, 3);
  m(1, 0) = 3. * pow(T, 2);
  m(2, 0) = 6. * T;
  m(0, 1) = pow(T, 4);
  m(1, 1) = 4. * pow(T, 3);
  m(2, 1) = 12. * pow(T, 2);
  m(0, 2) = pow(T, 5);
  m(1, 2) = 5. * pow(T, 4);
  m(2, 2) = 20. * pow(T, 3);
  // std::cout<<"JMT M: "<<m(0,0)<<", "<<m(1,0)<<", "<<m(2,0)<<std::endl;
  y(0) = end[0] - (start[0] + start[1]*T + 0.5*start[2]*pow(T, 2));
  y(1) = end[1] - (start[1] + start[2]*T);
  y(2) = end[2] - start[2];
  // std::cout<<"JMT end0: "<<end[0]<<std::endl;
  // std::cout<<"JMT y2: "<<y(2)<<std::endl;
  coeff = m.inverse() * y;
  return {start[0], start[1], 0.5*start[2], coeff[0], coeff[1], coeff[2]};
}

static double evaluate_JMT_poly(vector<double> coeff, double t){
  double val = 0;
  for(int i=0; i<6; ++i){
    val += coeff[i] * pow(t, i);
  }
  return val;
}

static vector<double> global_to_local(double ref_x, double ref_y, double ref_angle, double x_val, double y_val){
  vector<double> local_xy;
  double shift_x = x_val - ref_x;
  double shift_y = y_val - ref_y;
  local_xy.push_back(shift_x * cos(0. - ref_angle) - shift_y * sin(0. - ref_angle));
  local_xy.push_back(shift_x * sin(0. - ref_angle) + shift_y * cos(0. - ref_angle));
  return local_xy;
}

static vector<double> local_to_global(double ref_x, double ref_y, double ref_angle, double x_val, double y_val){
  vector<double> global_xy;
  global_xy.push_back(ref_x + x_val * cos(ref_angle) - y_val * sin(ref_angle));
  global_xy.push_back(ref_y + x_val * sin(ref_angle) + y_val * cos(ref_angle));
  return global_xy;
}

#endif  // HELPERS_H