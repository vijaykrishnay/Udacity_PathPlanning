#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <map>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Waypoints{
  public:
    vector<double> x, y, s, dx, dy;
    int read_map(string map_file);
};

// Load up map values for waypoint's x,y,s and d normalized normal vectors
int Waypoints::read_map(string map_file){
  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double _x;
    double _y;
    float _s;
    float _d_x;
    float _d_y;
    iss >> _x;
    iss >> _y;
    iss >> _s;
    iss >> _d_x;
    iss >> _d_y;
    x.push_back(_x);
    y.push_back(_y);
    s.push_back(_s);
    dx.push_back(_d_x);
    dy.push_back(_d_y);
  }
};
#endif