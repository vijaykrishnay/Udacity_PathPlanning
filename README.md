# Path Planning for Highway Driving
This project implements path planning to safely navigate around a virtual highway with other traffic that is driving +/-15 MPH of the 50 MPH speed limit. A sparse map list of waypoints around the highway and the car's localization and sensor fusion data are provided. The car attempts to stay close to the 50 MPH speed limit. This involves passing slower traffic when possible. The car avoids hitting other cars and drives inside of the marked road lanes at all times, unless going from one lane to another. Also the car avoids total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Input Data
#### Main car's localization Data (No Noise)
- ["x"] The car's x position in map coordinates
- ["y"] The car's y position in map coordinates
- ["s"] The car's s position in frenet coordinates
- ["d"] The car's d position in frenet coordinates
- ["yaw"] The car's yaw angle in the map
- ["speed"] The car's speed in MPH

#### Previous path data given to the Planner
- ["previous_path_x"] The previous list of x points previously given to the simulator
- ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 
- ["end_path_s"] The previous list's last point's frenet s value
- ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
- ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

#### The map of the highway is in data/highway_map.txt
- Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
- The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


## Output Data
- ["next_x"] List of x values for car positions separated by 0.02s
- ["next_y"] List of y values for car positions separated by 0.02s

## Solution
1. Distance increments are calculated in Frenet coordinates for each lane such that accel limits, vel limits and in distance to vehicle in the same lane are adhered to.
  - Distances are calculated for 3x the number of desired points to ensure optimality in the long term.
2. Possible lanes are screened based on current lane.
3. For lane changes, collisions are detected and distance is set to 0 (to eliminate choice).
4. Prefered lane is identified by distance travelled per lane. Lane changes require 10% additional distance travel.
5. Spline is created for the lane selected based on local x, y coords of way points (upto 80m).
6. Spline is interpolated based on distance increments for the selected lane and XY coords are converted back to global.
  - Spline interpolation points were tuned to result in lateral acc below thd for lane changes.

# Particle Filter Code
The directory structure of this repository is as follows:

```
root
|   CMakeLists.txt
|   cmakepatch.txt
|   install-mac.sh
|   install-ubuntu.sh
|   README.md
|   run.sh
|   LICENSE
|   
|___data
|   |   highway_map.csv
|   
|___src
    |   helpers.h
    |   main.cpp
    |   json.hpp
    |   spline.h
    |   vehicle.cpp
    |   vehicle.h
    |
    |___Eigen-3.3
```
- main.cpp
    - Communicates with the Simulator (see dependencies below) receiving localization and sensor fusion data. Calls `update_position()`, `plan_new_path()` functions at each time step.
- vehicle.cpp
    - Implements functions of vehicle class:
        - `update_position()`: Updates vehicle attributes to current time step.
        - `plan_new_path()`: Calculates distance travelled in each possible lane without violatig the constraints and computes new spline smoothed path.
- helpers.h
    - Helper functions to convert car coordinates from XY to Frenet and vice versa. Also includes functions for local <-> global transformations. 

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  
