# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.



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

## Model Documentation

Here we output a list of x and y global map co-ordinates where each pair of x and y  co-ordinates is a point and all the points together form a trajectory which the car in the simulator follows. We initially start with checking the size of the previous path traversed by the car and noting the size. If a previous path exists(ie. if the car is not stationary) then the end longitudnal displacement value (s) is considered as the cars current longitudnal displacement.The next step would be to iterate over the sensor fusion list to get the sensor data(x,y,vx, vy, s, d) of all cars on the highway, using the obtained values we check the lateral displacement of each car in the list and ascertain whether they are in the same lane as that of the car. If yes we check whether the car is in front of our car and if the distance between the two cars is less than 30m. If this condition holds good then it means that the car in front is travelling at a lower velocity hence we would need to lower our velocity gradually and change lanes if the conditions of the neighbouring lanes permit it. The velocity change here is controlled by setting a boolean variable 'too_close', and 'isCarInRangeForLane' function is used to iterate over the sensor fusion data of the cars in the lane to the right of the current lane first and if there are no cars within 15 m in the forward or backward direction(ie less probability of being obstructed by a car during lane transition) then the lane is changed to ensure that the car is able to maintain a speed close to the 50mph speed limit. If the right lane change is not possible the same process is repeated for the left lane to check whether a smooth transition to the left lane is possible. If both cases are not possible then the velocity of the car is gradually reduced at the rate of 0.324 miles per hour every 0.02 seconds(ie acceleration of 7.2 m/s^2) to avoid collison with the vehicle in front. Once a lane change is made or if there is no chance of collision from any upcoming vehicle then we also ensure that the velocity of the car is maintained close to the speed limit by gradually increasing the velocity of the car by 0.224 miles per hour every 0.02 seconds(ie acceleration of 5m/s^2)if it is lower than the reference velocity which is set to 49.5miles per hour. This gradual control of velocity also minimizes jerk that is experienced otherwise during the starting of the car or lane shift and other scenarios.


The trajectory generation for the car is done here by using the spline function since it ensures that the car passes through every point and hence gives a smoother trajectory. The input to the spline function are 2 lists of x and y points which are calculated as follows.
*  If the previous path has no points  or less than 2 points left then use current car x and y to calculate previous car x & y and add to ptsx and ptsy list.
* If previous path has enough points add the last 2 points to the ptsx and ptsy list.
* Predict future waypoints at distances of 30, 60 and 90m and add to points list. 

Once the ptsx and ptsy points are calculated and added they are converted to car co-ordinates from map co-ordinates using translation and rotation. Then these points are fit to a spline function to obtain the trajectory. We take the points remaining in the previous path (unused points from previous path) and add them to the next_x_vals and next_y_vals list ie the waypoints list. Considering an x value of 30m ahead of the car the corresponding y value is calculated using the spline function. This x and y point is used to calculate the target distance which is further used to calculate the number of divisions the trajectory is to be split into. For each of the remaining points which are left after adding the points from the previous path(50- points from previous path) we feed the x points to the spline to get the corresponding y point which is later converted back into map co-ordinates and added to the waypoints list. Using information from the previous path ensures that there is a smooth transition from cycle to cycle. Hence this results in a smooth trajectory transition in terms of lane change or change in speed and allows the car to traverse smoothly without unnecesary jerks and collisions as explained above.







