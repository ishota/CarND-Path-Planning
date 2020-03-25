# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[fastest]: ./picture/fastest.png "fastest"
[fantastic]: ./picture/fantastic.png "fantastic"

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

## Model Documentation

I controlled the self-driving car with prediction, behavior planning and trajectory calculation procedures.

### Prediction

Using the result of sensor fution, we observed the position of other car as seen from the ego car.
Using this result, it is predicted whether the car is driving on the same lane, the right lane, or the left lane.

Here, the speed and distance of the closest ahead car is also calculated.
I used them for trajectory calculation.

```cpp
if ( other_car_lane == lane ) {
  // Other car is in the same lane
  car_ahead |= check_car_s > car_s && check_car_s - car_s < JUDGEMENT_DISTANCE;
  if ( check_car_s > car_s && check_car_s - car_s < FOLLOWING_DISTANCE ) {
    // Save nearest other cars information
    if ( target_distance > check_car_s - car_s ) {
      // convert mps -> mph
      target_speed = check_speed * 2.23;
      target_distance = check_car_s - car_s;
    }
  }
} else if ( other_car_lane - lane == -1 ) {
  // Other car is on the left lane
  car_left |= car_s - JUDGEMENT_DISTANCE < check_car_s && car_s + JUDGEMENT_DISTANCE > check_car_s;
} else if ( other_car_lane - lane == 1 ) {
  // Other car is on the right lane
  car_right |= car_s - JUDGEMENT_DISTANCE < check_car_s && car_s + JUDGEMENT_DISTANCE > check_car_s;
}
```

### Behavior Planning

I used finite state machine.
The state consists of four.

1. Normal : Accelerates to maximum speed when there is no car around.
2. Follow : Control the speed acoording to the car ahead.
3. ChangeRigth : Change right lane.
4. ChangeLeft : Change left lane.

State transition conditions and actions after transition are defined as follows. [1]
The behavior related to horizontal control is used as reference method, but the action related to vertical control is deleted and made in Trajectory Calculation.

```cpp
fsm.add_transitions({
  //  from state        ,to state            ,triggers           ,guard                                                      ,action
  { States::Normal      ,States::ChangeLeft  ,Triggers::CarAhead ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}    ,[&]{lane--;} },
  { States::ChangeLeft  ,States::Normal      ,Triggers::Clear    ,[&]{return !car_ahead;}                                    ,[&]{} },
  { States::ChangeLeft  ,States::Follow      ,Triggers::CarAhead ,[&]{return car_ahead;}                                     ,[&]{} },
  { States::Follow      ,States::ChangeLeft  ,Triggers::CarAhead ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}    ,[&]{lane--;} },

  { States::Normal      ,States::ChangeRight ,Triggers::CarAhead ,[&]{return car_ahead && !car_right && lane != RIGHT_LANE;} ,[&]{lane++;} },
  { States::ChangeRight ,States::Normal      ,Triggers::Clear    ,[&]{return !car_ahead;}                                    ,[&]{} },
  { States::ChangeRight ,States::Follow      ,Triggers::CarAhead ,[&]{return car_ahead;}                                     ,[&]{} },
  { States::Follow      ,States::ChangeRight ,Triggers::CarAhead ,[&]{return car_ahead && !car_right && lane != RIGHT_LANE;} ,[&]{lane++;} },

  { States::Normal      ,States::Follow      ,Triggers::CarAhead ,[&]{return true;}                                          ,[&]{} },
  { States::Follow      ,States::Follow      ,Triggers::CarAhead ,[&]{return true;}                                          ,[&]{} },
  { States::Follow      ,States::Normal      ,Triggers::Clear    ,[&]{return !car_ahead;}                                    ,[&]{} },
  { States::Normal      ,States::Normal      ,Triggers::Clear    ,[&]{return !car_ahead;}                                    ,[&]{} },

}); // end fsm.add_transitions
```

I also changed the transition conditions.
Until car_d of the own car is located in the midle of the lane, it will not transit to the Change state to change one lane at a time.

Once car transitioned to Follw state, car will not transit to Normal state while ahead car in FOLLOWING_DISTANCE.

```cpp
if ( (2.5 < car_d && car_d < 5.5) || (6.5 < car_d && car_d < 9.5) ) {
  // Wait for completing Lane Change
  car_left = true;
  car_right = true;
}

if ( fsm.state() == Follow && target_distance < FOLLOWING_DISTANCE) {
  // Continue following
  car_ahead = true;
}
```

### Trajectory Calculation

Calculate the spline curve from the car position and map reference points.
Then, the speed at each point is calculated to satisfy all constraints.

I found the speed for each waypoints in the following code.
In case of normal state, accelerate to reach maximum speed.
In case of follow state, adjust speed to ahead cars.

```cpp
//Accelerate to reache MAX_VEL in Normal state
if ( fsm.state() == Normal ){
  if ( ref_vel + MAX_ACC < MAX_VEL ){
    ref_vel += MAX_ACC;
  }
}

//Follow 
if ( fsm.state() == Follow ){
  // 15m ( for avoiding collision )
  if ( target_distance < 15 ) {
    ref_vel -= MAX_ACC;
    if ( ref_vel < 0) {
      ref_vel = 0;
    }
  // adjust speed to following car's
  } else if ( target_speed > ref_vel && ref_vel + MAX_ACC < MAX_VEL) {
    ref_vel += MAX_ACC;
  } else if ( target_speed < ref_vel && ref_vel - MAX_ACC > MAX_ACC) {
    ref_vel -= MAX_ACC;
  }
}
```

## result

### result of fastest mode
![alt text][fastest]

### result of fantastic mode
![alt text][fantastic]

## reference

I refered finite state machine implements, spline calculation and lateral control.
I fixed and added vertical control. 
[1] https://github.com/mkoehnke/CarND-Path-Planning-Project