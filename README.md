# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

-----------------------------------------------------------------------------------------------------------------------------------

## Work performed
### functional step used

1. Vehicle lane following
    - Generation of guide points in Frenet coordinate to define a spline.
        - for the Frenet 's' coordinates:
			- The first two points are:
		        -  either the 2 last points of the previous trajectory 
		        -  or in  the starting case the  starting point and a virtual point used to capture the vehicle direction
	        - The next three pooints are equally spaced points at an arbitrary distance of 1/3 of the spline  lenght (~90m/3). 
        - for the Frenet 'd' coordinates,the vehicle is maintained in the same line by giving a constant d position for the 5 points selected for the spline generation. 
	        - Each lane being 4 meter wide and the 'd' reference starting on the outside live edge,  the 'd' value corespond to `2+4*lane`
    - A spline is generated through the five points using an imported library `spline.h`. 
	    - Note: the spline is generated in local vehicle coordinates then converted into global coordinates.
    - Only the first third of the spline (~30m) is passed to the simulator as a target trajectory `msgJson["next_x"]` and `msgJson["next_y"]`.
   
2. Vehicle speed control.
	- The speed is obtained by controlling the distance between target points.
	- The distance correspond to 0.02s of time, therefore for a given reference speed, the distance needs to be : `.02*ref_vel/2.24`. (Note: the 2.24 value is due to the mph unit conversion).

4. Vehicle jerk and acceleration.
	- to avoid exceeding the limits, the  velocity is increased slowly in a linear way up to the target velocity
	- in the form: 
		- v(t)= v(t-dt) + a x dt
		- with a * 0.02 = 0.3
    
5. vehicle control
	- the state of all other vehicle is defined
	- Each vehicle position is from their current position and speed up to the reference vehicle end of trajectory `other_car_s += ((double)prev_size*0.02*other_car_v)`
	- Each vehicle is then classified with respect to the reference vehicle as follow:
		- `bool same_lane` , (vehicle in same lane to reference vehicle)
		- `bool car_to_the_right`, (vehicle to the right of the reference vehicle)
		- `bool car_to_the_left`, (vehicle to the left of the refernce vehicle)
		- `bool too_close_ahead`, (vehicle ahead of reference vehicle and distance to reference vehicle lower than tolerance)
		- `bool too_close_behind`, (vehicle behind reference vehicle and s distance to reference vehicle lower than tolerance)
	- the combination of those characteristics defines the state of the reference vehicle:
		- `bool car_ahead_in_lane_too_close`
		- `bool can_move_right`
		- `bool can_move_left`
	- Once the state is defined the orders are given to the simulator:
		- if the lane ahead is blocked then change lane if possible if not possible then decelerate
		- if lane ahead is free then accelerate if speed lower than maximum permited.
	- Note: the lane change is implemented by incrementing the vehicle lane by -1 or +1
    - Note : the move is restricted to the possible lane in the initialisation of  the `can_move_right`  and `can_move_left` booleans.

## Results obtained
About 35 miles without incidents.
## Possible improvement
- State machine
	- Use an objective function to decide which lane to use in stead of the simplistic line change only when the vehicle is getting stuck by a vehicle in front.
- Trajectory within a lane
	- The vehicle some times tends to wobble in its lane, using more control points to generate the spline may solve this issue. 
	- The vehicle speed could be adjusted to the front vehicle speed with a PID in stead of playing the "yoyo" behind a car (currently only a P in speed)
- Detection or forward tolerance
	- this value could be adjusted with the vehicle speed, the faster the speed, the larger the tolerance.
## Conclusion
This very simple state machine algorithm gives surprisingly good results, even if it is clearly not optimal.

 

-----------------------------------------------------------------------------------------------------------------------------------


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

