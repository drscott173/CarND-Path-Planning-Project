# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Project Submission

## Compilation
First, download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).  Start it up.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Valid Trajectories

You can see evidence of driving a full 4.32 miles on the simulator by [viewing
the following](https://www.youtube.com/watch?v=GL1bQZy1vgM) on YouTube:

![YouTube Video](https://img.youtube.com/vi/GL1bQZy1vgM/hqdefault.jpg)

We maintain the speed limit by choosing a target velocity of 49.5 MPH to stay
within the 50 MPH limit.  We set the maximum acceleration to 4 m/s^2. We minimize
jerk by adding a few points to the previously planned path each time, then smoothing
the result with a spline.  We plan lane changes out for 3 seconds (150 iterations)
by interpolating waypoints from the current lane to the future lane and fitting
a spline.

We avoid collisions bby looking at cars on the 
left, right and in front of us whose path would potentially cross ours in the
next NUM_POINTS/50 seconds.  NUM_POINTS is the number of points on our
trajectory that we send to the simulator.  This conservative approach works
for this pedagogical example.

We stay in our lane until the
coast is clear on the left or right.  We also reduce our speed to match the
car in front of us, slowing down as need be if we're closer than 2 seconds
apart at our current speed.  We
stay in our lane by planning a path to the center of a lane, starting at
2 meters out, then shifting over 4 meters for every lane.

The video shows that the car successfully changes lanes, prefers to stay in
the middle lane for maneuverability, slows down for the cars in front,
and accelerates to the desired max speed when possible.

## Trajectories

We follow the "hints" video for calculating trajectories.  At the beginning,
we choose our current lane and find waypoints forward three seconds into
the future in Frenet coordiantes.  We then fit a spline.  We convert the
Frenet coordinates to X,Y as our initial trajectory.

After each iteration, the simulator returns the unfinished portion of our
previous trajectory.  We use heuristics to first calculate a desired speed,
then a desired lane.  Starting at the end of the previous plan, we estimate how
the path must change to adjust the velocity and lane.  For lane changes, we 
interpolate waypoints to form a direct path from the end of our current trajectory
to the new lane.  We then add a few more points until our old trajectory,
up to NUM_POINTS.

This approach gradually tweaks the trajectory given heuristic updates.

## Reflection

I cranked the max speed to a Porsche 911 at 150mph.  This quickly shows the flaws
in these heuristics that pass the simple project criteria.  The car gets most
confused when it cannot slow down sufficiently from 150mph to avoid hitting a slow
car that entered its lane.  I suspect this could happen in the slower speed
version, should a car suddenly cut us off.  The trajectories get even more
complex in heavier traffic.

A more sophisticated approach would create a local grid around the car's
current position.  The grid would start at the current car position and be as
long in s as one could travel at max speed, as wide in d as the full road.  We'd
divide the grid into cells, choosing the finest granularity we could navigate
while still being computationally tractable.

Next, I'd use hybrid A* search to create a piece-wise path that succesfully
follows each of the possible states, starting k steps into the previous
path returned by the simulator.  k corresponds to actuator delay, seen here
to be about 0.04 seconds or 2 steps.

These "states" of our care are currently baked into 
the heuristic as conditional statements:

1. Stay in lane at max speed
2. Follow car ahead with a two second rule
3. Pass on left
4. Pass on right

We'd use a cost function that accounts for the heuristics seen in our
submission.  A state trajectory with middle lanes has
less cost than the left or right.  Collisions have
maximal cost.  Straying from the middle lane incurs more cost.  Faster
is better than slow.  And so on.

This path with our least cost would be my initial trajectory.  I'd smooth
this path with a spline. Finally, I'd attach this new, revised trajectory
to the head of the previous plan for a nice, smooth transition.

This would be fun.  Its more than needed to pass the test.  I believe
in writing as little code as possible, then iterating.  :-)

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
