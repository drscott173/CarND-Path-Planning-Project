# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Project Submission

## Compilation
First, download the Term3 Simulator which contains
the Path Planning Project from the
[releases] tab (https://github.com/udacity/self-driving-car-sim/releases).
Start it up!

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Valid Trajectories

You can see evidence of driving a full 4.32 miles on the simulator by
[viewing](https://www.youtube.com/watch?v=GL1bQZy1vgM) the following on YouTube:

![YouTube Video](https://img.youtube.com/vi/GL1bQZy1vgM/hqdefault.jpg)

We maintain the speed limit by choosing a target velocity of 49.5mph to stay
within the 50mph limit.  We set the maximum acceleration to 0.4 m/s^2. We minimize
jerk by adding a few points to the previously planned path each time, then smoothing
the result with a spline.  We plan lane changes to occur in 3 seconds (150 iterations)
by interpolating waypoints from the current lane to the future lane and fitting
a spline.

We avoid collisions by looking at cars on the 
left, right and in front of us whose path would potentially cross ours in the
next NUM_POINTS/50 seconds.  NUM_POINTS is the number of points on our
trajectory that we send to the simulator.

We stay in our lane until the
coast is clear on the left or right.  We also reduce our speed to match the
car in front of us, using the "2 second rule" to adjust our distance.

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

I cranked the max speed to 150mph for testing.  This quickly shows the 
heuristic limitations. The car gets most
confused when it cannot slow down sufficiently to avoid collisions.
I suspect this could happen in the slower speed
version, should a car suddenly cut us off.  The trajectories get even more
complex in heavier traffic.

A better approach would create a local grid around the car's
current position.  The grid would start at the current car position and be as
long in s as one could travel at max speed, as wide in d as the full road.  We'd
divide the grid into cells, choosing the finest granularity we could physically
navigate in 0.02 seconds while still being computationally tractable.

Next, I'd use 
[hybrid A* search](http://blog.habrador.com/2015/11/explaining-hybrid-star-pathfinding.html) to create a piece-wise path that succesfully
follows each of the possible states, starting k steps into the previous
path returned by the simulator. k corresponds to actuator delay, with a minimum
of 2 to ensure smooth curve transition.  

We would have a cube of grids, one Frenet grid
for each incremental time step. Grid cells of open road are "free," whereas
grid cells of other cars would be "occupied."

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

One last complexity would model uncertainty for obstacles (cars, pedestrians,
etc) as we see in Kalman filters.  These distributions capture possible
states and paths forward. A cell would then be blocked a given
uncertainty, which we could perhaps
model by adjusting the collision cost (0-1).
The good news is we may only project a few hundred
milliseconds in the future, as uncertainty compounds quickly.

This would be fun.  Its far more than needed to pass the test.  I believe
in writing as little code as possible, then iterating.  :-)

Scott Penberthy
August, 2017
