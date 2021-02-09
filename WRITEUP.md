# CarND-Path-Planning-Project Model Documentation
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


Overview
---

In this writeup I describe implementation of the path planning algorithm, used to drive a cat in the simulator.
The code was developed for the project of the Udacity

## Path planning rules

Actually, all the planning behavior during a highway driving can be described by a small set of simple rules. At first, here is crucial rules, that can never be violated:

* Do not collide into other objects on the road
* Do not move faster then a current speed limitation
* Drive only in lanes at the right part of the road
* Do not exceed absolute maximum acceleration/deceleration limits, as it leads to wheel skidding

Other important rules *could* be violated in a particular situation to fulfill the crucial ones above:

* When the current speed is higher or lower that a necessary, accelerate or decelerate smoothly to prevent jerk
* When driving straight (e.g. not changing lanes): keep close to the lane center
* If all lanes are free: 
  * Prefer driving at the highest legal speed minus some small safety delta
  * If there are 3 lanes or more: prefer driving in the central lane(s), as the leftmost and rightmost usually are more dangerous
* If there's an obstacle ahead:
  * If no free adjacent lane available - decelerate and maintain a safe distance to the obstacle ahead
  * If there is a free adjacent lane - go to that lane via a smooth trajectory and cross the lane boundaries faster than in 3 seconds
* Avoid changing lanes too often (say, once in several seconds)
* Persist in completion of the lane changing, e.g. do not return back to the initial lane if the maneuver has been executing already
* When changing lanes, prefer doing this with acceleration, not deceleration


## Implementation

All the necessary code is in the `Planner` class in `planner.h` and `planner.cpp` files. The `main.cpp` file left almost untouched (compared to the upstream master [here](https://github.com/udacity/CarND-Path-Planning-Project)), except the call to the `Planner` class inside the telemetry processing part.
The `Planner` class the main entry point at the `process_telemetry(...)` method, which takes a telemetry JSON as an argument. After calling this method, it is possible to retrieve the new generated trajectory as 2 arrays of floating point numbers, representing 'x' and 'y' global map coordinates respectively, via calling `x_trajectory_points()` and `y_trajectory_points()`.

### Algorithm section

The code of the `process_telemetry(...)` method actually literally describes everything that happens each timestep:

```cpp
    void Planner::process_telemetry(const nlohmann::json &telemetry)
    {
      clear_trajectory();
      parse_ego(telemetry);
      parse_previous_path(telemetry);
      parse_obstacles(telemetry);
      update_allowed_speed();
      update_lane_change_counter();
      future_lane_ = choose_best_lane();
            ...
      generate_trajectory();
    }
```

### Telemetry parsing

The ego car status (coordinates, angle, speed) is parsed from telemetry info and stored in a dedicated wrapper. As the ego car's `d` coordinate is known, it is possible to detect in which lane it is situated using `update_current_lane()` method.
The same for other cars on the road (which are listed in 'sensor fusion' part of the telemetry JSON). I call them 'road objects' as, naturally, there might be not only cars on the road, but they still would have corrdinates and speed. While parsing the sensor fusion, each object is being put in either the `obstacles_ahead_` or `obstacles_behind_` list according to the object's `s` coordinate, and, inside of each of those lists there are 3 sets, each for a corresponding lane. According to the object's `d` coordinate it is being put inside of one of the sets. The sets automatically sort the obstacles, so it is easy to retrieve a closest object in any lane ahead or behind the ego car. As there is no too many objects, keeping the sorted sets costs not much in terms of performance, but significantly simplifies the rest of decision maker code.

### Speed control and collision prevention

As the speed control _is_ a part of collision prevention, and these two parts are crucial for a car's safety, I have encapsulated these two behaviors inside the `update_allowed_speed()` method. The method takes the closest obstacle in the current lane. If the ego car is too close to it, the allowed speed must be 0 (brake to stop), but I prefer having some very small number like 0.05 m/s just for preventing spline calculation crashes. Anyway, if the code reaches this point, it basically means the path planning was in vane, something bad has already happened and we only need to minimize the catastrophe.
If the object ahead is far enough, we compute its speed and then set the ego speed close to it: a bit higher, if we still have some large gap and want to approach the object ahead, or a bit lower, if we want more space. This speed would be kept until there is an opportunity to change lane.

### State machine

The lessons suggest implementing a finite state machine with distinct states such as 'keep lane', 'change lan left', etc.
However, while implementing a decision maker for choosing the best lane to drive in, I realized that it would be more natural to choose between the current lane and its adjacent lanes _each time_ we update the sensor information and car status. As a result, I have discarded the explicit finite state machine.
Practically, there are only 2 'states' - either the car drives in a lane straigh, or it changes the lane to another one.

### Lane selection

The lane selector in the `choose_best_lane()` method is the most sophisticated part of the `Planner`, but still it is relatively simple. At each timestep, we need to decide, which of the lanes - the current one or left/right adjacent one - is the best to move in. For example, if there are 3 lanes, and there is no objects ahead in only one of them, this would be definitely the best choice. *If* we are already in the leftmost lane, we can only consider the current (left) and the adjacent (center) lane, and *if* both of them have cars ahead of us, it is natural to choose the lane, in which the car ahead drives *faster*, or, *if* both cars in both lanes drive with equal speed, prefer the lane where the car ahead is *further*, but *if* both distance and speed are the same, prefer to just do nothing... 
Please count the `if` statements in the previous sentence. 
It is almost impossible to implement a finite state machine, covering as much cases as necessary, so the decision here is being made based on a sum of several cost functions. The lane, which has the higher cost (named `quality`), becomes a `future_lane_` and the ego car then goes to that lane. If `future_lane_` is equal to the `current_lane_`, the "state" can be considered as "keep lane", otherwise the car is in "changing lanes" state.
In a nutshell, a lane has maximum `quality` if it is free both ahead and behind the ego car, and the lowes `quality` (up to negative) if there are cars in that lane that could collide.
Having cost functions, it is easy to fine-tune the behavior in tons of situations. For example, if we want to change line, but there is a car behind the ego in that lane AND it is approaching (e.g. its speed is higher than the ego speed) - changing lane can be dangerous, even if the distance to that car is still large enough. On the other hand, even with a tight gap, a car behind that moves slower than ego, will not collide.
It is also easy to add some prize for driving in a center lane and penalize changing lanes too often (there is a time counter for keeping track of time for this purpose).

### Trajectory generation

Knowing the current ego car position and speed, and also knowing the future lane and allowed speed, it is possible to compute the exact trajectory points. The `generate_trajectory()` method is based mostly on the "Q&A" section of the lesson. What I do here is:
1. Take 2 last points of the previous trajectory or generate a pair of points using CCP coordinates.
2. Add 3 more points, each situated at the center of `future_lane_` ('d' coordinate), with the step of 30m ('s' coordinate).
3. Rotate these 5 points so the CCP angle becomes 0 rad.
4. Generate a spline through the 5 points.
5. The spline sets the geometric path, that should be close to the optimal path in terms of smoothness.
6. Compute the distance between trajectory points, that result in a necessary speed; each next point should be a bit further if ego car accelerates, and a bit closer, if decelerates; the exact distance increment between each two points is calculated using speed and acceleration limits.
7. All the points of the trajectory are rotated back to the global map coordinate system.

