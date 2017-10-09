# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Purpose
This repository containt Model Predictive Control project. It is a C++ implementation of kinematic model predictive controler. Additionally it is supposed to comply to [project rubric points described here](https://review.udacity.com/#!/rubrics/896/view)

## Implementation
Objective of model predictive control is to make car follow certain path described by number of waypoints with given coordinates, ar certain speed. Idea of this approach is to derive set of actuator values that would enable car to follow target path using kinematic motion model. Path must be described as polynomial function that fits all the waypoints.
There are 2 available actuators:
- Steering angle (affected by steering wheel) (delta)
- Acceleration (affected by throttle and brake) (a)

Model state consists of 6 parameters:
- X and Y coordinates of the car.
- Velocity (v)
- Orientation of the car (psi)
- Cross-track error, as distance between car and target path (cte)
- Orientation error, being angle between actual orientation of the car and orientation needed to follow target path

Kinematic model is described by following equations that govern transition from state (x0, y0, psi0, v0, cte0, epsi0) to next state (x1, y1, psi1, v1, cte1, epsi1) over discrete time interval dt.
- `x1 = x0 + v0 * cos(psi0) * dt`
- `y1 = y0 + v0 * sin(psi0) * dt`
- `psi1 = psi0 + v0 / Lf * delta0 * dt`
- `v1 = v0 + a0 * dt`
- `cte1 = f0 - y0 + v0 * sin(psi0) * dt`
- `epsi1 = psi0 - psides0 + v0 / Lf * delta0 * _dt`
Additionally, we have that Lf is length of the car, f0 is value of path polynomial for x0, and psides is desired orientation of the car. Model predictive control uses polynomial optimization to devise set of actuator values (delta, a) that would be applied in discrete moments to make car follow a trajectory that best satisfies certain set of criteria and constraints. Constraints are equations described above plus physical limitations of actuators and environment (perhaps we can prevent running off road by limiting cte). As for criteria function, it is weighted sum of following values: `cte^2`, `epsi^2`, `(v0 - v1)^2`, `v0 * (delta0 - delta1)^2`, `(a0 - a1)^2`, `delta0^2`, `a0^2`

## Parameter tuning
MPC has discrete time interval length (dt) and number of discrete intervals (N) as main parameters. Smaller value of dt provides for more granular control and enables better soultions, but it is constrained by physical actuator latency. Larger value of N means that optimizer delivers plan for larger segment of target path, and is therefore more likely to fit well. However if length of the path that is covered by optimizer solution significantly exceeds lenght of desired path, optimizer might return degenerate solution. This means that we want to choose N so that for given dt and target speed, lentgth of resultant path loosely matches length of target path. Following this reasoning I have arrived to N=17 and dt=0.1 for target speed of 25 m/s which made optimizer fit actuator values for next 42.5m.

## Polynomial Fitting and MPC Preprocessing
I have choosen to fit waypoints from simulator to polynomial of 4th degree.
Additionally, I have choosen to transform waypoint and car coordinates and orientation from simulator world reference to car nose reference. This transformation was necessary because simulator needs waypoints and projected trajectory sent like that in order to render visualization. So for car coordinates (px, py) orientation psi, and waypoint (x, y) we would have transformed waypoint (tx, ty) calculated as follows:
- `tx = (x - px) * cos(-psi) - (y - py) * sin(-psi)`
- `ty = (x - px) * sin(-psi) + (y - py) * cos(-psi)`

Once this is done, car position defined as (x, y, psi) becomes (0, 0, 0) which simplifies further calculations.

## Model Predictive Control with Latency
Aside from limiting sensible choices for dt, actuator latency also throws off kinematic model by causing car positon and orientation to shift from the moment when actuator values are calculated unti they are applied. I opted to deal with this by leveraging optimizer that is already used by MPC implementation as follows:
- Choose dt that matches latency ammount.
- Add one extra time interval to the begining of control plan.
- Constrain actuator values for first step to be the same as last actuator values that were sent to the car.
- Exclude cost of first step from total cost of optimized control plan.
- Use actuator values from step 2 of optimized control plan (as step 1 corresponds to period when car runs on prior input due to latency)




