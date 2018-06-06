# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

The MPC-project is the last project of the second term of the Udacity Self-Driving Car
Engineer Nanodegree. The challenge is to apply Predictive Control over a vehicle driving
a track in a simulator. First, I want to describe the model and how it is implemented.
Then, I want to present how the waypoints for the predicted trajectory are selected by
tuning the parameters for the timestep length and the elapsed duration. Then, I shortly
want to address the polynomial fitting and MPC preprocessing I applied. Finally, the
approach for considering latency will be argued.

### The model:
The model used in this project is an optimizer that minimizes a defined cost function
and returns control inputs (steering / acceleration / braking). The cost function
evaluates the deviation of the desired trajectory to a predicted trajectory, which is
evaluated by the model. The predicted trajectory is calculated with kinematic formulas.
The model evaluates the predicted trajectory repeatedly for every discrete timestep t
(see next chapter) and updates the control inputs with this rate.

The model takes the current state of the vehicle as input. The current state consists of
the vehicle’s position in 2D (x and y coordinates), the orientation, the velocity, the cross
track error and the orientation error. The last two properties describe the distance from
the desired trajectory and the difference to the desired orientation angle, respectively.

Furthermore, constrains for the actuators are postulated. The characteristics of the car
are assumed to allow for steering angles up to 25 degrees and restrict acceleration and
deceleration, which are realistic assumptions for real cars.

The cost function encompasses different costs that penalize divergence from desired
behavior. First, deviations from the reference state – the desired trajectory – are
penalized. Those deviations are the cross track error and the heading error. The cost
function takes also deviations from the desired speed into account to prevent the car
from stopping. Furthermore, there is a penalty for not desired actuator behavior. Strong
steering angles and strong variations within the steering angles are penalized because
those could lead to unsafe behavior and comprising comfort.

### Timestep Length and Elapsed Duration (N and dt)
The considered future horizon is defined by choosing the number and the time intervals
of waypoints of the trajectory. I chose the number of time intervals to N=10 and the time
interval to t=0.1s so the future path is predicted in 10 waypoints every 100ms. Thus, one
second of the future trajectory is considered. I chose those values for the parameters N
and t because I observed the best behavior. A larger N (e.g. 15) while keeping t
constantly leads to a longer planning horizon but the car gets more instable and tends to
move from the left to the right similar to the behavior of a P-controller or a PIDcontroller
with to little differential share. A smaller N endangers the car to loose track.
Furthermore, I analyzed the variation of t by keeping N fix. I evaluated t=0.1s to be a
good value in combination with N=10 because a smaller t (e.g. 0.05) leads to instable
behavior – similar to increasing N while keeping t fix. A higher t (e.g. 0.2) leads to slow
driving and finally the abandoning of the track.

Summing up I found out that a large planning horizon (looking at the trajectory more
than 1 second in the future) does not lead to satisfying results. Keeping other
parameters in the cost and constrains constant, a planning horizon of 2 seconds leads to
slow driving and eventually loosing track in sharp curves. More than 10 waypoints (e.g.
the combination N = 20 and t = 0.05) lead to instable behavior. The optimum appears to
be at N = 10 and t = 0.01 although a tuning of other parameters could imaginably shift
this optimum.

### Polynomial Fitting and MPC Preprocessing
The incoming data from the simulator include the x and y positions of the waypoints. A
third order polynomial is fitted through them because it reflects real world curvature
quite well. In order to simplify calculations, I have applied some preprocessing. In this
preprocessing the perspective is transformed into the one of the ego vehicle so that xand
y-coordinates and the orientation angle psi are always zero. I realized this by
subtracting from each waypoint the x- and y-positions of the car and doing some
trigonometry. The transformed state vector with px=0, py=0 and psi=0 is used for the
visualization. You can see a screenshot below. A yellow line displays the desired
trajectory while the green line shows the solver’s solution of the waypoints.

### Model Predictive Control with Latency
In order to do the calculations necessary for model predictive control, there is always
latency. Realistic values are at about 100ms, which I used in this project. To account for
latency the control must look at the steering and acceleration output of the solver
100ms ago instead of the current one. Since the timestep t is set to 100ms as well, the
implementation is quite simple. The actuations (steering angle and acceleration) take
the actuation from the preceding timestep for the current one.

Comment: To run the code you need to add the Eigen-library to src.




## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

