# CarND-Controls-MPC
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

#### Daniel Prado Rodriguez
#### Feb'2017 Cohort
---
## Intro
This project implements an MPC (Model Predictive Control) Controller that drives Udacity's Car Simulator around a closed virtual track, following specific waypoints that mark the middle line along the track. Waypoints info are provided online by the simulator server.

Unlike the PID controller subject of the previous project, the MPC keeps the future predicted values into account when optimizing the actuators for the current timeslot (iteration). This is done by optimizing a finite time-horizon, in our case the predicted vs. desired trajectory of the car, then apply the actuators for the current iteration, and optimize the finite time-horizon trajectory again in next iteration.

As an additional difficulty, the controller simulates a 100ms latency in the actuators, which is a situation that would be found in a real system.

See my final results in the following video:
[![Project 5 Output](https://img.youtube.com/vi/aZsFpR6GgnA/0.jpg)](https://www.youtube.com/watch?v=aZsFpR6GgnA)

### Project Steps
The code follows the following functional structure:
* Fit a polynomial line (desired trajectory) along the track waypoints provided by the simulator.
* Evaluate the current car state with respect to the reference desired trajectory.
* Set state, polynomial coeffs, variables and constraints for the MPC.
* Solve the MPC calculations
* Apply the result MPC actuators (Steer and Throttle).
* Apply the result MPC values to draw the predicted trajectory.

## Model Description

**Rubric Point - The Model** : Student describes their model in detail. This includes the state, actuators and update equations.

The project applies a basic kinematic model for the simulated car. We need to keep in mind that a kinematic model is a simplification of a dynamic model that ignores tire forces, gravity, and mass. This allows to drive the simulated car at a very high speeds of even 100 mph along quite sharp curves.

The model state is defined by the following variables, provided by the simulator. Shown here the simulator tag and (variable).
* x (px): car's current x-position in map coordinates.
* y (py): car's current y-position in map coordinates.
* psi (psi): car's orientation
* speed (v) : car's longitudinal speed.

Besides, the simulator provides additional data:
* ptsx & ptsy: x and y positions of waypoints (desired/reference trajectory) in map coordinates.
* steering_angle (delta) : steering wheel angle of the car, not to confuse with psi (actual orientation of the car).
* throttle (a) : current throttle.
Note: I find it confussing how in the project we are expected to assimilate that acceleration (a) is the same concept that the throttle value. Obviously both are related, but not in the same units.

#### Polynomial Fitting
The project uses the car's coordinate system. As shown above the first step is to fit the polynomial line along the track waypoints, and this is done in vehicle coordinates. To do so, we shift the origin to the current poistion of the vehicle and perform rotation to align the x-axis with the heading direction.
The polynomial fitting is done to a third order, that is enough to model a car trajectory in most cases. The transformation used is 
```
 X' =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
 Y' =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
```
where `X',Y'` denote coordinates in the vehicle coordinate system. The initial position of the car and heading direction are always zero in this system of reference. Thus the state of the car in the vehicle cordinate system would be
`state << 0, 0, 0, v, cte, epsi;`
initially, and assuming zero latency.

#### Dealing with Model Latency
The project code simulates actuator latency by introducing a 100ms delay before sending the actuator inputs back to the simualtor. 
In my implementation, I first made the model work with latency 0ms, but then when setting it back to 100ms, the car started to oscillate quite violently.
This is logical, because the optimized steer and throttle values are estimated based on the vehicle current state and not on the state after the latency time has transcurred.

My approach is to incorporate the latency model in the basic model by predicting the future car state after 100 ms time. This is done after the system of reference transformation explained in the previous section. Hence, the state prediceted after 100ms is calculated as follows:
```
    //Predicted car's state after latency transcurred
    // x, y and psi are all zero after transformation above
	double pred_px = 0.0 + v * latency; // Since psi is zero, cos(0) = 1
	const double pred_py = 0.0; // Since sin(0) = 0, (y + v * 0 * dt)
	double pred_psi = 0.0 + v * -delta / Lf * latency;
	double pred_v = v + a * latency;
	double pred_cte = cte + v * sin(epsi) * latency;
	double pred_epsi = epsi + v * -delta / Lf * latency;
```
After this transformation, the state is feed like this: 
`state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;`

#### MPC Model state Update and Cost Function
Finally the MPC model is defined by the following equations:
```
    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    // psi_[t+1] = psi[t] (+) v[t] / Lf * delta[t] * dt
    // v_[t+1] = v[t] + a[t] * dt
    // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    // epsi[t+1] = psi[t] - psides[t] (+) v[t] * delta[t] / Lf * dt
```
Note that in my code I have updated the sign `(+)` of the `delta[t]` to take into account how the simulator expects the steer values (opposite to our coordinates system).

, the following constraints:
```
//  -0.436332 < delta < 0.436332 // - that is [-25,+25] degrees, in radians.
//  -1.0 < trottle < 1.0 //
```
, and the following cost function (pseudocode shown). Note that multipliers have been tuned manually in order to give more or less weight to cost components.
```
   Cost  = SUM(i) of { 2000 * cte(i)^2
              + 2000 * epsi(i)^2 
              + (v(i)-v_ref)^2
              + 100 * delta(i)^2 
              + 10 * a(i)^2 
              + 500 * [delta(i+1)-delta(i)] 
              + 10 * [a(i+1)-a(i)] }
```

Finally we take the following output of the MPC Solver function:
* Actuators: these are the `throttle` and `steer` values to be applied in the current iteration.
* `{mpc_x, mpc_y}` is the car trajectory predicted by the model. Note how the model tries to approach the reference trajectory as much as possible (also depending on how we have defined the cost function - for example, if we gave too little weight to the CTE component, then the car could follow a trajectory parallel to the reference, but not overlapping it.).

---

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
