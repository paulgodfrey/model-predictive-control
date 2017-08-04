# Model Predictive Control

Model predictive control reframes the task of following a target trajectory as an optimization problem. The solution to that problem is to find the optimal trajectory for the vehicle.

The optimal trajectory can be found by taking the vehicles current state and then simulating different control inputs (steering, accelerating, braking) to see how they impact the vehicles trajectory.

In this controller I've used kinematic motion model to predict the vehicles trajectory under different control inputs. Using this model you can be predict the vehicles trajectory given it's previous state:

### Predicting a vehicles trajectory at timestep t + 1
```
x​t + 1 ​​= xt ​​+ vt ​​∗ cos(psit) ∗ dt // x coordinate
y​t + 1 ​​= yt ​​+ vt ​​∗ sin(psi​t) ∗ dt // y coordinate
psit + 1 ​​= psit ​​+ ​L​f/​vt ​​∗ δt ​​∗ dt // heading, δ is steering angle
v​t + 1 ​​= v​t ​​+ a​t ∗ dt // velocity
```

Note: When calculating psi​t + 1 we use an additional variable Lf to represent the difference between the front of the vehicle and its center of gravity (the larger the vehicle, the slower its turn rate).

Once you know the simulated trajectory for a given set of control inputs you can compute the crosstrack (cte) and orientation (epsi) errors. The path with the minimum cost will be closest to the target trajectory.

### Calculate the orientation error (epsi​) at timestep t + 1:
```
epsit = psi​t - - ψ​dest // current eψ is desired orientation subtracted from current orientation
vt/Lf * steert * dt // change in error caused by vehicles movement
epsi​t + 1 = epsit + vt/Lf * steert * dt
```

### Calculate the crosstrack error at timestep t + 1:
```
ctet = f(xt) - yt // current cte
vt * sin(epsit) * dt // change in error caused by vehicles movement
ctet + 1 = f(xt) - yt + (vt * sin(epsit) * dt)
```

When calculating the cost for steering and speed across a trajectory it's important to penalize jerky steering or rapid changes in velocity (think of every terrible yellow taxi ride you've ever had). This is accomplished by tuning the coefficients when calculating cost for those inputs (see MPC.cpp lines 50-75).

```
// cost based on the reference state.
for (int t = 0; t < N; t++) {
  // increase coefficients to magnify cost of control variables
  fg[0] += 2000*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += 2000*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}
```

### Adding constraints to simulated input

As part of the model for simulating trajectory it's also important to introduce constraints on your input controls both for computational efficiency and to emulate real world inputs as closely as possible. As an example when you're driving a vehicle it's never going to be turn on a dime at a 90 degree angle — for this reason I've constrained the steering wheel angle in the simulations to -25 <-> 25 degrees.   

```
// upper and lower limits of delta are set to -25 and 25
// degrees (values in radians)
for (int i = delta_start; i < a_start; i++) {
  vars_lowerbound[i] = -0.436332*Lf;
  vars_upperbound[i] = 0.436332*Lf;
}
```

I chose to calculate the simulated trajectories over a time horizon of one second (N=10 and dt=0.1). Because the number of timesteps (N) directly correlates to computational cost finding the lowest possible number while still providing good results is key.

After finding the optimal trajectory, steering and throttle adjustments are sent to the actuator. While the model has calculated a series of controls over the time horizon (N*dt) everything outside of the first timestep is thrown out (note: in MPC.cpp I use the actuator controls at the second timestep - this is because i'm simulating a 100ms delay and N*dt = 1 so solution at s+1 is for 100ms in future).

Instead of using the old predicted trajectory, you take the new state from vehicle sensors (x, y, psi, v) and calculate a new optimal trajectory (i.e. this whole process runs in a continuous loop). This approach is also called "receding horizon control" because you're constantly calculating inputs over a future horizon.

Note: While the track simulator (https://github.com/udacity/self-driving-car-sim/releases) returns waypoints using the maps coordinate system it's important to transform the target trajectory so that it's horizontal to the car:

```
// transform waypoints so they are horizontal to car
for(int i = 0; i < ptsx.size(); i++) {

  // shift car reference angle to 90 degrees
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;

  // rotate all points so that psi is 0
  ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
  ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
}
```


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
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
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
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
