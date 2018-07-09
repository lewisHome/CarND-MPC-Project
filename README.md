# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Rubric Points

## The Model

The model used in the code is a kinematic model theat ignores tire forces, gravity and mass. While the model is more simpler than dynamic models it is more portable between different vehicles. Considering tire forces can change dependant upon weather conditions and wear while the mass of the vehicle is highly dependant upon how the vehicle is loaded.

The model equations used are 

      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      
Here `[t]` is the current time and `[t+1]` is the susequent time step which is `dt` seconds later.
`x_` , `y_` and `psi_` describe the position and orientation of the vehicle.

`v` is the velocity of the vehicle
`a` is the acceleration of the vehicle
`delta` is the steering anfle of the vehicle

`cte` is the cross track error this describes how far the vehicle is from the road center line
`epsi` is the orientation error and describes how far the vehicle angle deviates from the angle of the road centre line

##Time step Length and Elapsed Duration (N & dt)
I settled upon using a timestep `dt` of 0.1 with the Number of steps `N` as 10. I found if I increased or decreased the number of steps or timestep length the vehicle quickly lost control and crashed.

##Polynomial Fitting and MPC Preprocessing

The waypoints that come out of the json message are in a global co-ordinate reference frame. These points are changed to the car reference frame where the car has an `x_, y_, psi_` value of `0,0,0` and the `cte` and `epsi` are the offset and orientation of a polynomial fitted through the transformed waypoints.

##Model Predictive Control with Latency
The model has a latency of 100ms to actuator actuation at line 131 in main.cpp. The maximum speed of the vehicle is limited to 125mph and this ensures that the vehicle has not travelled to great a distance before actuation occurs.