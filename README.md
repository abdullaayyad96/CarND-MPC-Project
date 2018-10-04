# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Picture Definition)
[model]: https://user-images.githubusercontent.com/37302013/46427017-2a563e00-c77b-11e8-9c97-f41667ea51dc.JPG



## Project Discription

This project is part of UDacity's Self-Driving Car Engineer Nanodegree Program. The purpose is to develop and MPC controller capable of running the vehicle in a track. The project was developed to meet the rubric found [here](https://review.udacity.com/#!/rubrics/896/view).

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

## The model

A discrete time kinematic model was utilized for the Model Predictive Controller including four states (x & y position, orientation, and velocity) as shown below:

![kinematic model][model]

Where x and y represent the position in 2D coordinates, v is the speed of the vehicle, a is the acceleration, dt is the time step, psi represents the car's orientation and delta is the steering angle. 

The errors to minimize by the controller are the cross-track error (CTE) and orientation error (epsi) which were defined as the difference between the current states and the ideal states from a fitted polynomial as will be discussed below.

## Polynomial Fitting and MPC Preprocessing

The purpose of the MPC is to provide proper control commands to minimize the cross track and orientation errors. In order to do so, a desired or an optimum path must be generated. The desired path is described by a third order polynomial which is obtained by fitting several points on the desired track. The function for fitting the polynomials can be observed in the polyfit function in lines 60-79 in "src/main.cpp". 

Prior to passing the desired and current states to the polynomial fitting function or the MPC, these states are converted from the global coordinate frame to the car's coordinate frame for simplicity. This is done using the local_coordnate function in lines 45-54 in "src/main.cpp". Additionally, initial values of the errors are calculated assuming that the current states of position and orientation would be zero in the car's coordinate frame. 


## MPC design and tuning

The implementation of the MPC can be seen in "src/MPC.cpp". MPC can be considered as a constrained optimization problem and CppAD and Ipopt were used to solve the optimization problem. The purpose of the MPC is to minimize a cost function while adhering to constraints set by the physical model or limitations. Thus, careful design of the cost function becomes crucial and it's implementation can be seen in lines 50-84 in "src/MPC.cpp". In this project, the cost function was a weighted sum of quadratic functions of:
- cross track error
- orientation error
- velocity error
- steering angle
- vehicle's acceleration/throttle
- variations in steering angle
- variations in acceleration/throttle

The weights were initially selected as the reciprocal of the accepted range of error for each of the elements mentioned above. While this provided satisfactory results in low speeds, Excessive tuning was needed to accommodate for higher speeds. 

Another important parameter tuning required for the MPC are the number of steps N and the optimizer timestep dt which in turn determine the prediction horizon of the MPC. First for dt, it was set as 100ms which is identical to the latency assumed in the system (latency is implemented in line 187 of "src/main.cpp" to make the simulation more realistic) as it would not be very practically beneficial to set the timestep less than the system's latency. As for the number of steps N, 10 provided a good balance between computational time and performance, as going below that can reduce the controller's performance while increasing it highly increases the computational cost.

## MPC with Latency

As mentioned in the previous section, a latency of 100ms was introduced in the system to mimic a real vehicle. This latency can dramatically change the response of the system if not properly accounted for. In order to compensate for the latency, additional constaints were added to the MPC in which the current control inputs (steering angle and throttle) cannot be changed for the duration of the latency (which is identical to the MPC timestep as mentioned in previous section). Adding these constraints greatly improved the performance of the MPC under latency as heaving oscillations were observed prior to implementing these constraints.
