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

A discrete time kinematic model was utilizedfor the Model Predictive Controller including four states (x & y position, orientation, and velocity) as shown below:

![kinematic model][model]

Where x and y represent the position in 2D coordinates, v is the speed of the vehicle, a is the acceleration, dt is the time step, psi is represents the car's orientation and delta is the steering angle. 

The errors to minimize by the controller are the cross track error (CTE) and orientation error (epsi) which were defined as the differance between the current states and the ideal states from a fitted polynomial as will be discussed below.

## 
