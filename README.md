### **Unscented Kalman Filter**

## Writeup
---

### Udacity Course, October 2017 cohort

**Self-Driving Car Engineer Nanodegree Program**

**Project 'Unscented Kalman Filter', March 2018**

**Claus H. Rasmussen**

---

Implement an unscented Kalman filter in C++ using the CTRV motion model.
Two bicycle simulation data set, Data set 1 and Data set 2 (Ascii text files), are used with the Term 2 Simulator.

[//]: # (Image References)

[Ds1_both]: ./UKF.png "Data set 1, using both radar and lidar data"
[Ds1_radar]: ./UKF.png "Data set 1, using only radar data"
[Ds1_lidar]: ./UKF.png "Data set 1, using only lidar data"
[Ds2_both]: ./UKF_Dataset2_both.png "Data set 1, using both radar and lidar data"

Like the Extended Kalman Filter (EKF), the Unscented Kalman Filter (UKF) have the same three steps:

* Initialization
* Prediction
* Update

These steps are coded in the ukf.cpp file.

This project has used the following initialization parameters:

*Initial state covariance matrix*
  **P_** =  1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1

*Process noise standard deviation longitudinal acceleration in m/s^2*
  **std_a_** = 5

*Process noise standard deviation yaw acceleration in rad/s^2*
  **std_yawdd_** = 0.4


The Root Mean Square Error (**RMSE**) are calculated for position X & Y and Velocity VX & VY by comparing the predicted UKF values compared to the Ground True values, which are supplied with the test data set. In this project, the combined radar and lidar accuracy must be smaller than the Metric values.

---

**Data set 1:**

| Both radar and lidar | RMSE    | Metric    |
|:--------------------:|:-------:|:---------:|
| X                    | **0.0778**  | 0.09      |
| Y                    | **0.0874**  | 0.10      |
| VX                   | **0.3837**  | 0.40      |
| VY                   | **0.2769**  | 0.30      |
|:--------------------:|:-------:|:---------:|

| Only radar           | RMSE    | Metric    |
|:--------------------:|:-------:|:---------:|
| X                    | 0.2491  | 0.09      |
| Y                    | 0.3336  | 0.10      |
| VX                   | 0.4717  | 0.40      |
| VY                   | 0.4424  | 0.30      |
|:--------------------:|:-------:|:---------:|

| Only lidar           | RMSE    | Metric    |
|:--------------------:|:-------:|:---------:|
| X                    | 0.1232  | 0.09      |
| Y                    | 0.1024  | 0.10      |
| VX                   | 0.6354  | 0.40      |
| VY                   | 0.2979  | 0.30      |
|:--------------------:|:-------:|:---------:|


*Using both radar and lidar data.*

![alt text][Ds1_both]


*Using only radar data.*

![alt text][Ds1_radar]


*Using only lidar data.*

![alt text][Ds1_lidar]

---

**Data set 2:**

Data set 2 goes in the opposite direction and has irregular timesteps. The implemented UKF also handles this dataset:

*Using both radar and lidar data.*

![alt text][Ds2_both]


---
---
---

## **This section was provided by Udacity to help set up the coding environment etc.**


In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
