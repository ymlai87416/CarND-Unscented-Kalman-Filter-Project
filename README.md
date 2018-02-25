[//]: # (Image References)

[image1]: ./output/CTRV_model.png "CTRV model"
[image2]: ./output/state_vector.PNG "state vector"
[image3]: ./output/lidar_measurement.PNG "Lidar measurement"
[image4]: ./output/lidar_init_state.PNG "Lidar initial state"
[image5]: ./output/lidar_init_covariance_matrix.PNG "Lidar covariance matrix"
[image6]: ./output/radar_measurement.PNG "Radar measurement"
[image7]: ./output/radar_init_state.PNG "Radar initial state"
[image8]: ./output/radar_init_covariance_matrix.PNG "Radar covariance matrix"
[image9]: ./output/augmented_state_vector.PNG "Augmented state"
[image10]: ./output/augmented_covariance_matrix.PNG "Augmented convariance matrix"
[image11]: ./output/yaw_dot_k.PNG "yaw_dot_k"
[image12]: ./output/sigma_point_prediction_1.PNG "sigma point prediction non zero"
[image13]: ./output/sigma_point_prediction_2.PNG "sigma_point_prediction zero"
[image14]: ./output/sigma_point_predicted_mean.PNG "sigma pooint predicted mean"
[image15]: ./output/sigma_point_predicted_covariance_matrix.PNG "sigma point predicted covariance matrix"
[image16]: ./output/lidar_measurement.PNG "lidar measurement"
[image17]: ./output/lidar_measurement_h.PNG "lidar measurement model"
[image18]: ./output/radar_measurement.PNG "radar measurement"
[image19]: ./output/radar_measurement_h.PNG "radar measurement model"
[image20]: ./output/predict_measurement_model.PNG "predicted measurement model"
[image21]: ./output/predict_covariance.PNG "predicted covariance"
[image22]: ./output/cross_correlation_matrix.PNG "cross correlation matrix"
[image23]: ./output/kalman_gain_k.PNG "Kalman gain K"
[image24]: ./output/update_state.PNG "update state"
[image25]: ./output/covariance_matrix_update.PNG "covariance matrix update"
[image26]: ./output/delta_t.PNG "delta_t"

# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

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

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

### Compiling


### Accuracy


### General processing flow

#### CTRV - Constant turn rate velocity model
The following graph shows the CTRV model, and the corresponding parameters.

![alt text][image1]

The state vector is

![alt text][image2]


### Initial measurement
#### Lidar measurement

Given the measurement vector z:

![alt text][image3]

The initial state is:

![alt text][image4]

The initial covariance matrix is:

![alt text][image5]

#### Radar measurement
Given the measurement vector z:

![alt text][image6]

The initial state is:

![alt text][image7]

The initial covariance matrix is:

![alt text][image8]

### Predict and update loop

#### UKF augmentation

Augmented state

![alt text][image9]

Augmented covariance matrix

![alt text][image10]

#### Sigma point prediction

Given ![alt text][image26], and sigma points, the UKF predict where will be the sigma points when the time is t + 
![alt text][image26]

if ![alt text][image11] is not zero:

![alt text][image12] 

if ![alt text][image11] is zero:

![alt text][image13] 

#### Predict mean and covariance

To calculate predicted mean and predicted covariance, we first calculate the weights first.
The weights are defined as:
 
Predicted mean:

![alt text][image14] 

Predicted covariance:

![alt text][image15] 

#### Measurement model

The measurement model transform state vector back to measurement vector, In this project, the sensor
fusion filter collects reading from Lidar and Radar.

##### Lidar

The measurement vector is:

![alt text][image16]

The measurement model is:

![alt text][image17] 


##### Radar

The measurement vector is:

![alt text][image18]

The measurement model is:

![alt text][image19]

Predicted measurement model

![alt text][image20]

Predicted covariance

![alt text][image21]

#### UKF update

Cross-correlation matrix

![alt text][image22]

Kalman gain K

![alt text][image23]

Update state

![alt text][image24]

Covariance matrix update

![alt text][image25]

### Performance analysis (Combined vs Lidar vs Radar)



#### Comparison between ground truth and prediction value

#### Lidar only performance

#### Radar only performance

#### Conclusion

### Comparison with the Extended Kalman filter

### Images
1. Sigma point generation
    a. What is x, P, Q, vk and the sigma points X
2. The transformation for C???