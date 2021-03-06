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
[image27]: ./output/generating_sigma_points.PNG "generating sigma points"

[image28]: ./output/NIS_laser_na_3_ny_1.png "NIS laser low"
[image29]: ./output/NIS_laser_na_6_ny_1.png "NIS laser normal"
[image30]: ./output/NIS_laser_na_30_ny_30.png "NIS laser high"
[image31]: ./output/NIS_radar_na_3_ny_1.png "NIS radar low"
[image32]: ./output/NIS_radar_na_6_ny_1.png "NIS radar normal"
[image33]: ./output/NIS_radar_na_30_ny_30.png "NIS radar high"

[image34]: ./output/ukf_px_comparison.png "UKF Px"
[image35]: ./output/ukf_py_comparison.png "UKF Py"
[image36]: ./output/ukf_vx_comparison.png "UKF Vx"
[image37]: ./output/ukf_vy_comparison.png "UKF Vy"
[image38]: ./output/ukf_yaw_comparison.png "UKF yaw"
[image39]: ./output/ukf_yaw_dot_comparison.png "UKF yaw dot"
[image40]: ./output/ekf_px_comparison.png "EKF Px"
[image41]: ./output/ekf_py_comparison.png "EKF Py"
[image42]: ./output/ekf_vx_comparison.png "EKF Vx"
[image43]: ./output/ekf_vy_comparison.png "EKF Vy"

[image44]: ./output/data_set_1_laser_acc.png "dataset 1 laser"
[image45]: ./output/data_set_1_radar_acc.png "dataset 1 radar"
[image46]: ./output/data_set_2_laser_acc.png "dataset 2 laser"
[image47]: ./output/data_set_2_radar_acc.png "dataset 2 radar"
[image48]: ./output/data_set_1_acc.png "dataset 1"
[image49]: ./output/data_set_2_acc.png "dataset 2"

[image50]: ./output/nis_eq.PNG "nis equation"
[image51]: ./output/ukf_performance.png "ukf performance"

[image52]: ./output/sigma_point_weight.PNG "ukf sigma points weightings"

[image53]: ./output/symbols/posterior_result.png "posterior_result"
[image54]: ./output/symbols/ms2.PNG "ms2"
[image55]: ./output/symbols/rads2.PNG "rads2"

[image56]: ./output/symbols/small_process_noise.PNG "small"
[image57]: ./output/symbols/normal_process_noise.PNG "normal"
[image58]: ./output/symbols/high_process_noise.PNG "high"

[image59]: ./output/symbols/chi_square.PNG "chi square"

[image60]: ./output/symbols/px.png "px"
[image61]: ./output/symbols/py.png "py"
[image62]: ./output/symbols/yaw.png "yaw"
[image63]: ./output/symbols/yaw_dot.png "yaw dot"


# Unscented Kalman Filter Project
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

Refer to the instruction in section "Basic Build Instructions" to compile the code and run.

### Accuracy

| Dataset 1           | Dataset 2   |
|:-----:| :-----:|
| ![alt text][image48] | ![alt text][image49] |

UKF tracks the vehicle position, velocity and yaw pretty well. For yaw dot (turn rate), the predictions are not that 
accurate as others.

![alt text][image51]

### General processing flow

#### CTRV - Constant turn rate velocity model
CTRV model assumes that the vehicle is driving at a constant turn rate ![alt text][image63], ![alt text][image62] is the angle of the steering angle. ![alt text][image60] and ![alt text][image61] 
are the position of the vehicle and v is the linear velocity along the steering direction.

The following graph describes the parameters in the CTRV model.

![alt text][image1]

The state vector is

![alt text][image2]


### Initial measurement
#### Lidar measurement

Lidar is capable of measuring the position of the vehicle.

Given the measurement vector z:

![alt text][image3]

The initial state is:

![alt text][image4]

The initial covariance matrix is:

![alt text][image5]

#### Radar measurement

Radar is capable of measuring the position of the polar coordinate and the velocity away from the radar of the vehicle.

Given the measurement vector z:

![alt text][image6]

The initial state is:

![alt text][image7]

The initial covariance matrix is:

![alt text][image8]

### Predict and update loop

UKF consists of a loop which predicts the state of the vehicle and updates the state using the sensor measurements.

#### UKF augmentation

First, UKF augments the state of the vehicle to include the process noise.

Augmented state

![alt text][image9]

Augmented covariance matrix

![alt text][image10]

#### Generating Sigma points
After the augmentation, UKF generates 15 sigma points by the following equation. 

![alt text][image27]

#### Sigma point prediction

Given ![alt text][image26], and sigma points, the UKF predict where will be the sigma points when the time is t + 
![alt text][image26] using the process model.

Here is the process model:

if ![alt text][image11] is not zero:

![alt text][image12] 

if ![alt text][image11] is zero:

![alt text][image13] 

#### Predict mean and covariance

To calculate predicted mean and predicted covariance, UKF calculates the weightings first.
The weightings are calculated as:


![alt text][image52] 
 
Predicted mean:

![alt text][image14] 

Predicted covariance:

![alt text][image15] 

This completes the prediction part. In the next part, UKF uses the sensor measurements to enhance the predicted state 
and covariance matrix.

#### Measurement model

The measurement model transforms state vector back to measurement vector, In this project, the sensor
fusion algorithm collects reading from both lidar and radar.

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

Here ![alt text][image53] are the final result of the predict and update loop.

### Performance analysis (Combined vs Lidar vs Radar)

#### Lidar only performance

| Values        | Both           |Lidar   |
| ----- |:-----:| :-----:|
| Dataset 1      | ![alt text][image48] | ![alt text][image44] |
| Dataset 2      | ![alt text][image49] |   ![alt text][image46] |

#### Radar only performance

| Values        | Both           |Radar   |
| ----- |:-----:| :-----:|
| Dataset 1      | ![alt text][image48] | ![alt text][image45] |
| Dataset 2      | ![alt text][image49] |   ![alt text][image47] |


#### Conclusion
Using both lidar and radar measurements helps reduce the overall RMSE in both cases.

### Process Noise and Measurement Noise
There are 2 kinds of noise which are considered when creating the UKF. They are process noise and measurement noise.

Measurement noises are easy to find because they can be found in the manual of the lidar and radar.
In this project, lidar has the following measurement noises:

| Measurement        | Noise           |
| ------------- |:-------------:|
| position in x-axis      | 0.15 |
| position in y-axis      | 0.15      |

Measurement noises from radar are:

| Measurement        | Noise           |
| ------------- |:-------------:|
| polar coordinate - radius      | 0.3 |
| polar coordinate - angle      | 0.03      |
| velocity | 0.3     |

Process noises in this project are linear acceleration and yaw acceleration. 
To find out the linear acceleration, by statistics, we assume that the max acceleration is 9 ![alt text][image54].
we assume the noise of linear acceleration equals to sqrt(9) * 2 = 6 ![alt text][image54]
For yaw acceleration, as I have no relevant data on my hand, I assume it to be 1 ![alt text][image55].

To find out if I make the correct guess of these noise parameters, I make use of the tool called Noise Innovation Squared (NIS).
NIS is calculated as followed:

![alt text][image50]

I also have tried different sets of value for the process noise.

| Values        | Lidar           | Radar  |
| ------------- |:-------------:| :-----:|
| ![alt text][image56]      | ![alt text][image28] | ![alt text][image31] |
| ![alt text][image57]      | ![alt text][image29]      |   ![alt text][image32] |
| ![alt text][image58] | ![alt text][image30]      |    ![alt text][image33] |

| % of data points > ![alt text][image59]        | Lidar (dof=2)          | Radar (dof=3) |
| ------------- |:-------------:| :-----:|
| ![alt text][image56]      | 2.81% | 3.61% |
| ![alt text][image57]     | 2.81%      |  2.81%  |
| ![alt text][image58] | 0.80%      |  1.61%  |

All the charts look good and does not show that I overestimate and underestimate too much for both
process noise parameters. I further looked into the percentage of data points which has the NIS value larger than
95 percentile of the expected chi-square distribution, I found that ![alt text][image57] is a good set of parameters and 
this set of value also give a satisfactory low level of RMSE for vehicle position and velocity predictions. 


### Comparison with the Extended Kalman filter
Here, we compare EKF (Constant Velocity) against UKF (Constant Turn Rate Velocity) by showing how good both sensor fusion algorithms
track the vehicle position and velocity.

| UKF           | EKF  |
|:-------------:| :-----:|
| ![alt text][image34] | ![alt text][image40] |
| ![alt text][image35]      |   ![alt text][image41] |
| ![alt text][image36]      |    ![alt text][image42] |
| ![alt text][image37]      |    ![alt text][image43] |

From the above charts, we can see that both sensor fusion algorithms perform well in tracking vehicle position, but when it comes to 
tracking the velocity of the vehicle, the prediction of UKF is closer to the ground truth, and the EKF predictions
seems lagging behind the ground truth.

Compare to the RMSE given by both sensor fusion algorithm and the above charts, UKF is a better choice in tracking vehicle compares to EKF.

### Bonus challenge

I have also completed the "Catch the Run Away Car with UKF" bonus challenge, please refer to [here](https://github.com/ymlai87416/CarND-Catch-Run-Away-Car-UKF)

Here is the [result](https://youtu.be/40h_eBTMvLg) .