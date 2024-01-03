<!-- omit from toc -->
# IMU-Based 3D Orientation Estimation

**Table of Contents**
- [Phase 1 - IMU Attitude Estimation](#phase-1---imu-attitude-estimation)
  - [Overview](#overview)
  - [Methodology](#methodology)
  - [Code Structure](#code-structure)
  - [Discussion](#discussion)
  - [Results](#results)
    - [Combined Results](#combined-results)
    - [Gyroscope-Based Orientation](#gyroscope-based-orientation)
    - [Accelerometer-Based Orientation](#accelerometer-based-orientation)
    - [Complementary Filter Orientation](#complementary-filter-orientation)
- [References](#references)
- [Designer Details](#designer-details)
- [License](#license)



This repository contains code for a 2 part project:
-  **Phase 1 - IMU Attitude Estimation:** We implement three methods to estimate the three-dimensional orientation/attitude. We are given data from an ArduIMU+ V2, a six-degree of freedom Inertial Measurement Unit (6-DoF IMU) sensor i.e., readings from a 3-axis gyroscope and a 3-axis accelerometer. We will estimate the underlying 3D orientation and compare it with the ground truth data given by a Vicon motion capture system based on three methods:
   -  Gyroscope Only Attitude Estimation
   -  Accelerometer Only Attitude Estimation
   -  Gyroscope + Accelerometer-Based Complimentary Filter Attitude Estimation

-  **Phase 2 - Waypoint Navigation and AprilTag Landing:** Design and implement a navigation and landing system for a simulated quadrotor in an environment. The quadrotor is equipped with the ability to navigate to the provided 3D position waypoint. We are required to autonomously navigate through a sequence of predefined waypoints, perform AprilTag scanning after we reach every waypoint, and execute a landing maneuver on an AprilTag with the ID value of `4`.

# Phase 1 - IMU Attitude Estimation
## Overview

The goal is to estimate the 3D orientation using data from a 6-DoF IMU sensor (3-axis gyroscope and 3-axis accelerometer) and compare it with ground truth data provided by a Vicon motion capture system.

## Methodology

1. **Read Data**: Read the raw IMU and Vicon Data from a `.mat` file.
  
    The IMU Data is of the form:
    
$$
\begin{bmatrix}
  a_{x} && a_{y} && a_{z} && \omega_{z} && \omega_{x} && \omega_{y}
\end{bmatrix}^{T}
$$

  The Vicon Data is stored in the form of a 3x3 Rotation Matrix, denoting `Z-Y-X` Euler Angles for orientation, and should be considered as ground truth for testing.

2. **Convert to SI Units**: We need to convert the Accelerometer and Gyroscope Data from Raw Values to SI Units using the following formulae:
  
$$
\begin{equation}
  \tilde{a_{x}} = \frac{a_{x} + b_{a,x}}{s_{x}}
\end{equation}
$$

where, 
- $\tilde{a_{x}}$ denotes acceleration in physical values
- $a_{x}$ denotes acceleration in raw values
- $b_{a,x}$ denotes bias factor
- $s_{x}$ denotes scale factor

  
$$
\begin{equation}
  \tilde{\omega_{x}} = \frac{3300}{1023} * \frac{\pi}{180} * 0.3 * (\omega - b_{g})
\end{equation}
$$

where, 
- $\tilde{\omega_{x}}$ denotes angular velocity in physical values
- $\omega_{x}$ denotes angular velocity in raw values
- $b_{g}$ denotes the bias factor (calculated as the average of the first few 100 values assuming IMU is at rest)

We also use this stage to convert the Vicon Data from the Rotation Matrix of `ZYX` format to Euler Angles using the following conversion:
<!-- 
$$
\begin{matrix}
  \cos{z}\cos{y} &&
  \cos{z}\sin{y}\sin{x} - \cos{x}\sin{z} &&
  \sin{z}\sin{x} + \cos{z}\cos{x}\sin{y} \\

  \cos{y}\sin{z} &&
  \cos{z}\cos{y}\cos{x} + \sin{z}\sin{y}\sin{x} &&
  \cos{x}\sin{z}\sin{y} - \cos{z}\sin{x} \\

  -\sin{y} &&
  \cos{y}\sin{x} &&
  \cos{y}\cos{x}
\end{matrix}
$$

which gives us: -->

$$
\begin{equation}
  \theta_{x} = \arctan(\frac{r_{32}}{r_{33}})
\end{equation}
$$

$$
\begin{equation}
  \theta_{y} = \arctan(\frac{-r_{31}}{\sqrt{r_{32}^2 + r_{33}^2}})
\end{equation}
$$

$$
\begin{equation}
  \theta_{z} = \arctan(\frac{r_{21}}{r_{11}})
\end{equation}
$$

1. **Data Alignment**: Align the IMU Time Stamps with the Vicon stamps. Here, Vicon Time Stamps are considered as true time stamps to which the IMU Data is aligned. Acceleration Data is interpolated using `Linear Interpolation (LERP)` and the Gyroscope Data is aligned using `Spherical Interpolation (SLERP)`

2. **Orientation Estimation**:
   - **Gyroscope-Based Orientation**: Compute orientation using only gyro data, assuming initial orientation from Vicon.

      This is done by modeling the gyroscope with noise and bias in the form:

$$
  \begin{equation}
    \omega = \hat{\omega} + b_{g} + n_{g}
  \end{equation}
$$

where,
- $\omega$ is the noise and bias affected gyroscope value we have from IMU
- $\hat{\omega}$ is the ideal value we want to read
- $b_{g}$ is a bias that is calculated using the first few 100 values when at rest
- $n_{g}$ is a modelled white gaussian noise

Using the ideal value we find at each timestamp, we do manual integration as follows:

```py
orientation[0] = viconInitialPose
for i in len(timeStampList): # Starting i from 1
  omegaHat = omega - bg- ng
  dt = timeStampList[i] - timeStampList[i-1]
  dTheta = omegaHat * dt
  orientation[i] = orientation[i-1] + dTheta
```

   - **Accelerometer-Based Orientation**: Estimate orientation using only accelerometer data, under the assumption that the IMU is only rotating.

      This is done by modeling the accelerometer with noise and bias in the form:

$$
  \begin{equation}
    a = (\hat{a}-g) + b_{a} + n_{a}
  \end{equation}
$$

where,
- $a$ is the noise and bias-affected accelerometer value we have from IMU
- $\hat{a}$ is the ideal value we want to read
- $b_{a}$ is a bias that is calculated using the first few 100 values when at rest
- $n_{a}$ is a modelled white gaussian noise

Using the ideal value of accelerations, we find the rotation at each step as follows:

$$
\begin{equation}
  \theta_{x} = \arctan(\frac{a_{y}}{\sqrt{{a_{x}}^{2} + {a_{z}}^{2}}})
\end{equation}
$$

$$
\begin{equation}
  \theta_{y} = \arctan(\frac{-a_{x}}{\sqrt{{a_{y}}^{2} + {a_{z}}^{2}}})
\end{equation}
$$

$$
\begin{equation}
  \theta_{z} = \arctan(\frac{\sqrt{{a_{x}}^{2} + {a_{y}}^{2}}}{a_{z}})
\end{equation}
$$

   - **Complementary Filter**: Fuse gyro and accelerometer data using a simple complementary filter using the fusion factor $\alpha$.

      This is implemented based on the accelerometer and gyroscope values we have. At each time stamp, a portion of the Accelerometer and Gyroscope is used to find the final state as follows:

$$
\begin{equation}
  \hat{x_{t}} = (1-\alpha)*x_{t,g} + \alpha*x_{t,a}
\end{equation}
$$

where,
- $\alpha$ is the fusion factor
- $\hat{x_{t}}$ is the filtered state
- $x_{t,g}$ is the state from gyroscope pose
- $x_{t,a}$ is the state from accelerometer pose

## Code Structure

The code is written in Python using a Python notebook. The Phase 1 implementation can be found in the root directory of the project.

`main.ipynb`: Script to run the orientation estimation algorithms.

## Discussion

While making the observations on the results I noticed a few things:
- Gyroscope based Attitude Estimation gives us accurate results when the motion of IMU is slow but fails under rapid motion.
- Accelerometer based Attitude Estimation gives us accurate results when the motion of IMU is rapid but fails under slower motions.
- Complimentary Filter based Attitude Estimation gives us better results as it is fuse of Accelerometer and Gyroscope to find a spot where the results might not be perfect but close to ground truth.
- IMU cannot measure yaw because Earth's gravity acts in the same direction as the Z-axis, which prevents the accelerometer from measuring rotation on the Z-axis.

## Results

### Combined Results

![s1](./Results/Phase%201/Set%201/Combined%20Graph.png)

![s2](./Results/Phase%201/Set%202/Combined%20Graph.png)

![s3](./Results/Phase%201/Set%203/Combined%20Graph.png)

![s4](./Results/Phase%201/Set%204/Combined%20Graph.png)

![s5](./Results/Phase%201/Set%205/Combined%20Graph.png)

![s6](./Results/Phase%201/Set%206/Combined%20Graph.png)


### Gyroscope-Based Orientation

![s1](./Results/Phase%201/Set%201/gyro.png)

![s2](./Results/Phase%201/Set%202/gyro.png)

![s3](./Results/Phase%201/Set%203/gyro.png)

![s4](./Results/Phase%201/Set%204/gyro.png)

![s5](./Results/Phase%201/Set%205/gyro.png)

![s6](./Results/Phase%201/Set%206/gyro.png)

### Accelerometer-Based Orientation

![s1](./Results/Phase%201/Set%201/accel.png)

![s2](./Results/Phase%201/Set%202/accel.png)

![s3](./Results/Phase%201/Set%203/accel.png)

![s4](./Results/Phase%201/Set%204/accel.png)

![s5](./Results/Phase%201/Set%205/accel.png)

![s6](./Results/Phase%201/Set%206/accel.png)

### Complementary Filter Orientation

![s1](./Results/Phase%201/Set%201/compli.png)

![s2](./Results/Phase%201/Set%202/compli.png)

![s3](./Results/Phase%201/Set%203/compli.png)

![s4](./Results/Phase%201/Set%204/compli.png)

![s5](./Results/Phase%201/Set%205/compli.png)

![s6](./Results/Phase%201/Set%206/compli.png)



<!-- # Phase 2 - Waypoint Navigation and AprilTag Landing

## Overview

The goal is to design and implement a navigation and landing system for a simulated quadrotor in an environment. The quadrotor is equipped with the ability to navigate to the provided 3D position waypoint. We are required to autonomously navigate through a sequence of predefined waypoints, perform AprilTag scanning after reaching every waypoint, and execute a landing maneuver on an AprilTag with the ID value of `4`

## Methodology

## Code Structure

## Discussion

## Results -->

# References

- Course Website: [RBE595 / CS549](https://rbe549.github.io/rbe595/fall2023/proj/p0/)
- IMU Sensor Data: [ArduIMU+ V2](www.sparkfun.com)
- Ground Truth Data: [Vicon Motion Capture System](www.vicon.com)

# Designer Details

- Designed for:
  - Worcester Polytechnic Institute
  - RBE595 - Hands-on Autonomous Aerial Vehicles
- Designed by:
  - [Parth Patel](mailto:parth.pmech@gmail.com)

# License

This project is licensed under [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) (see [LICENSE.md](LICENSE.md)).

Copyright 2024 Parth Patel

Licensed under the GNU General Public License, Version 3.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at

_https://www.gnu.org/licenses/gpl-3.0.en.html_

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.