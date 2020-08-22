# PlanExWP6_IMU-EKF
Code produced as part of the Technical University Berlin Summer Semester 2020 Course "Planetary Exploration and Space Robotics"

# Extended Kalman Filter

### Description
Simulation with IMU Filter utilizing EKF and inertial sensor fusion.

### Requirements
Make sure following packages are installed:

~~~
sudo apt install ros-melodic-desktop-full
sudo apt-get install ros-melodic-gazebo-ros-pkgs
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-joint-state-publisher-gui
sudo apt-get install ros-melodic-hector-gazebo-pugins
sudo apt install ros-melodic-turtlebot3-description
sudo apt install ros-melodic-turtlebot3-gazebo
pip2 install matplotlib
pip2 install numpy
pip2 install pandas
~~~

### Execution steps
1) Clone Repository:
~~~
git clone git@github.com:WhatWheatleySaid/PlanExWP6_IMU-EKF.git
~~~

2) Navigate to ...repo/rosws
~~~
catkin_make
source devel/setup.bash
~~~

3) Start simulations
~~~
roscore
roslaunch sr_ros_navi_tutorial turtlebot_sim.launch
~~~
Delete model/ros_symbol to remove the arena.

4) Start Simulation controller
Navigate to repo/rosws/src/wp6imufilter/src/
~~~
python2 simulationcontroller.py
~~~
or type
~~~
rosrun wp6imufilter simulationcontroller.py
~~~
This assumes, that the shebang-line inside the `simulationcontroller.py` points towards your python2 installation.

5) Start Turtlebot rqt controller for setting forward velocity and turning rate
~~~
rqt
~~~
In the dropdown menus locate
~~~
Plugins/robot tools/robot steering
~~~

### Tuning EKF parameters
in file
~~~
repo/rosws/src/wp6imufilter/src/kalman/ori_estimation.py
~~~
change V and W multiplicators

### add Bias
in file
~~~
repo/rosws/src/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro
~~~
under plugins imu_sensor and magnetic_sensor, add offset,drift and/or Gaussian Noise.
