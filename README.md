# PlanExWP6_IMU-EKF
Code produced as part of the Technical University Berlin Summer Semester 2020 Course "Planetary Exploration and Space Robotics"

# Usage
This code is intented to be run on Ubuntu.
The EKF code is located in PlanExWP6_IMU-EKF/rosws/src/wp6imufilter/src/kalman. In order to be able to run the simulationcontroller GUI, which served as validation and development tool,
you'll have to install ROS-melodic and the turtlebot3 packages via the APT of ubuntu. Then `catkin_make` inside 'rosws' and source the produced 
`./devel/setup`- file (.bash/.sh/.zsh based on the shell you are using). Also the python dependencies need to be installed via pip2: matplotlib, pandas, numpy. 
Finally check if the shebang-line inside the simulation controller points to your python2 installation.

To start the simulation controller GUI type `rosrun wp6imufilter simulationcontroller.py`. The GUI will then wait for a ROS-Gazebo instance. 
The example we used was the turtlebot3_burger, which can be opened by typing `roslaunch sr_ros_navi_tutorial turtlebot_sim.launch`.
