# kubot_base_driver

[![Apache-2.0 License](https://img.shields.io/badge/license-Apache2.0-purple)](https://opensource.org/licenses/Apache-2.0)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
[![Platform Badge](https://img.shields.io/badge/platform-ROS_Melodic-blue.svg)](http://wiki.ros.org/melodic)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
[![version](https://img.shields.io/badge/version-1.0.1-green)](https://robot.shayangye.com/robots/59)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
[![robot](https://img.shields.io/badge/robot-KUBOT-orange)](http://www.shayangye.com/)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;

# Introduction

Only driver board control, and the basic motion control rostopic of the KOBOT Robot will be generated. More information please refer to [kubot wiki - base driver]().

# Repository Contents
- [x] kubot_bringup
	- [x] [bringup.launch](https://github.com/KUBOT-Robot/kubot_ros/wiki/2.1-kubot_bringup)
	- [x] [bringup_with_ekf.launch](https://github.com/KUBOT-Robot/kubot_ros/wiki/2.2-kubot_bringup_ekf)
	- [x] [robot.launch](https://github.com/KUBOT-Robot/kubot_ros/wiki/2.3-kubot_robot)
- [x] kubot_imu
- [x] kubot_msgs

# Install
This will download the package of kubot_base_driver and install it.

```sh
cd ~/kubot_ros/ros_ws/src
git clone https://github.com/KUBOT-Robot/kubot_base_driver.git -b melodic-devel
cd ..
catkin_make
source ~/.bashrc
```
