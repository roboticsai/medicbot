#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/linux/medicbot/devel/setup.bash
export ROS_IP=192.168.1.68
export ROS_MASTER_URI=http://192.168.1.68:11311
roslaunch catkin_pkg master.launch
