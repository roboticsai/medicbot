#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/odroid/medicbot/devel/setup.bash
export ROS_IP=192.168.1.102
export ROS_MASTER_URI=http://192.168.1.102:11311
roslaunch catkin_pkg catkin.launch
