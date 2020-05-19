#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/pi/medicbot/devel/setup.bash
export ROS_IP=192.168.1.67
export ROS_MASTER_URI=http://192.168.1.102:11311
roslaunch catkin_pkg diff_drive.launch
