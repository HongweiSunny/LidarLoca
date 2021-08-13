#! /bin/bash   
source ~/.bashrc
roscore 
cd ~/catkin_ws_my
source ./devel/setup.bash

roslaunch LidarLoca launch_lidar_loca01.launch 




