#!bin/bash -f

source /opt/ros/noetic/setup.bash
source /home/jetson/quadruped_sim/devel/setup.bash

rosnode kill -a
killall -9 rosmaster
killall gzserver
killall gzclient
