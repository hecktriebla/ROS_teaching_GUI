#!/usr/bin/env bash
source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
export ROSCONSOLE_FORMAT='${message}'
roslaunch teachinggui teaching.launch