#!/bin/bash


mkdir -p /workspace/team_code/catkin_ws/src
cd /workspace/team_code/catkin_ws/
/bin/bash -c 'source /opt/ros/noetic/setup.bash' && catkin_make -j4
cd /workspace