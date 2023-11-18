#!/bin/bash
#
# builds everything without installing
#
# Modal AI Inc. 2019
# author: james@modalai.com

set -e

cd catkin_ws

# load main ros environment
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]; then
    source /opt/ros/kinetic/setup.bash
elif [ -f /opt/ros/indigo/setup.bash ]; then
    source /opt/ros/indigo/setup.bash
fi

catkin_make

catkin_make install