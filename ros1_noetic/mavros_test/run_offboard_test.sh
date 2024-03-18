#!/bin/bash
#
# start mavros AND offboard_test nodes which runs the demo


source ros_environment.sh
roslaunch offboard_test offboard_test.launch flight_pattern:=1 flight_height:=1.00
