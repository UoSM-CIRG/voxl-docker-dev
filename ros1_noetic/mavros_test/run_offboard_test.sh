#!/bin/bash
#
# start mavros AND offboard_test nodes which runs the demo


source ros_environment.sh

# flight_pattern - 0 = Circular, 1 = Spiral, 2 = Cloud, 3 = Sine, 4 = N-gram
# max_iter - 2
# dt - 0.05
# radius - 1.00 m
# height - 1.00 m
# speed 
# min_speed
# offset_x - 0.00 m
# offset_y - 0.00 m
# offset_z - 0.50 m
# frequency
# ngram_vertices
# ngram_step
roslaunch offboard_test offboard_test.launch flight_pattern:=0 height:=1.00
