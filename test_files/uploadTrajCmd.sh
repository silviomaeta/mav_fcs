#!/bin/bash
cat "$1" | rostopic pub -l /fcs/trajectory_command small_copter_msgs/Trajectory
