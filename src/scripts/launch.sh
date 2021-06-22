#!/bin/bash

# Run Gazebo.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Gazebo" -e "
gazebo" &
sleep 5

# Run ROS core.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "ROS" -e "
source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5

# Run RVIZ node.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "RVIZ" -e "
rosrun rviz rviz"
