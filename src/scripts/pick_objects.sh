#!/bin/sh
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Gazebo" -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "AMCL" -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "RVIZ" -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Pick Objects" -e "roslaunch pick_objects pick_objects.launch" &
