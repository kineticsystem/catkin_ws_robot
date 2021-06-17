#!/bin/sh
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Gazebo" -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Mapping" -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "RVIZ" -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Teleop" -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
