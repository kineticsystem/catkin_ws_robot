#!/bin/sh
xfce4-terminal -T "Gazebo" -e "gazebo" &
sleep 5
xfce4-terminal -T "ROSCore" -e "source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xfce4-terminal -T "RVIZ" -e "rosrun rviz rviz"
