#!/bin/bash

export BASE_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:$BASE_DIR/src";
export ROBOT_INITIAL_POSE="-x -1.325962 -y 1.325962 -z 0 -R 0 -P 0 -Y 0";

cd $BASE_DIR
source devel/setup.bash

# Run Gazebo.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Gazebo" -e "
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$BASE_DIR/src/map/myworld.world" &
sleep 5

# Run the AMCL node.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "AMCL" -e "
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$BASE_DIR/src/map/myworld.yaml initial_pose_x:=-1.325962 initial_pose_y:=1.325962 initial_pose_a:=0" &
sleep 5

# Run RVIZ node.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "RVIZ" -e "
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
