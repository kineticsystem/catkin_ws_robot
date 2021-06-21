#!/bin/sh

export BASE_DIR="$(cd "$(dirname "$1")/.." && pwd)/"

# Run Gazebo.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "Gazebo" -e "
cd $BASE_DIR;
source devel/setup.bash;
export GAZEBO_RESOURCE_PATH='$GAZEBO_RESOURCE_PATH:$BASE_DIR/src';
export ROBOT_INITIAL_POSE='-x -0.015210 -y -1.325962 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$BASE_DIR/src/map/myworld.world" &
sleep 5

# Run the AMCL node.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "AMCL" -e "
cd $BASE_DIR;
source devel/setup.bash; 
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$BASE_DIR/src/map/myworld.yaml" &
sleep 5

#Run the RVIZ node.
xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "RVIZ" -e "
cd $BASE_DIR;
source devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
