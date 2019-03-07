#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/PlayArena.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation_marker.launch" &
sleep 5
xterm -e "rosrun add_markers add_constant_markers_node"