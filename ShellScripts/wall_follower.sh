#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/PlayArena.world"  &
sleep 5
xterm -e "roslaunch custom_gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun wall_follower wall_follower" 