# RoboND_Home_Service_Robot

This project is the final project of the Udacity Robotics Software Engineering Nanodegree Program. The project includes creation of several ROS packages supported by ROS libraries to simulate a home service robot for performing pick up and drop off operation in a built in Gazebo environment.

## Installation & Build
### ROS Kinetic
The project was developed on Ubuntu environment of Udacity Workspace with [ROS Kinetic](http://wiki.ros.org/kinetic), [Gazebo](http://gazebosim.org/) and [catkin](http://wiki.ros.org/catkin) installed.

### Supporting Packages

Following are the supporting packages to install for the project:

**gmapping:** With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.

**turtlebot_teleop:** With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.

**turtlebot_rviz_launchers:** With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you. 

**turtlebot_gazebo:** With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it. 

### rosdep Dependencies
After cloning the supporting package in ``catkin_ws/src``, run rosdep on each of the package names to install their dependencies.

``rosdep -i install gmapping``
``rosdep -i install turtlebot``

### Building the Workspace

``cd catkin_ws`` 
``catkin_make``
``source devel/setup.bash``

### Run Shell Scripts

``cd src/ShellScripts`` 
``./<ShellScriptsName.sh>``

ShellScriptsName: test_slam.sh, wall_follower.sh, test_navigation.sh, pick_objects.sh,add_marker.sh, home_service.sh

### Project Content

**wall_follower:** Allows the robot to traverse around the room following the walls with left hand side prefernce using laser scan data.

**pick_objects:** Publishes static goals to the goal topic and directs the robot through the ROS navigation stack.

**add_markers:** Publishes markers in RViz. 

**World** file includes a Play Arena model built in Gazebo's Building Editor, and a map in ``.pgm`` and ``.yaml`` format produced from the robot performing SLAM in the house.

### Results

**Gazebo World:** The Gazebo world built is inspired from a Play Arena format where the robot starts from a random position in the beginning and needs to come out of the maze look alike situation 
by navigating and planning path till the goal position.

The Gazebo world looks like:
![Gazebo world](Results\1_Gazebo_world_PlayArena.jpg)

wall_follower.sh constructs the following map of Gazebo world using a customized gmapping package:
![Rviz Map](Results\2_PlayArena_Map_built_by_Wall_followerScript.jpg)

Home Service Robot at pick up zone:
![Robot At Pick Up Zone](Results\3_Turtlebot_At_Pickup_Zone.png)

Home Service Robot at drop off zone:
![Robot At Drop Off Zone](Results\5_Turtlebot_At_DropOff_Zone.png)
