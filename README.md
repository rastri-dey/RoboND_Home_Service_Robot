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
Use catkin to build the packages from source. From ``catkin_ws``, run:
``catkin_make``
``source devel/setup.bash``
to build the workspace packages and add them to the paths of ROS.

### Project Content
#### Directory Structure
The project consists of three packages with custom ROS node in the src directory:

**wall_follower:** uses a modified script provided by Udacity to instruct the robot to traverse around the room while following the walls using laser scan data.

**pick_objects:** is a node that publish static goals to the goal topic and thus directs the robot through the navigation packages.

**add_markers:** contains two nodes that publishes markers in RViz. The add_constant_markers_node publishes markers at the goal positions with constant time intervals, while the add_markers_node publishes the markers in response to the robot's movements in the environment. Note how the second node uses the MarkerAdder class that inherits from the CustomMarkerAdder class used in the first node!

**World** file includes a Play Arena model built in Gazebo's Building Editor, and a map produced from the robot performing SLAM in the house.
ShellScripts contains all the composite scripts to run each task of the project, and also supporting XML launch files amcl_demo.launch, gmapping_demo.launch and view_navigation.launch with customized parameters.

