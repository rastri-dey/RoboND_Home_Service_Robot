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
rosdep -i install gmapping
rosdep -i install turtlebot

### Building the Workspace
Use catkin to build the packages from source. From catkin_ws, run:
catkin_make; source devel/setup.bash
to build the workspace packages and add them to the paths of ROS.

