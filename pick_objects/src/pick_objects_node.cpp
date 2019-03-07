#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// MoveBaseGoal message with x and y as Robot positions and orientation about z, as Robot Yaw
move_base_msgs::MoveBaseGoal defineGoal(double positionX, double positionY, double rotationZ) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = positionX;
  goal.target_pose.pose.position.y = positionY;
  // based on a quarternion's property, z^2 + w^2 = 1.0 when x and y are both 0.0
  goal.target_pose.pose.orientation.z = rotationZ;
  goal.target_pose.pose.orientation.w = sqrt(1.0 - rotationZ * rotationZ);

  return goal;
}

bool sendGoalAndReach(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal, const char* goalInfo, const char* successMsg, const char* failMsg) {
   // Send the goal position and orientation for the robot to reach
  ROS_INFO(goalInfo);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  bool goalReached = (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  if(goalReached) {
    ROS_INFO(successMsg);
  } else {
    ROS_INFO(failMsg);
  }
  return goalReached;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
// Turtlebot moving towards Pickup Goal
  move_base_msgs::MoveBaseGoal pickupGoal = defineGoal(2.5, 4.5, 0.7);
  sendGoalAndReach(ac, pickupGoal, "Sending PickUp goal", "The turtlebot has reached the pickup location", "The turtlebot failed to reached the pickup location");

  // Sleep for 5 seconds
  ros::Duration(5.0).sleep();
  
// Turtlebot moving towards DropOff Goal
  move_base_msgs::MoveBaseGoal dropoffGoal = defineGoal(-10, -2, -0.7);
  sendGoalAndReach(ac, dropoffGoal, "Sending DropOff goal", "The turtlebot has reached the dropoff location", "The turtlebot failed to reached the dropoff location");

  // Sleep for 5 seconds
  ros::Duration(5.0).sleep();

  return 0;
}