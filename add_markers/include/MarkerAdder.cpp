#include "MarkerAdder.h"
#include <cmath>
#include <sstream>

// Constructor
MarkerAdder::MarkerAdder(const double* pickupZone, const double* dropoffZone) : pickedUp(false), ConstantMarkerAdder(pickupZone, dropoffZone) {
  // Assign constants
  odom_sub = n.subscribe("/odom", 1, &MarkerAdder::odomCallback, this);

  setMarker(pickupZone[0], pickupZone[1], pickupZone[2]);
  showMarker(true);
}

// The odometry callback checks whether the robot is reaching the pickup zone or the dropoff zone, and
// show/hide the marker accordingly.

void MarkerAdder::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  double robot_XPos = msg->pose.pose.position.x;    //Robot X position
  double robot_YPos = msg->pose.pose.position.y;    //Robot Y position
  
  // Calculate Robot distance to pickup zone
  double pickupDistance = sqrt(pow(robot_XPos - pickupZone[0], 2.0) + pow(robot_YPos - pickupZone[1], 2.0));
  
  //Initially Robot pickedUp is set to false. It is set to true once the pickupDistance < THRESHOLD, hide the marker thereafter
  if (!pickedUp) {
    std::ostringstream pickupSs;
    pickupSs << "pickup distance " << pickupDistance;
    ROS_INFO(pickupSs.str().c_str());
    if (pickupDistance < THRESHOLD) {
      showMarker(false);
      ros::Duration(SLEEP_TIME).sleep();
      pickedUp = true;
    }
  }
  
// Calculate Robot distance to dropoff zone
  double dropoffDistance = sqrt(pow(robot_XPos - dropoffZone[0], 2.0) + pow(robot_YPos - dropoffZone[1], 2.0));
  
  // If pickedUp is set as true, Robot moves towards dropoff zone until dropoffDistance < THRESHOLD, show the Marker     thereafter
  if (pickedUp) {
    std::ostringstream dropoffSs;
    dropoffSs << "dropoff distance " << dropoffDistance;
    ROS_INFO(dropoffSs.str().c_str());
    if (dropoffDistance < THRESHOLD) {
      setMarker(dropoffZone[0], dropoffZone[1], dropoffZone[2]);
      showMarker(true);
    }
  }
}