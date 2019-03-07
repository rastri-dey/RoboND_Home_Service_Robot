#include "ConstantMarkerAdder.h"
#include <cmath>

// Constructor
ConstantMarkerAdder::ConstantMarkerAdder(const double* pickupZone, const double* dropoffZone) : pickupZone(pickupZone), dropoffZone(dropoffZone) {
  // Assign constants
  MARKER_COLOR[0] = 0.0;
  MARKER_COLOR[1] = 0.0;
  MARKER_COLOR[2] = 1.0;
  MARKER_FRAME_ID = "map";
  MARKER_NS = "add_markers";  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  initMarker();

}

void ConstantMarkerAdder::displayMarkers() {
  // show the pickup marker
  setMarker(pickupZone[0], pickupZone[1], pickupZone[2]);
  showMarker(true);
  ROS_INFO("At pick up zone");
  
  // hide the pickup marker after five seconds
  ros::Duration(SLEEP_TIME).sleep();
  showMarker(false);
  ROS_INFO("In sleep mode");
  
  // show the dropoff marker after another five seconds
  ros::Duration(SLEEP_TIME).sleep();
  setMarker(dropoffZone[0], dropoffZone[1], dropoffZone[2]);
  showMarker(true);
  ROS_INFO("At drop off zone");
  
}
// Initialize the marker at the origin as a blue square marker
void ConstantMarkerAdder::initMarker() {
    marker.header.frame_id = MARKER_FRAME_ID;
    marker.header.stamp = ros::Time::now();
  
 // The namespace (ns) and id are used to create a unique name for the marker. If a marker message is received with the same ns and id, the new marker will replace the old one. 
    marker.ns = MARKER_NS;
    marker.id = 0;

    // Set the marker type: CUBE
    marker.type = MARKER_SHAPE;

    // Set the marker action:ADD
    marker.action = visualization_msgs::Marker::ADD;

    // Initialize pose of the marker. 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (in meters)
    marker.scale.x = MARKER_SIZE;
    marker.scale.y = MARKER_SIZE;
    marker.scale.z = MARKER_SIZE;

    // Set the color
    marker.color.r = MARKER_COLOR[0];
    marker.color.g = MARKER_COLOR[1];
    marker.color.b = MARKER_COLOR[2];
    marker.color.a = 0.0;

    marker.lifetime = ros::Duration();
}

// Create a marker with provided shape, pose and color information.
// Marker is for 2D poses, x and y are for position in respective axes, rotationZ is for rotation along z-axis.
void ConstantMarkerAdder::setMarker(double positionX, double positionY, double rotationZ) {
  marker.pose.position.x = positionX;
  marker.pose.position.y = positionY;
  marker.pose.orientation.z = rotationZ;
  marker.pose.orientation.w = sqrt(1.0 - rotationZ * rotationZ);
}

void ConstantMarkerAdder::showMarker(bool showing) {
  if (showing) {
    marker.color.a = 1.0;
  } else {
    marker.color.a = 0.0;  //Transparent
  }

  // publish the marker to its subscribers in rviz
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}