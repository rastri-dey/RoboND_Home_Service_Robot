#include <ros/ros.h>
#include "MarkerAdder.h"

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");

  double pickupZone[3] = {2.5, 4.5, 0.7};
  double dropoffZone[3] = {-10, -2, -0.7};
  MarkerAdder markerAdder(pickupZone, dropoffZone);
  ros::spin();

  return 0;
}
