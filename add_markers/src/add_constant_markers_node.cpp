#include <ros/ros.h>
#include "ConstantMarkerAdder.h"

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");

  const double pickupZone[3] = {2.5, 4.5, 0.7};
  const double dropoffZone[3] = {-10, -2, -0.7};
  ConstantMarkerAdder constantMarkerAdder(pickupZone, dropoffZone);
  constantMarkerAdder.displayMarkers();
  ros::spin();

  return 0;
}
