/**
 * MarkerAdder class function: 
 * 1. Publish marker in Rviz at designated pickup zone
 * 2. Subscribe to the odometry topic for current robot position w.r.t pickup zone, move Robot towards pickup zone and HIDE
 *    the marker with MarkerAdder once robot reaches pickup zone.
 * 3. Pause for five seconds
 * 4. Move Robot towards dropoff zone and once the robot reaches dropoff zone, SHOW the marker with MarkerAdder in the
 *    dropoff zone.
 */
#include "nav_msgs/Odometry.h"
#include "ConstantMarkerAdder.h"

class MarkerAdder : ConstantMarkerAdder {
public:
  MarkerAdder(const double* pickupZone, const double* dropoffZone);

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber odom_sub;
  bool pickedUp;

  constexpr static double THRESHOLD = 0.9;
};