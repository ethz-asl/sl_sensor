
#include <ros/ros.h>
#include "sl_sensor_vo/structured_light_odometry.hpp"

int main(int argc, char** argv) {
  // Init ros node
  ros::init(argc, argv, "structured_light_odometry_node");
  ros::NodeHandle nh;

  // Create StructuredLightOdometry class object
  sl_sensor::vo::StructuredLightOdometry structured_light_odometry(nh);

  while (ros::ok()) {
    structured_light_odometry.LoopOnce();
    ros::spinOnce();
  }

  return 0;
}