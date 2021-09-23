
#include <ros/ros.h>
#include "sl_sensor_visualise/pose_stamped_to_path.hpp"

using namespace sl_sensor::visualise;

int main(int argc, char** argv) {
  // Init ros node
  ros::init(argc, argv, "pose_stamped_to_path");
  ros::NodeHandle nh_public;
  ros::NodeHandle nh_private("~");

  // Create Pose_stamped_to_path class object
  PoseStampedToPath pose_to_path(nh_public, nh_private);

  // Just keep spinning until termination
  ros::spin();

  return 0;
}