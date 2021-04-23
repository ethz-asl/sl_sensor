
#include <ros/ros.h>
#include "pc_registration_kitti/structured_light_odometry.hpp"

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "structured_light_odometry_node");
  ros::NodeHandle nh;

  // Create StructuredLightOdometry class object
  StructuredLightOdometry structured_light_odometry(nh);

  // Specify run freq
  double run_freq = 5;
  nh.param<double>("run_freq", run_freq, run_freq);
  // ros::Rate r(run_freq);

  while (ros::ok())
  {
    // std::cout << "Looping" << std::endl;
    structured_light_odometry.LoopOnce();
    ros::spinOnce();
    // r.sleep();
  }

  return 0;
}