#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace visualise
{
/**
 * @brief Subscribes to a pose stamped topic and publishes a path
 * message that contains all previously received messages
 *
 */
class PoseStampedToPath
{
public:
  /**
   * @brief Construct a new Pose Stamped To Path object
   *
   * @param nh_public - Public node handle for Publisher and Subscriber
   * @param nh_private - Private node handle for getting topics addresses
   */
  PoseStampedToPath(ros::NodeHandle nh_public, ros::NodeHandle nh_private);

  /**
   * @brief Callback when a pose stamped message is received
   *
   * @param pose_stamped
   */
  void Callback(const geometry_msgs::PoseStamped& pose_stamped);

private:
  std::string subscribe_topic_ = "pose_stamped_in";
  std::string publish_topic_ = "path_out";
  ros::Publisher pub_;
  ros::Subscriber sub_;
  nav_msgs::Path path_;
};
}  // namespace visualise
}  // namespace sl_sensor