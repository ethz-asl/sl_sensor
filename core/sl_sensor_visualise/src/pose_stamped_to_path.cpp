#include "sl_sensor_visualise/pose_stamped_to_path.hpp"

namespace sl_sensor {
namespace visualise {
PoseStampedToPath::PoseStampedToPath(ros::NodeHandle nh_public, ros::NodeHandle nh_private) {
  nh_private.param<std::string>("subscribe_topic", subscribe_topic_, subscribe_topic_);
  nh_private.param<std::string>("publish_topic", publish_topic_, publish_topic_);

  pub_ = nh_public.advertise<nav_msgs::Path>(publish_topic_, 5);
  sub_ = nh_public.subscribe(subscribe_topic_, 5, &PoseStampedToPath::Callback, this);
}

void PoseStampedToPath::Callback(const geometry_msgs::PoseStamped& pose_stamped) {
  path_.header = pose_stamped.header;
  path_.poses.push_back(pose_stamped);
  pub_.publish(path_);
}
}  // namespace visualise
}  // namespace sl_sensor