#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <memory>

#include "sl_sensor_codec/codec.hpp"

namespace sl_sensor
{
namespace codec
{
/**
 * @brief Nodelet that decodes an image sequence with structured light patterns
 *
 */
class DecoderNodelet : public nodelet::Nodelet
{
public:
  DecoderNodelet();

private:
  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher decoded_pub_;
  ros::Subscriber image_array_sub_;

  std::string image_array_sub_topic_ = "/grouped_images_input";
  std::string decoded_pub_topic_ = "/decoded_images_output";
  std::string decoder_name_ = "";

  std::unique_ptr<Decoder> decoder_ptr_;

  int number_cameras_;
};

}  // namespace codec
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::codec::DecoderNodelet, nodelet::Nodelet);
