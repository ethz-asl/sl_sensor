#include "sl_sensor_motion_compensation/linear_motion_compensation_nodelet.hpp"

#include "sl_sensor_motion_compensation/phase_correlation_utils.hpp"

#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

using namespace sl_sensor::image_acquisition;

namespace sl_sensor
{
namespace motion_compensation
{
LinearMotionCompensationNodelet::LinearMotionCompensationNodelet()
{
}

void LinearMotionCompensationNodelet::onInit()
{
  // Get Node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Get key information from ROS params
  private_nh_.param<std::string>("input_topic", sub_topic_, sub_topic_);
  private_nh_.param<std::string>("output_topic", pub_topic_, pub_topic_);
  private_nh_.param<int>("reference_indice", reference_indice_, reference_indice_);
  private_nh_.param<double>("subsample_factor", subsample_factor_, subsample_factor_);

  // Setup subscribers / publishers
  input_sub_ = nh_.subscribe(sub_topic_, 10, &LinearMotionCompensationNodelet::ImageArrayCb, this);
  output_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(pub_topic_, 10);
}

void LinearMotionCompensationNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr)
{
  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
  image_acquisition::ConvertImgArrToCvPtrVec(image_arr_ptr, cv_img_ptr_vec);
  std::vector<cv::Mat> unaligned_images = {};
  for (const auto& img_ptr : cv_img_ptr_vec)
  {
    unaligned_images.emplace_back(img_ptr->image);
  }

  // Align images
  std::vector<cv::Mat> aligned_images = {};
  PhaseCorrelateAlignImageSequence(unaligned_images, aligned_images, reference_indice_, subsample_factor_);

  // Publish aligned images
  std::vector<std::string> encoding_vec;

  for (const auto& data : image_arr_ptr->data)
  {
    encoding_vec.push_back(data.encoding);
  }

  PublishCvMatVec(output_pub_, aligned_images, image_arr_ptr->header.frame_id, image_arr_ptr->header.stamp,
                  ros::Time::now(), encoding_vec);
}

}  // namespace motion_compensation
}  // namespace sl_sensor