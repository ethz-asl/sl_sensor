#include "sl_sensor_motion_compensation/linear_motion_compensation_nodelet.hpp"

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
  private_nh_.param<std::string>("pattern_direction", pattern_direction_, pattern_direction_);
  private_nh_.param<int>("reference_index", reference_index_, reference_index_);
  private_nh_.param<double>("subsample_factor", subsample_factor_, subsample_factor_);
  private_nh_.param<std::string>("filter_id", filter_id_, filter_id_);

  // Note: We apply shifts only in the direction perpendicular to the projected pattern
  if (pattern_direction_ == "horizontal")
  {
    shifting_option_ = ShiftingOption::kVerticalShiftingOnly;
  }
  else if (pattern_direction_ == "vertical")
  {
    shifting_option_ = ShiftingOption::kHorizontalShiftingOnly;
  }
  else if (pattern_direction_ == "both")
  {
    shifting_option_ = ShiftingOption::kBothDirectionsShifting;
  }
  else
  {
    ROS_WARN("[LinearMotionCompensationNodelet] Invalid/No pattern direction provided! Defaulting "
             "to apply both horizontal and vertical shifting");
  }

  // Setup subscribers / publishers
  input_sub_ = nh_.subscribe(sub_topic_, 10, &LinearMotionCompensationNodelet::ImageArrayCb, this);
  output_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(pub_topic_, 10);
}

void LinearMotionCompensationNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr)
{
  // If a filter id is specified, make sure that the id from the message is correct before continuing with processing
  if (!filter_id_.empty() && filter_id_ == image_arr_ptr->id)
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
    PhaseCorrelateAlignImageSequence(unaligned_images, aligned_images, reference_index_, subsample_factor_,
                                     shifting_option_);

    // Publish aligned images
    std::vector<std::string> encoding_vec;

    for (const auto& data : image_arr_ptr->data)
    {
      encoding_vec.push_back(data.encoding);
    }

    // Note: We tag the output images with the timestamp of the reference image. This is important in matching the
    // acquisition time of the point cloud to its position it the trajectory
    PublishCvMatVec(output_pub_, aligned_images, image_arr_ptr->header.frame_id,
                    image_arr_ptr->data[reference_index_].header.stamp, ros::Time::now(), encoding_vec);
  }
}

}  // namespace motion_compensation
}  // namespace sl_sensor