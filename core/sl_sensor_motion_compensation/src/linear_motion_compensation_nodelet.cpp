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
  private_nh_.param<int>("camera_index", camera_index_, camera_index_);
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
  if (filter_id_.empty() || (!filter_id_.empty() && filter_id_ == image_arr_ptr->id))
  {
    bool proceed_with_alignment = true;

    // We expect an equal number of images for each camera, and the number of images per camera should be larger than
    // reference index (so index is not out of range). If these conditions are not met, we do not proceed with
    // processing
    bool equal_number_images_per_camera = (image_arr_ptr->data.size() % (size_t)image_arr_ptr->number_cameras) == 0;
    size_t images_per_camera = image_arr_ptr->data.size() / image_arr_ptr->number_cameras;

    if ((size_t)reference_index_ >= images_per_camera)
    {
      proceed_with_alignment = false;
      ROS_WARN("[LinearMotionCompensationNodelet] Provided reference indice is larger than the number of images for "
               "each camera, ignoring this image array message");
    }

    if (!equal_number_images_per_camera)
    {
      proceed_with_alignment = false;
      ROS_WARN("[LinearMotionCompensationNodelet] Unequal number of images from each camera, this is not expected, "
               "ignoring this image array message");
    }

    // Note 1: For the current implmentation, this nodelet only processes one set of camera images and outputs it (
    // images from other cameras will be ignored)
    if (proceed_with_alignment)
    {
      // Convert image msg to cv img
      std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
      image_acquisition::ConvertImgArrToCvPtrVec(image_arr_ptr, cv_img_ptr_vec);

      // Add to unaligned_images the camera images that we want to align and also keep track of the encoding
      std::vector<cv::Mat> unaligned_images;
      std::vector<std::string> encoding_vec;
      for (size_t i = camera_index_ * images_per_camera; i < camera_index_ * images_per_camera + images_per_camera; i++)
      {
        unaligned_images.emplace_back(cv_img_ptr_vec[i]->image);
        encoding_vec.push_back(cv_img_ptr_vec[i]->encoding);
      }

      // Align images
      std::vector<cv::Mat> aligned_images = {};
      PhaseCorrelateAlignImageSequence(unaligned_images, aligned_images, reference_index_, subsample_factor_,
                                       shifting_option_);

      // Publish aligned images

      // Note 2: We tag the output images with the timestamp of the reference image. This is important in matching the
      // acquisition time of the point cloud to its position it the trajectory
      // Note 3: As explained in Note 1, we are only dealing with image sequence from one camera only, hence the
      // number_cameras argument is set to 1
      PublishCvMatVec(output_pub_, aligned_images, image_arr_ptr->header.frame_id,
                      image_arr_ptr->data[reference_index_].header.stamp, ros::Time::now(), encoding_vec, 1,
                      image_arr_ptr->id);
    }
  }
}

}  // namespace motion_compensation
}  // namespace sl_sensor