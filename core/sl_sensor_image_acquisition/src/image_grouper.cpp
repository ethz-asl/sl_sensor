#include "sl_sensor_image_acquisition/image_grouper.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace image_acquisition
{
ImageGrouper::ImageGrouper(std::string image_sub_topic, int number_images_per_group, double image_trigger_period,
                           double lower_bound_tol, double upper_bound_tol)
  : image_sub_topic_(image_sub_topic)
  , number_images_per_group_(number_images_per_group)
  , image_trigger_period_(image_trigger_period)
  , lower_bound_tol_(lower_bound_tol)
  , upper_bound_tol_(upper_bound_tol)
{
}

void ImageGrouper::Init(ros::NodeHandle nh)
{
  nh_ = nh;

  // Set up subscriber
  image_sub_ = nh_.subscribe(image_sub_topic_, 10, &ImageGrouper::ImageCb, this);
};

void ImageGrouper::ImageCb(const sensor_msgs::ImageConstPtr& image_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);

  // Add image to buffer only if Start() has been called
  if (is_running_)
  {
    image_ptr_buffer_.push_back(image_ptr);
  }
}

void ImageGrouper::Start()
{
  boost::mutex::scoped_lock lock(mutex_);
  is_running_ = true;
};

void ImageGrouper::Stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  is_running_ = false;

  // Clear image buffer
  image_ptr_buffer_.clear();
};

void ImageGrouper::SetToleranceBounds(double lower_tol, double upper_tol)
{
  lower_bound_tol_ = lower_tol;
  upper_bound_tol_ = upper_tol;
};

void ImageGrouper::SetImagesPerGroup(int number_images_per_group)
{
  number_images_per_group_ = number_images_per_group;
};

void ImageGrouper::SetImageTriggerPeriod(double trigger_period)
{
  image_trigger_period_ = trigger_period;
};

bool ImageGrouper::RetrieveImageGroup(const ros::Time& projector_time,
                                      std::vector<sensor_msgs::ImageConstPtr>& result_image_vec,
                                      bool clear_image_buffer_if_successful)
{
  bool success = false;

  boost::mutex::scoped_lock lock(mutex_);

  // If not enough projector timings / images we do not attempt any matching
  if (!is_running_ || (int)image_ptr_buffer_.size() < number_images_per_group_)
  {
    return false;
  }

  std::vector<sensor_msgs::ImageConstPtr> temp_img_ptr_vec = {};
  ros::Time successful_projector_time;

  temp_img_ptr_vec.clear();

  // We check if we can retrieve all images that make up a frame for the current projector time
  for (int i = 0; i < number_images_per_group_; i++)
  {
    // Computation of expected image time
    auto target_time = projector_time + ros::Duration((double)i * image_trigger_period_);

    sensor_msgs::ImageConstPtr temp_ptr;

    // Attempt to obtain an image with a timestamp close to target_time
    bool image_acquisition_successful = GetImagePtrFromBuffer(target_time, temp_ptr);

    if (image_acquisition_successful)
    {
      // Successful frame acquisition, store the image pointer in temp_img_ptr_vec
      temp_img_ptr_vec.push_back(temp_ptr);
    }
    else
    {
      // If not successful, we stop the looking for next image and break the for loop
      break;
    }
  }

  // Check success (all image pointer for a frame in temp_img_ptr_vec)
  if ((int)temp_img_ptr_vec.size() == number_images_per_group_)
  {
    success = true;
  }

  // If successful
  if (success)
  {
    // Fill in result image vector
    ros::Time last_image_time = temp_img_ptr_vec.back()->header.stamp;
    result_image_vec.clear();
    result_image_vec.swap(temp_img_ptr_vec);

    // Clear all images before and including this frame if specified
    if (clear_image_buffer_if_successful)
    {
      ClearAllImagesFromBufferBeforeTimingNoLock(last_image_time);
    }
  }

  return success;
}

void ImageGrouper::ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time)
{
  boost::mutex::scoped_lock lock(mutex_);
  ClearAllImagesFromBufferBeforeTimingNoLock(target_time);
}

void ImageGrouper::ClearAllImagesFromBufferBeforeTimingNoLock(const ros::Time& target_time)
{
  int counter = 0;

  // Iterate over image buffer, delete images before target_time
  for (auto it = image_ptr_buffer_.begin(); it != image_ptr_buffer_.end(); it++)
  {
    if (((*it)->header.stamp - target_time).toSec() <= 0.0f)
    {
      image_ptr_buffer_.erase(it--);
      counter++;
    }
    else
    {
      break;
    }
  }
}

bool ImageGrouper::GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_it)
{
  bool result = false;

  for (const auto img_ptr : image_ptr_buffer_)
  {
    double delta_t = (img_ptr->header.stamp - target_time).toSec();
    if (delta_t >= -1.0 * lower_bound_tol_ && delta_t <= upper_bound_tol_)
    {
      // If we find a image that satisfies the requirement, write the pointer to image_ptr and break the loop
      image_it = img_ptr;
      result = true;
      break;
    }
    else if (delta_t > upper_bound_tol_)
    {
      // If image is in the future of target time, we stop looking
      result = false;
      break;
    }
  }

  return result;
}

sensor_msgs::ImageConstPtr ImageGrouper::GetLatestImageAndClearBuffer()
{
  boost::mutex::scoped_lock lock(mutex_);

  // We make a copy of the latest image pointer (not just get the reference) in the buffer
  auto pointer = sensor_msgs::ImageConstPtr((image_ptr_buffer_.empty()) ? nullptr : image_ptr_buffer_.back());

  // Clear buffer
  ClearBuffer();

  return pointer;
}

void ImageGrouper::ClearBuffer()
{
  // Clear buffer
  image_ptr_buffer_.clear();
}

}  // namespace image_acquisition
}  // namespace sl_sensor