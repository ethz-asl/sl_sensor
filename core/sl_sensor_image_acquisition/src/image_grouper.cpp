#include "sl_sensor_image_acquisition/image_grouper.hpp"

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
  image_sub_ = nh_.subscribe(image_sub_topic_, 10, &ImageGrouper::ImageCb, this);
};

void ImageGrouper::ImageCb(const sensor_msgs::ImageConstPtr& image_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (is_running_)
  {
    image_ptr_buffer_.emplace_back(image_ptr);
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
                                      std::vector<sensor_msgs::ImageConstPtr>& result_image_vec)
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
    auto target_time = projector_time + ros::Duration(i * image_trigger_period_);

    sensor_msgs::ImageConstPtr temp_ptr;

    bool image_acquisition_successful = GetImagePtrFromBuffer(target_time, temp_ptr);

    if (image_acquisition_successful)
    {
      // Successful frame acquisition, store the image pointer in temp_img_ptr_vec
      temp_img_ptr_vec.push_back(temp_ptr);
      continue;
    }
    else
    {
      // If not successful, we stop the looking for next image
      // std::cout << "Unsuccessful at the " << i + 1 << "th image" << std::endl;
    }
  }

  // if successful (all image pointer for a frame in temp_img_ptr_vec)
  if ((int)temp_img_ptr_vec.size() == number_images_per_group_)
  {
    success = true;
  }

  // If successful
  if (success)
  {
    // Fill in result image vector
    result_image_vec.clear();
    result_image_vec.swap(temp_img_ptr_vec);

    // Clear all images before and including this frame
    ClearAllImagesFromBufferBeforeTiming(temp_img_ptr_vec.back()->header.stamp);
  }

  return success;
}

void ImageGrouper::ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time)
{
  int counter = 0;

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

}  // namespace image_acquisition
}  // namespace sl_sensor