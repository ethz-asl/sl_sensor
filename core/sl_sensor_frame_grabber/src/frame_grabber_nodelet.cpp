#include "sl_sensor_frame_grabber/frame_grabber_nodelet.hpp"

namespace sl_sensor
{
namespace frame_grabber
{
FrameGrabberNodelet::FrameGrabberNodelet()
{
}

void FrameGrabberNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<double>("lower_bound_tol", lower_bound_tol_, lower_bound_tol_);
  private_nh_.param<double>("upper_bound_tol", upper_bound_tol_, upper_bound_tol_);
  private_nh_.param<double>("image_trigger_period", image_trigger_period_, image_trigger_period_);

  private_nh_.param<std::string>("image_topic", image_sub_topic_, image_sub_topic_);
  private_nh_.param<std::string>("projector_topic", projector_timing_sub_topic_, projector_timing_sub_topic_);
  private_nh_.param<std::string>("frame_id", frame_id_, frame_id_);

  image_array_pub_ = nh_.advertise<sl_sensor_frame_grabber::ImageArray>(image_array_pub_topic_, 10);
  image_sub_ = nh_.subscribe(image_sub_topic_, 10, &FrameGrabberNodelet::ImageCb, this);
  projector_timing_sub_ = nh_.subscribe(projector_timing_sub_topic_, 10, &FrameGrabberNodelet::ProjectorTimeCb, this);
  grab_image_service_ = nh_.advertiseService("command_frame_grabber", &FrameGrabberNodelet::GrabImages, this);
}

void FrameGrabberNodelet::ImageCb(const sensor_msgs::ImageConstPtr& image_ptr)
{
  if (is_grabbing_)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      image_ptr_buffer_.emplace_back(image_ptr);
    }
  }
}

void FrameGrabberNodelet::ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr)
{
  if (is_grabbing_)
  {
    boost::mutex::scoped_lock lock(mutex_);
    projector_time_buffer_.push_back(time_numbered_ptr->time);
  }
}

bool FrameGrabberNodelet::GrabImages(sl_sensor_frame_grabber::GrabImages::Request& req,
                                     sl_sensor_frame_grabber::GrabImages::Response& res)
{
  std::string command(req.command);

  if (command == "start")
  {
    is_grabbing_ = true;

    double delay_s = ((double)req.delay_ms) / 1000.0f;
    ros::Duration delay(delay_s);
    start_time_ = ros::Time::now() + delay;

    number_images_ = req.number_images;
    target_number_frames_ = req.number_frames;
  }
  else if (command == "stop")
  {
    is_grabbing_ = false;
  }

  return true;
}

void FrameGrabberNodelet::ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time)
{
  int counter = 0;

  for (auto it = projector_time_buffer_.begin(); it != projector_time_buffer_.end(); it++)
  {
    // remove odd numbers
    if ((*it - target_time).toSec() <= 0.0f)
    {
      projector_time_buffer_.erase(it--);
      counter++;
    }
    else
    {
      break;
    }
  }

  // std::cout << counter << " projector timings deleted" << std::endl;
}

void FrameGrabberNodelet::ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time)
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

  // std::cout << counter << " images deleted" << std::endl;
}

void FrameGrabberNodelet::PublishImageArray(const std::vector<sensor_msgs::ImageConstPtr>& image_vec)
{
  sl_sensor_frame_grabber::ImageArray img_arr;

  img_arr.header.stamp = ros::Time::now();
  img_arr.header.frame_id = frame_id_;

  for (const auto img_ptr : image_vec)
  {
    auto& img = *img_ptr;
    img_arr.data.emplace_back(std::move(img));
  }

  image_array_pub_.publish(img_arr);
}

void FrameGrabberNodelet::Update()
{
  bool success = false;

  boost::mutex::scoped_lock lock(mutex_);

  // If not enough projector timings / images we do not attempt any matching
  if (projector_time_buffer_.empty() || (int)image_ptr_buffer_.size() < number_images_)
  {
    return;
  }

  std::vector<sensor_msgs::ImageConstPtr> temp_img_ptr_vec = {};
  ros::Time successful_projector_time;

  for (const auto& projector_time : projector_time_buffer_)
  {
    temp_img_ptr_vec.clear();

    // We check if we can retrieve all images that make up a frame for the current projector time
    for (int i = 0; i < number_images_; i++)
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
        break;
      }
    }

    // if successful (all image pointer for a frame in temp_img_ptr_vec)
    if ((int)temp_img_ptr_vec.size() == number_images_)
    {
      successful_projector_time = projector_time;
      success = true;
      // Exit while loop
      break;
    }
  }

  // Fill in ImageArray Message and publish
  if (success)
  {
    PublishImageArray(temp_img_ptr_vec);

    // Clear all images before and including this frame
    ClearAllImagesFromBufferBeforeTiming(temp_img_ptr_vec.back()->header.stamp);

    // Clear all projector times before and including this frame
    ClearAllProjectorTimingsFromBufferBeforeTiming(successful_projector_time);

    // Update state of frame grabber
    current_number_frames_++;

    if (target_number_frames_ > 0 && current_number_frames_ > target_number_frames_)
    {
      is_grabbing_ = false;
    }
  }
}

bool FrameGrabberNodelet::GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_it)
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

}  // namespace frame_grabber
}  // namespace sl_sensor