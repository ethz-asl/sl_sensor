#include <iostream>
#include "sl_sensor_image_acquisition/image_synchroniser_nodelet.hpp"

namespace sl_sensor
{
namespace image_acquisition
{
ImageSynchroniserNodelet::ImageSynchroniserNodelet()
{
}

void ImageSynchroniserNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<double>("lower_bound_tol", lower_bound_tol_, lower_bound_tol_);
  private_nh_.param<double>("upper_bound_tol", upper_bound_tol_, upper_bound_tol_);
  private_nh_.param<double>("image_trigger_period", image_trigger_period_, image_trigger_period_);

  private_nh_.param<std::string>("image_topic", image_sub_topic_, image_sub_topic_);
  private_nh_.param<std::string>("projector_topic", projector_timing_sub_topic_, projector_timing_sub_topic_);
  private_nh_.param<std::string>("output_topic", image_array_pub_topic_, image_array_pub_topic_);
  private_nh_.param<std::string>("frame_id", frame_id_, frame_id_);

  image_array_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(image_array_pub_topic_, 10);
  image_sub_ = nh_.subscribe(image_sub_topic_, 10, &ImageSynchroniserNodelet::ImageCb, this);
  projector_timing_sub_ =
      nh_.subscribe(projector_timing_sub_topic_, 10, &ImageSynchroniserNodelet::ProjectorTimeCb, this);
  grab_image_service_ = nh_.advertiseService("command_frame_grabber", &ImageSynchroniserNodelet::GrabImages, this);

  // Setup ImageGroupers

  // Load Projector Config
  proj_config_ = YAML::LoadFile(yaml_directory);
}

void ImageSynchroniserNodelet::ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr)
{
  if (is_running_)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_projector_time_ = *time_numbered_ptr;
  }
}

bool ImageSynchroniserNodelet::ProcessCommand(sl_sensor_image_acquisition::CommandImageSynchroniser::Request& req,
                                              sl_sensor_image_acquisition::CommandImageSynchroniser::Response& res)
{
  std::string command(req.command);

  if (command == "start")
  {
    is_running_ = true;

    double delay_s = ((double)req.delay_ms) / 1000.0f;
    ros::Duration delay(delay_s);
    start_time_ = ros::Time::now() + delay;

    target_number_scans_ = req.number_images;
    target_number_frames_ = req.number_frames;
  }
  else if (command == "stop")
  {
    is_running_ = false;
  }

  return true;
}

void ImageSynchroniserNodelet::PublishImageArray(const std::vector<sensor_msgs::ImageConstPtr>& image_vec)
{
  sl_sensor_image_acquisition::ImageArray img_arr;

  img_arr.header.stamp = ros::Time::now();
  img_arr.header.frame_id = frame_id_;

  for (const auto img_ptr : image_vec)
  {
    auto& img = *img_ptr;
    img_arr.data.emplace_back(std::move(img));
  }

  image_array_pub_.publish(img_arr);
}

}  // namespace image_acquisition
}  // namespace sl_sensor