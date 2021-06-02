#include <iostream>
#include "sl_sensor_image_acquisition/image_synchroniser_nodelet.hpp"

#include <sl_sensor_projector/CommandProjector.h>
#include <sl_sensor_projector/lightcrafter_4500.hpp>

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

  private_nh_.param<std::string>("projector_topic", projector_timing_sub_topic_, projector_timing_sub_topic_);
  private_nh_.param<std::string>("output_topic", image_array_pub_topic_, image_array_pub_topic_);
  private_nh_.param<std::string>("frame_id", frame_id_, frame_id_);

  private_nh_.param<std::string>("projector_yaml_directory", projector_yaml_directory_, projector_yaml_directory_);

  image_array_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(image_array_pub_topic_, 10);
  projector_timing_sub_ =
      nh_.subscribe(projector_timing_sub_topic_, 10, &ImageSynchroniserNodelet::ProjectorTimeCb, this);
  synchroniser_service_ = nh_.advertiseService("command_image_synchroniser",
                                               &ImageSynchroniserNodelet::ProcessImageSynchroniserCommand, this);

  // Setup ImageGroupers

  // Load Projector Config
  projector_config_ = YAML::LoadFile(projector_yaml_directory_);

  // Init Service Client to projector
  projector_client_ = nh_.serviceClient<sl_sensor_projector::CommandProjector>(projector_service_name_);
  SendProjectorCommand("black", 0);
}

void ImageSynchroniserNodelet::SendProjectorCommand(const std::string& command, int pattern_no)
{
  sl_sensor_projector::CommandProjector srv;
  srv.request.command = command;
  srv.request.pattern_no = pattern_no;
  if (!projector_client_.call(srv))
  {
    ROS_INFO("[ImageSynchroniserNodelet] Projector command failed to execute");
  }
}

void ImageSynchroniserNodelet::ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr)
{
  if (synchroniser_state_.is_running)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_projector_time_ = time_numbered_ptr->time;
  }
}

bool ImageSynchroniserNodelet::ProcessImageSynchroniserCommand(
    sl_sensor_image_acquisition::CommandImageSynchroniser::Request& req,
    sl_sensor_image_acquisition::CommandImageSynchroniser::Response& res)
{
  std::string command(req.command);

  boost::mutex::scoped_lock lock(mutex_);

  if (command == "start")
  {
    std::string pattern_name(req.pattern_name);
    bool pattern_exists = projector::Lightcrafter4500::PatternExists(projector_config_, pattern_name);

    if (pattern_exists)
    {
      synchroniser_state_ = SynchroniserState();
      synchroniser_state_.is_running = true;
      synchroniser_state_.is_hardware_trigger = req.is_hardware_trigger;
      synchroniser_state_.delay = ((double)req.delay_ms) / 1000.0f;
      synchroniser_state_.target_number_scans = req.number_scans;
      synchroniser_state_.number_images_per_scan =
          projector::Lightcrafter4500::GetNumberProjections(projector_config_, pattern_name);
    }
    else
    {
      ROS_INFO("[ImageSynchroniserNodelet] Cannot execute command, pattern does not exist");
    }
  }
  else if (command == "stop")
  {
    synchroniser_state_ = SynchroniserState();
    synchroniser_state_.is_running = false;
    SendProjectorCommand("black", 0);
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

void ImageSynchroniserNodelet::ExecuteCommandHardwareTrigger(){};

void ImageSynchroniserNodelet::ExecuteCommandSoftwareTrigger(){};

}  // namespace image_acquisition
}  // namespace sl_sensor