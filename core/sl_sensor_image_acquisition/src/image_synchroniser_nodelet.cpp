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
  std::string image_topics_full_string = {};
  private_nh_.param<std::string>("image_topics", image_topics_full_string, image_topics_full_string);
  auto image_topics = SplitString(image_topics_full_string, std::string(" "));

  for (const auto& topic : image_topics)
  {
    auto temp_grouper_ptr =
        std::make_unique<ImageGrouper>(topic, 1, image_trigger_period_, lower_bound_tol_, upper_bound_tol_);
    // We set images per group to 1 for now, will be set based on service request message
    image_grouper_ptrs_.push_back(std::move(temp_grouper_ptr));
  }

  for (const auto& grouper_ptr : image_grouper_ptrs_)
  {
    grouper_ptr->Init(nh_);
  }

  // Load Projector Config
  projector_config_ = YAML::LoadFile(projector_yaml_directory_);

  // Init Service Client to projector
  projector_client_ = nh_.serviceClient<sl_sensor_projector::CommandProjector>(projector_service_name_);
  SendProjectorCommand("black", 0);

  // Create thread for main loop
  main_loop_thread_ptr_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ImageSynchroniserNodelet::MainLoop, this)));
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
  boost::mutex::scoped_lock lock(mutex_);
  if (synchroniser_state_.is_running && synchroniser_state_.is_hardware_trigger)
  {
    projector_time_buffer_.push_back(time_numbered_ptr->time);
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
      // Update state based on request message
      synchroniser_state_.Reset();
      synchroniser_state_.is_running = true;
      synchroniser_state_.pattern_name = pattern_name;
      synchroniser_state_.is_hardware_trigger = req.is_hardware_trigger;
      synchroniser_state_.delay = ((double)req.delay_ms) / 1000.0f;
      synchroniser_state_.target_number_scans = req.number_scans;
      synchroniser_state_.number_images_per_scan =
          projector::Lightcrafter4500::GetNumberProjections(projector_config_, pattern_name);

      // Configure ImageGrabbers to start collecting images
      for (const auto& grouper_ptr : image_grouper_ptrs_)
      {
        grouper_ptr->SetImagesPerGroup(
            (synchroniser_state_.is_hardware_trigger) ? synchroniser_state_.number_images_per_scan : 1);
        grouper_ptr->Start();
      }

      // If hardware triggered, we configure projector now
      SendProjectorCommand(synchroniser_state_.pattern_name, -1);
    }
    else
    {
      ROS_INFO("[ImageSynchroniserNodelet] Cannot execute command, pattern does not exist");
    }
  }
  else if (command == "stop")
  {
    Cleanup();
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

bool ImageSynchroniserNodelet::ExecuteCommandHardwareTrigger()
{
  ros::Time successful_projector_time;
  bool success = false;
  std::vector<std::vector<sensor_msgs::ImageConstPtr>> nested_img_ptr_vec = {};

  // For each projector time in buffer
  for (const auto& projector_time : projector_time_buffer_)
  {
    nested_img_ptr_vec.clear();

    // For each image grouper
    for (const auto& grouper_ptr : image_grouper_ptrs_)
    {
      std::vector<sensor_msgs::ImageConstPtr> temp_vec = {};
      grouper_ptr->RetrieveImageGroup(projector_time, temp_vec, false);

      // If image grouper is successful, add to nested_img_ptr_vec
      if (!temp_vec.empty())
      {
        nested_img_ptr_vec.push_back(temp_vec);
      }
      else
      {
        // If an image grouper is unsucessful, we stop processing this projector time
        break;
      }
    }

    // If all image grouper is successful, we break the loop and start processing the results
    if ((int)nested_img_ptr_vec.size() == synchroniser_state_.number_images_per_scan)
    {
      success = true;
      successful_projector_time = projector_time;
      break;
    }
  }

  if (success)
  {  // Process and publish image array
    std::vector<sensor_msgs::ImageConstPtr> imgs_to_publish = {};
    MergeNestedVectors(nested_img_ptr_vec, imgs_to_publish);
    PublishImageArray(imgs_to_publish);

    // Clear projector timer buffer up till successful timer
    ClearAllProjectorTimingsFromBufferBeforeTiming(successful_projector_time);

    // Clear image buffers in image groupers
    for (int i = 0; i < (int)image_grouper_ptrs_.size(); i++)
    {
      image_grouper_ptrs_[i]->ClearAllImagesFromBufferBeforeTiming(nested_img_ptr_vec[i].back()->header.stamp);
    }
  }

  return success;
}

void ImageSynchroniserNodelet::ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time)
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

bool ImageSynchroniserNodelet::ExecuteCommandSoftwareTrigger()
{
  std::vector<std::vector<sensor_msgs::ImageConstPtr>> temp(image_grouper_ptrs_.size(),
                                                            std::vector<sensor_msgs::ImageConstPtr>());

  for (int i = 0; i < synchroniser_state_.number_images_per_scan; i++)
  {
    SendProjectorCommand(synchroniser_state_.pattern_name, i);

    // Sleep for exposure period and delay
    ros::Duration(synchroniser_state_.delay + image_trigger_period_).sleep();

    for (const auto& grouper_ptr : image_grouper_ptrs_)
    {
      auto img_ptr = grouper_ptr->GetLatestImage();

      if (img_ptr)
      {
        temp[i].push_back(img_ptr);
      }
      else
      {
        // If shared pointer is empty, this acquisition attempt has failed
        // Turn off projector and return false
        SendProjectorCommand("black", 0);
        return false;
      }
    }
  }

  // Merge all vectors together and publish
  std::vector<sensor_msgs::ImageConstPtr> imgs_to_publish = {};
  MergeNestedVectors(temp, imgs_to_publish);
  PublishImageArray(imgs_to_publish);

  return true;
}

void ImageSynchroniserNodelet::MainLoop()
{
  while (ros::ok())
  {
    bool is_running;
    {
      boost::mutex::scoped_lock lock(mutex_);
      is_running = synchroniser_state_.is_running;
    }

    if (is_running)
    {
      boost::mutex::scoped_lock lock(mutex_);

      // If running, we try to retrieve an image sequence
      bool success_this_iteration =
          (synchroniser_state_.is_hardware_trigger) ? ExecuteCommandHardwareTrigger() : ExecuteCommandSoftwareTrigger();

      // If successful, we update the number of scans retrieved
      if (success_this_iteration)
      {
        synchroniser_state_.current_number_scans++;

        // If we have met the target number, we reset cleanup
        if (synchroniser_state_.target_number_scans > 0 &&
            synchroniser_state_.current_number_scans >= synchroniser_state_.target_number_scans)
        {
          Cleanup();
        }
      }
    }
  }
}

void ImageSynchroniserNodelet::Cleanup()
{
  synchroniser_state_.Reset();

  // Configure ImageGrabbers to start collecting images
  for (const auto& grouper_ptr : image_grouper_ptrs_)
  {
    grouper_ptr->Stop();
  }

  SendProjectorCommand("black", 0);
}

std::vector<std::string> ImageSynchroniserNodelet::SplitString(const std::string& s, const std::string& delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

}  // namespace image_acquisition
}  // namespace sl_sensor