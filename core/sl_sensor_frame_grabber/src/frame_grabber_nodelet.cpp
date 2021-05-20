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
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  // pub = private_nh.advertise<std_msgs::String>("out", 10);
  // sub = private_nh.subscribe("in", 10, &FrameGrabberNodelet::Callback, this);
}

void FrameGrabberNodelet::ImageCb(const sensor_msgs::ImageConstPtr& image_ptr)
{
  if (is_grabbing_)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      image_ptr_buffer_.push_back(image_ptr);
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

/**
void FrameGrabberNodelet::Callback(const sensor_msgs::ConstImgPtr& input)
{
  std_msgs::String output;
  output.data = input->data;
  pub.publish(output);
}
**/

}  // namespace frame_grabber
}  // namespace sl_sensor