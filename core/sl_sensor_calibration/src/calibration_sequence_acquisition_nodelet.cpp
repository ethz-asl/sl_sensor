#include "sl_sensor_calibration/calibration_sequence_acquisition_nodelet.hpp"

#include <iostream>
#include <string>

#include <cstdint>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <experimental/filesystem>

#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <sl_sensor_image_acquisition/CommandImageSynchroniser.h>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

#include <sl_sensor_projector/CommandProjector.h>

namespace sl_sensor
{
namespace calibration
{
CalibrationSequenceAcquisitionNodelet::CalibrationSequenceAcquisitionNodelet()
{
}

void CalibrationSequenceAcquisitionNodelet::onInit()
{
  // Get Node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Get key information from ROS params
  nh_.param<std::string>("image_synchroniser_service_name", image_synchroniser_service_name_,
                         image_synchroniser_service_name_);
  nh_.param<std::string>("projector_service_name", projector_service_name_, projector_service_name_);
  nh_.param<std::string>("erase_sequence_service_name", erase_sequence_service_name_, erase_sequence_service_name_);
  nh_.param<std::string>("grab_sequence_service_name", grab_sequence_service_name_, grab_sequence_service_name_);

  private_nh_.param<std::string>("image_array_sub_topic", image_array_sub_topic_, image_array_sub_topic_);
  private_nh_.param<std::string>("save_directory", save_directory_, save_directory_);
  private_nh_.param<std::string>("save_filename", save_filename_, save_filename_);
  private_nh_.param<int>("checkerboard_num_rows", checkerboard_num_rows_, checkerboard_num_rows_);
  private_nh_.param<int>("checkerboard_num_cols", checkerboard_num_cols_, checkerboard_num_cols_);
  private_nh_.param<int>("number_cameras", number_cameras_, number_cameras_);

  checkerboard_size_ = cv::Size(checkerboard_num_cols_, checkerboard_num_rows_);

  // Setup subscribers
  image_array_sub_ =
      nh_.subscribe(image_array_sub_topic_, 10, &CalibrationSequenceAcquisitionNodelet::ImageArrayCb, this);

  // Setup publishers
  for (int i = 0; i < number_cameras_; i++)
  {
    std::string number = std::to_string(i + 1);

    image_pubs_.push_back(std::vector<ros::Publisher>{});

    std::string shading_topic_name = "calibration_data_acquisition_nodelet/cam" + number + "/shading";
    image_pubs_[i].push_back(nh_.advertise<sensor_msgs::Image>(shading_topic_name, 10));

    std::string mask_topic_name = "calibration_data_acquisition_nodelet/cam" + number + "/mask";
    image_pubs_[i].push_back(nh_.advertise<sensor_msgs::Image>(mask_topic_name, 10));

    std::string up_topic_name = "calibration_data_acquisition_nodelet/proj" + number + "/up";
    image_pubs_[i].push_back(nh_.advertise<sensor_msgs::Image>(up_topic_name, 10));

    std::string vp_topic_name = "calibration_data_acquisition_nodelet/proj" + number + "/vp";
    image_pubs_[i].push_back(nh_.advertise<sensor_msgs::Image>(vp_topic_name, 10));
  }

  // Set up service clients
  image_synchroniser_client_ =
      nh_.serviceClient<sl_sensor_image_acquisition::CommandImageSynchroniser>(image_synchroniser_service_name_);
  projector_client_ = nh_.serviceClient<sl_sensor_projector::CommandProjector>(projector_service_name_);

  // Set up service servers
  erase_sequence_client_ = nh_.advertiseService(
      erase_sequence_service_name_, &CalibrationSequenceAcquisitionNodelet::ProcessEraseSequenceCommand, this);
  grab_next_sequence_client_ = nh_.advertiseService(
      grab_sequence_service_name_, &CalibrationSequenceAcquisitionNodelet::ProcessGrabNextSequenceCommand, this);

  // Setup folders
  GenerateDataFolders();

  // Start thread to handle first command to image synchroniser
  initialisation_thread_ptr_ =
      std::shared_ptr<std::thread>(new std::thread(std::bind(&CalibrationSequenceAcquisitionNodelet::Init, this)));
}

void CalibrationSequenceAcquisitionNodelet::Init()
{
  // Sleep for a few seconds to make sure other nodelets are initialised properly
  ros::Duration(3.0).sleep();

  // Fully illuminate projector
  SendProjectorCommand("white", 0);

  // Set Nodelet to be ready receive image sequences
  ready_ = true;
}

void CalibrationSequenceAcquisitionNodelet::ImageArrayCb(
    const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr)
{
  std::scoped_lock lock(mutex_);

  // Check number of images are correct
  if ((int)(image_arr_ptr->data.size()) != (int)(number_cameras_ * 4))
  {
    ROS_INFO("[CalibrationSequenceAcquisitionNodelet] Number of images in image array does not correspond to ("
             "4 * number cameras). Ignoring this image array message");
    return;
  }

  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_const_ptr_vec_buffer;
  image_acquisition::ConvertImgArrToCvPtrVec(image_arr_ptr, cv_img_const_ptr_vec_buffer);

  image_vec_buffer_.clear();
  for (const auto& cv : cv_img_const_ptr_vec_buffer)
  {
    image_vec_buffer_.emplace_back(cv->image.clone());
  }

  // For each camera try to extract checkerboard from shading
  std::vector<bool> extracted_checkerboard_success_vec;
  std::vector<std::vector<cv::Point2f>> checkerboard_coordinates_vec;

  for (int i = 0; i < number_cameras_; i++)
  {
    std::vector<cv::Point2f> checkerboard_coordinates;
    bool success = cv::findChessboardCorners(image_vec_buffer_[i * 4 + 3], checkerboard_size_, checkerboard_coordinates,
                                             cv::CALIB_CB_ADAPTIVE_THRESH);

    if (!success)
    {
      std::string info_string =
          "[CalibrationSequenceAcquisitionNodelet] Failed to extract checkerboard from camera " + std::to_string(i + 1);
      ROS_INFO("%s", info_string.c_str());
    }

    extracted_checkerboard_success_vec.push_back(success);
    checkerboard_coordinates_vec.push_back(checkerboard_coordinates);
  }

  // Publish results for visualisation (Headers are set to empty)
  for (int i = 0; i < number_cameras_; i++)
  {
    // Publish shading for visualisation(with corner locations if they are detected)
    sensor_msgs::ImagePtr shading_display_ptr = boost::make_shared<sensor_msgs::Image>();
    cv::Mat shading_colour;
    cv::cvtColor(image_vec_buffer_[i * 4 + 3], shading_colour, cv::COLOR_GRAY2RGB);
    cv::drawChessboardCorners(shading_colour, checkerboard_size_, checkerboard_coordinates_vec[i],
                              extracted_checkerboard_success_vec[i]);
    cv_bridge::CvImage(std_msgs::Header(), "rgb8", shading_colour).toImageMsg(*shading_display_ptr);
    image_pubs_[i][0].publish(shading_display_ptr);

    // Publish mask for visualisation
    sensor_msgs::ImagePtr mask_display_ptr = boost::make_shared<sensor_msgs::Image>();
    cv_bridge::CvImage(std_msgs::Header(), "8UC1", image_vec_buffer_[i * 4 + 2]).toImageMsg(*mask_display_ptr);
    image_pubs_[i][1].publish(mask_display_ptr);

    // Publish decoded horizontal projector (up) coordinates (first convert float matrix to an 8 bit image for
    // visualisation)
    sensor_msgs::ImagePtr up_display_ptr = boost::make_shared<sensor_msgs::Image>();
    cv::Mat up_display;
    ProcessFloatImage(image_vec_buffer_[i * 4], up_display);
    cv_bridge::CvImage(std_msgs::Header(), "8UC1", up_display).toImageMsg(*up_display_ptr);
    image_pubs_[i][2].publish(up_display_ptr);

    // Publish decoded vertical projector (vp) coordinates (first convert float matrix to an 8 bit image for
    // visualisation)
    sensor_msgs::ImagePtr vp_display_ptr = boost::make_shared<sensor_msgs::Image>();
    cv::Mat vp_display;
    ProcessFloatImage(image_vec_buffer_[i * 4 + 1], vp_display);
    cv_bridge::CvImage(std_msgs::Header(), "8UC1", vp_display).toImageMsg(*vp_display_ptr);
    image_pubs_[i][3].publish(vp_display_ptr);
  }

  // Display status message
  std::string message = "[CalibrationSequenceAcquisitionNodelet] Checkerboard detection for this round: ";
  for (int i = 1; i <= number_cameras_; i++)
  {
    message += ("Cam" + std::to_string(i) + ": " + std::to_string(extracted_checkerboard_success_vec[i - 1]) + " ");
  }
  ROS_INFO("%s", message.c_str());

  // Fully illuminate projector
  SendProjectorCommand("white", 0);
}

bool CalibrationSequenceAcquisitionNodelet::GetInputAndCheckIfItIsExpectedChar(const std::string& message,
                                                                               char expected_char)
{
  std::string input;
  std::cout << message;
  std::getline(std::cin, input);
  return (!input.empty() && input.length() == 1 && (input.c_str())[0] == expected_char) ? true : false;
}

bool DirectoryExists(const char* path)
{
  struct stat info;

  if (stat(path, &info) != 0)
  {
    return false;
  }
  else if (info.st_mode & S_IFDIR)
  {
    return true;
  }
  else
  {
    return false;
  }
};

bool CalibrationSequenceAcquisitionNodelet::GenerateDataFolders()
{
  bool success = true;

  std::string full_file = save_directory_ + save_filename_;

  bool directory_exists = DirectoryExists(full_file.c_str());

  if (!directory_exists)
  {
    success = success && std::experimental::filesystem::create_directory(full_file);
  }

  if (success)
  {
    for (int i = 1; i <= number_cameras_; i++)
    {
      std::string number = std::to_string(i);
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "cam" + number);
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "cam" + number + "/" + "mask");
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "cam" + number + "/" + "shading");
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "proj" + number);
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "proj" + number + "/" + "up");
      success = success && std::experimental::filesystem::create_directory(save_directory_ + save_filename_ + "/" +
                                                                           "proj" + number + "/" + "vp");
    }
  }
  return success;
}

void CalibrationSequenceAcquisitionNodelet::ProcessFloatImage(const cv::Mat& input_image, cv::Mat& output_image)
{
  output_image = input_image.clone();
  cv::normalize(output_image, output_image, 0, 255, cv::NORM_MINMAX);
  output_image.convertTo(output_image, CV_8UC1);
}

void CalibrationSequenceAcquisitionNodelet::SendCommandForNextCalibrationSequence()
{
  std::scoped_lock lock(mutex_);

  // If buffer is not empty, we automatically save it before sending command
  if (!image_vec_buffer_.empty())
  {
    SaveData(image_vec_buffer_);
    image_vec_buffer_.clear();
  }

  // Send image synchroniser command to project calibration pattern
  sl_sensor_image_acquisition::CommandImageSynchroniser srv;

  srv.request.command = "start";
  srv.request.pattern_name = "calibration";
  srv.request.is_hardware_trigger = false;
  srv.request.number_scans = 1;
  srv.request.delay_ms =
      500;  // At least half a second of delay to take into account that the projector is in a transient state between
            // for some time between projecting nothing to an illuminated pattern

  if (!image_synchroniser_client_.call(srv))
  {
    ROS_INFO("[CalibrationSequenceAcquisitionNodelet] Projector command failed to execute");
  }
}

void CalibrationSequenceAcquisitionNodelet::SendProjectorCommand(const std::string& command, int pattern_no)
{
  sl_sensor_projector::CommandProjector srv;
  srv.request.command = command;
  srv.request.pattern_no = pattern_no;
  if (!projector_client_.call(srv))
  {
    ROS_INFO("[CalibrationSequenceAcquisitionNodelet] Projector command failed to execute");
  }
}

void CalibrationSequenceAcquisitionNodelet::SaveData(const std::vector<cv::Mat>& cv_img_ptr_vec)
{
  std::string status_message = "[CalibrationSequenceAcquisitionNodelet] Saving sequence " + std::to_string(counter_);
  ROS_INFO("%s", status_message.c_str());

  for (int i = 0; i < number_cameras_; i++)
  {
    std::string number = std::to_string(i + 1);
    std::string counter = std::to_string(counter_);

    // Save decoded horizontal projector coordinates (up) to xml file
    std::string up_directory =
        save_directory_ + save_filename_ + "/" + "proj" + number + "/" + "up" + "/" + counter + ".xml";
    cv::FileStorage up_file(up_directory, cv::FileStorage::WRITE);
    up_file << "up" << cv_img_ptr_vec[i * 4];

    // Save decoded vertical projector coordinates (vp) to xml file
    std::string vp_directory =
        save_directory_ + save_filename_ + "/" + "proj" + number + "/" + "vp" + "/" + counter + ".xml";
    cv::FileStorage vp_file(vp_directory, cv::FileStorage::WRITE);
    vp_file << "vp" << cv_img_ptr_vec[i * 4 + 1];

    // Save mask to bmp file
    std::string mask_directory =
        save_directory_ + save_filename_ + "/" + "cam" + number + "/" + "mask" + "/" + counter + ".bmp";
    cv::imwrite(mask_directory, cv_img_ptr_vec[i * 4 + 2]);

    // Save shading to bmp file
    std::string shading_directory =
        save_directory_ + save_filename_ + "/" + "cam" + number + "/" + "shading" + "/" + counter + ".bmp";
    cv::imwrite(shading_directory, cv_img_ptr_vec[i * 4 + 3]);
  }

  // Increment count for number of sequences saved
  counter_++;
}

bool CalibrationSequenceAcquisitionNodelet::ProcessGrabNextSequenceCommand(
    [[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (ready_)
  {
    SendCommandForNextCalibrationSequence();
  }

  res.success = ready_;

  return true;
}

bool CalibrationSequenceAcquisitionNodelet::ProcessEraseSequenceCommand(
    [[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)

{
  std::scoped_lock lock(mutex_);

  if (image_vec_buffer_.empty())
  {
    ROS_INFO("[CalibrationSequenceAcquisitionNodelet] There is no calibration sequence to be erased!");
  }
  else
  {
    ROS_INFO("[CalibrationSequenceAcquisitionNodelet] Erased most recent calibration sequence!");
    image_vec_buffer_.clear();
  }

  res.success = true;

  return true;
}

CalibrationSequenceAcquisitionNodelet::~CalibrationSequenceAcquisitionNodelet()
{
  std::scoped_lock lock(mutex_);

  // Save last calibration sequence before exiting
  if (!image_vec_buffer_.empty())
  {
    SaveData(image_vec_buffer_);
    image_vec_buffer_.clear();
  }
}

}  // namespace calibration
}  // namespace sl_sensor
