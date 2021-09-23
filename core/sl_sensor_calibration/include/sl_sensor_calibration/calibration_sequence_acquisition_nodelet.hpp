#pragma once

#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "sl_sensor_image_acquisition/ImageArray.h"

namespace sl_sensor {
namespace calibration {

/**
 * @brief Nodelet to save images required for calibration
 * For each camera, this includes:
 * 1) Shading (within cam folder)
 * 2) Mask (within cam folder)
 * 3) Decoded horizontal projector coordinates, up (within proj folder)
 * 4) Decoded vertical projector coordinates, vp (within proj folder)
 */
class CalibrationSequenceAcquisitionNodelet : public nodelet::Nodelet {
 public:
  CalibrationSequenceAcquisitionNodelet();
  ~CalibrationSequenceAcquisitionNodelet();

 private:
  ros::Subscriber image_array_sub_;
  std::vector<std::vector<ros::Publisher>> image_pubs_;
  ros::ServiceClient image_synchroniser_client_;
  ros::ServiceClient projector_client_;
  ros::ServiceServer erase_sequence_client_;
  ros::ServiceServer grab_next_sequence_client_;

  std::shared_ptr<std::thread> initialisation_thread_ptr_;

  std::string image_array_sub_topic_ = "/image_array_receive";
  std::string save_directory_ = "/tmp";
  std::string save_filename_ = "calibration_session";
  std::string erase_sequence_service_name_ = "erase_calibration_sequence";
  std::string grab_sequence_service_name_ = "grab_calibration_sequence";
  std::string image_synchroniser_service_name_ = "command_image_synchroniser";
  std::string projector_service_name_ = "command_projector";
  int checkerboard_num_rows_ = 10;
  int checkerboard_num_cols_ = 10;
  cv::Size checkerboard_size_;
  int number_cameras_ = 1;
  int counter_ = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::vector<cv::Mat> image_vec_buffer_;

  std::mutex mutex_;

  bool ready_ = false;

  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr);

  /**
   * @brief Converts input image that is in float format to an 8-bit image to be visualised. Float
   * values are normalised to a value between 0-255
   *
   * @param input_image - Input float image
   * @param output_image - Output 8-bit image
   */
  void ProcessFloatImage(const cv::Mat& input_image, cv::Mat& output_image);

  /**
   * @brief Create folders where calibration images will be saved
   *
   * @return true - Success
   * @return false - Failure
   */
  bool GenerateDataFolders();

  /**
   * @brief Initialise nodelet to be ready to take in new calibration sequences
   *
   */
  void Init();

  /**
   * @brief Send service request to image acquisition nodelet to project and grab next calibration
   * sequence
   *
   */
  void SendCommandForNextCalibrationSequence();

  /**
   * @brief Sends a service request to the projector
   *
   * @param command - Command to be sent to projector
   * @param pattern_no - Pattern indice
   */
  void SendProjectorCommand(const std::string& command, int pattern_no);

  /**
   * @brief Processes "Grab Next Sequence" Button Press from RQT GUI to start acquisition of next
   * calibration sequence
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool ProcessGrabNextSequenceCommand(std_srvs::Trigger::Request& req,
                                      std_srvs::Trigger::Response& res);

  /**
   * @brief Processes "Erase Last Sequence" Button Press from RQT GUI to erase the last calibration
   * sequence obtained that is stored in the buffer
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool ProcessEraseSequenceCommand(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res);

  /**
   * @brief Save calibration images to specified folders
   *
   * @param cv_img_vec - Vector of images to be saved in the order of 1) up 2) vp 3) mask 4) shading
   */
  void SaveData(const std::vector<cv::Mat>& cv_img_vec);
};

}  // namespace calibration
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::calibration::CalibrationSequenceAcquisitionNodelet,
                       nodelet::Nodelet);
