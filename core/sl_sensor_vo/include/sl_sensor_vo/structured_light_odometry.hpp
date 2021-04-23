#pragma once
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <memory>

#include <sl_sensor_io/tum_csv_reader.hpp>
#include <sl_sensor_io/tum_csv_writer.hpp>
#include <sl_sensor_timer/timer.hpp>
#include "sl_sensor_vo/visual_odometry_frame.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <sl_sensor_slstudio/calibration_data.h>
#include <sl_sensor_slstudio/codec2p1tpu.h>
#include <sl_sensor_slstudio/image_undistorter.h>
#include <sl_sensor_slstudio/triangulator.h>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include <cv_bridge/cv_bridge.h>

#include <unordered_map>

namespace sl_sensor
{
namespace vo
{
/**
 * @brief Class object that performs VO using the SL sensor
 *
 */
class StructuredLightOdometry
{
public:
  /**
   * @brief Construct a new Structured Light Odometry object
   *
   * @param nh - Node handle which the subscribers and publishers will be established with
   */
  StructuredLightOdometry(ros::NodeHandle nh);

  /**
   * @brief Update to be called as fast as possible, or at a desired frame rate
   *
   */
  void LoopOnce();

private:
  // ROS topic addresses
  std::string keypoints_topic_ = "/registered_pc";
  std::string pose_topic_ = "/registered_pose";
  std::string frame_name_ = "/map";
  std::string projector_trigger_topic_ = "";
  std::string image_topic_ = "";

  // ROS node, subscribers and publishers
  ros::NodeHandle nh_;
  std::unique_ptr<ros::AsyncSpinner> async_spinnter_ptr_;
  ros::Publisher pose_publisher_;
  ros::Publisher kp_publisher_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber projector_trigger_time_subscriber_;

  // 2D feature detection, description and matching
  cv::Ptr<cv::MSER> mser_detector_ptr_;
  cv::Ptr<cv::xfeatures2d::SURF> surf_descriptor_ptr_;
  cv::Ptr<cv::DescriptorMatcher> feature_matcher_ptr_;

  // Transformation Estimation SVD Setup
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac_rejector_;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float> transform_estimator_svd_;
  pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ, float>
      transform_estimator_dq_;
  pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, float> transform_estimator_2d_;

  // VO options
  bool received_first_frame_ = false;
  bool display_correspondences_ = false;
  bool publish_pose_stamped_ = false;
  bool publish_correspondences_ = false;
  double scaling_factor_ = 1.0;
  const int reference_image_indice_ = 1;
  int number_patterns_per_frame_;
  double image_tolerance_time_;
  double time_per_pattern_;
  bool write_output_relative_pose_csv_ = false;
  bool write_output_frame_timings_csv_ = false;
  std::string output_relative_pose_csv_filename_ = "";
  std::string output_frame_timings_csv_filename_ = "";

  // IO setup
  enum class InputType
  {
    kNumberedFrames,
    kRosSubscriber,
    kSize,
  };
  InputType input_type_ = InputType::kSize;
  YAML::Node config_;
  sl_sensor::io::NumberedFrameReader numbered_frame_reader_;
  std::unique_ptr<sl_sensor::io::TumCsvWriter> output_relative_pose_csv_writer_ptr_;
  std::unique_ptr<sl_sensor::io::FrameTimingsCsvWriter> output_frame_timings_csv_writer_ptr_;

  // Variables that store current state of VO
  std::unique_ptr<VisualOdometryFrame> prev_frame_ptr_;
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  std::vector<sensor_msgs::ImageConstPtr> image_ptr_buffer_ = {};
  std::vector<ros::Time> projector_time_buffer_ = {};

  // SLStudio utilities
  sl_sensor::slstudio::CalibrationData calibration_data_;
  std::unique_ptr<sl_sensor::slstudio::DecoderPhaseShift2p1Tpu> decoder_ptr_;
  std::unique_ptr<sl_sensor::slstudio::Triangulator> triangulator_ptr_;
  std::unique_ptr<sl_sensor::slstudio::ImageUndistorter> image_undistorter_ptr_;

  // mutex to prevent vo pipeline and subscribers to access the same variables at the same time
  boost::mutex mutex_;

  // Timer setup
  int timer_counter_ = 0;
  double total_processing_time_ = 0.0f;
  std::unordered_map<std::string, slsensor::timer::Timer> _timers = {
    { "Image Preprocessing", slsensor::timer::Timer("Image Preprocessing") },
    { "Image Registration", slsensor::timer::Timer("Image Registration") },
    { "Keypoint Detection", slsensor::timer::Timer("Keypoint Detection") },
    { "Keypoint Triangulation", slsensor::timer::Timer("Keypoint Triangulation") },
    { "Triangulation Filtering", slsensor::timer::Timer("Triangulation "
                                                        "Filtering") },
    { "Keypoint Description", slsensor::timer::Timer("Keypoint Description") },
    { "Correspondence Matching", slsensor::timer::Timer("Correspondence "
                                                        "Matching") },
    { "Correspondence Filtering 2D", slsensor::timer::Timer("Corresponde"
                                                            "nce "
                                                            "Filtering "
                                                            "2D") },
    { "Correspondence Filtering 3D", slsensor::timer::Timer("Correspon"
                                                            "dence "
                                                            "Filtering"
                                                            " 3D") },
    { "Pose Estimation", slsensor::timer::Timer("Pose Estimation") }
  };
};

/**
 * @brief Perform the frame processing step
 *
 * @param frame - Output frame, if successful
 * @return true - Successful frame processing
 * @return false - Unsuccessful frame processing
 */
bool FrameProcessing(VisualOdometryFrame& frame);

/**
 * @brief Perform the frame matching step
 *
 * @param prev_frame - Previous frame
 * @param curr_frame - Current frame
 * @param output_transform - Relative transform from previous frame to current frame
 * @param correspondences - Correspondences used to estimate output transforme
 * @return true - Successful
 * @return false - Unsuccessful
 */
bool FrameMatching(VisualOdometryFrame& prev_frame, VisualOdometryFrame& curr_frame, Eigen::Matrix4f& output_transform,
                   pcl::Correspondences& correspondences);

/**
 * @brief Initialise all functionality required for VO
 *
 */
void Init();

void InitRosSubscribers();
void InitRosPublishers();
void InitYaml();
void InitStructuredLight();
void InitOdometry();
void InitNumberedFrameReader();
void InitCsvOutput();

/**
 * @brief Callback when an image is received
 *
 * @param image_ptr
 */
void ImageCb(const sensor_msgs::ImageConstPtr& image_ptr);

/**
 * @brief Callback when a projector time is received
 *
 * @param time_numbered_ptr
 */
void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

/**
 * @brief Function that publishes a pose stamped message
 *
 * @param pose - pose to publish
 * @param timestamp
 */
void PublishPoseStamped(const Eigen::Matrix4f& pose, const ros::Time& timestamp);

/**
 * @brief Publish 3D points of correspondences used for pose estimation (currently only publishes max 10 points to
 * prevent lagging of pipeline)
 *
 * @param frame - Frame where wi
 * @param correspondences - Correspondences to publish
 * @param pose - Absolute pose of the frame
 * @param timestamp
 */
void PublishCorrespondences(const VisualOdometryFrame& frame, const pcl::Correspondences& correspondences,
                            const Eigen::Matrix4f& pose, const ros::Time& timestamp);

/**
 * @brief Visually display correspondences between two frames (Warning: this is a blocking function, only use for input
 * type is kNumberedFrames when inputs are not real-time)
 *
 * @param prev_frame
 * @param curr_frame
 * @param correspondences
 */
void DisplayFeatureCorrespondences(const VisualOdometryFrame& prev_frame, const VisualOdometryFrame& curr_frame,
                                   const pcl::Correspondences& correspondences);

/**
 * @brief Attempt to get an image sequence from subsriber inputs
 *
 * @param image_sequence - where image sequence will be written to
 * @param timestamp_vec - timestamps of the images in the sequence
 * @return true - successful
 * @return false - unsuccessful
 */
bool GetNextImageSequenceFromSubscribers(std::vector<cv::Mat>& image_sequence, std::vector<ros::Time>& timestamp_vec);

/**
 * @brief Get the Image From Image Buffer object
 *
 * @param target_time
 * @param image_ptr
 * @return true
 * @return false
 */
bool GetImageFromImageBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_ptr);

/**
 * @brief Remove all images in the buffer that has a smaller timestamp than target_time
 *
 * @param target_time
 */
void ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time);

/**
 * @brief Remove all projector timings in the buffer that has a smaller timestamp than target_time
 *
 * @param target_time
 */
void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time);

}  // namespace vo
}  // namespace sl_sensor