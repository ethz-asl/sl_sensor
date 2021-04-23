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
class StructuredLightOdometry
{
public:
  StructuredLightOdometry(ros::NodeHandle nh);
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
  ros::Publisher pose_publisher_;
  ros::Publisher kp_publisher_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber projector_trigger_time_subscriber_;

  // 2D feature detection, description and matchin
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

  NumberedFrameReader numbered_frame_reader_;

  double scaling_factor_ = 1.0;

  YAML::Node config_;

  const int reference_image_indice_ = 1;

  std::unique_ptr<single_frame> prev_frame_ptr_;

  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

  CalibrationData calibration_data_;
  std::unique_ptr<DecoderPhaseShift2p1Tpu> decoder_ptr_;
  std::unique_ptr<Triangulator> triangulator_ptr_;
  std::unique_ptr<Image_undistorter> image_undistorter_ptr_;

  bool GetLatestFrame(single_frame& frame);

  bool PerformOdometrySingleFrame(single_frame& prev_frame, single_frame& curr_frame, Eigen::Matrix4f& output_transform,
                                  pcl::Correspondences& correspondences);

  void Init();

  void InitRosSubscribers();

  void InitRosPublishers();

  void InitYaml();

  void InitStructuredLight();

  void InitOdometry();

  void InitNumberedFrameReader();

  void PublishPoseStamped(const Eigen::Matrix4f& pose, const ros::Time& timestamp);

  void PublishCorrespondences(const single_frame& frame, const pcl::Correspondences& correspondences,
                              const Eigen::Matrix4f& pose, const ros::Time& timestamp);

  void DisplayFeatureCorrespondences(const single_frame& prev_frame, const single_frame& curr_frame,
                                     const pcl::Correspondences& correspondences);

  enum class InputType
  {
    kNumberedFrames,
    kRosSubscriber,
    kSize,
  };

  InputType input_type_ = InputType::kSize;

  void ImageCb(const sensor_msgs::ImageConstPtr& image_ptr);

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  std::vector<sensor_msgs::ImageConstPtr> image_ptr_buffer_ = {};

  std::vector<ros::Time> projector_time_buffer_ = {};

  boost::mutex mutex_;  // Must use recursive mutex because we have some functions

  bool GetNextImageSequenceFromSubscribers(std::vector<cv::Mat>& image_sequence, std::vector<ros::Time>& timestamp_vec);

  bool GetImageFromImageBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_ptr);

  void ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time);

  void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time);

  int number_patterns_per_frame_;
  double image_tolerance_time_;
  double time_per_pattern_;

  int timer_counter_ = 0;
  double total_processing_time_ = 0.0f;

  std::unique_ptr<ros::AsyncSpinner> async_spinnter_ptr_;

  // int images_received_ = 0;

  int projector_timings_received_ = 0;

  std::string output_relative_pose_csv_filename_ = "";

  std::string output_frame_timings_csv_filename_ = "";

  void InitCsvOutput();

  bool write_output_relative_pose_csv_ = false;

  bool write_output_frame_timings_csv_ = false;

  std::unique_ptr<TumCsvWriter> output_relative_pose_csv_writer_ptr_;

  std::unique_ptr<FrameTimingsCsvWriter> output_frame_timings_csv_writer_ptr_;

  std::unordered_map<std::string, Timer> _timers = { { "Image Preprocessing", Timer("Image Preprocessing") },
                                                     { "Image Registration", Timer("Image Registration") },
                                                     { "Keypoint Detection", Timer("Keypoint Detection") },
                                                     { "Keypoint Triangulation", Timer("Keypoint Triangulation") },
                                                     { "Triangulation Filtering", Timer("Triangulation "
                                                                                        "Filtering") },
                                                     { "Keypoint Description", Timer("Keypoint Description") },
                                                     { "Correspondence Matching", Timer("Correspondence "
                                                                                        "Matching") },
                                                     { "Correspondence Filtering 2D", Timer("Corresponde"
                                                                                            "nce "
                                                                                            "Filtering "
                                                                                            "2D") },
                                                     { "Correspondence Filtering 3D", Timer("Correspon"
                                                                                            "dence "
                                                                                            "Filtering"
                                                                                            " 3D") },
                                                     { "Pose Estimation", Timer("Pose Estimation") } };
};

}  // namespace vo
}  // namespace sl_sensor