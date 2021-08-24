#pragma once
#include "sl_sensor_vo/visual_odometry_frame.hpp"

#include <sl_sensor_slstudio/calibration_data.h>
#include <sl_sensor_slstudio/codec2p1tpu.h>
#include <sl_sensor_slstudio/image_undistorter.h>
#include <sl_sensor_slstudio/triangulator.h>
#include <sl_sensor_io/frame_timings_csv_writer.hpp>
#include <sl_sensor_io/numbered_frame_reader.hpp>
#include <sl_sensor_io/tum_csv_reader.hpp>
#include <sl_sensor_io/tum_csv_writer.hpp>
#include <sl_sensor_io/tum_pose.hpp>
#include <sl_sensor_timer/timer.hpp>

#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <mutex>
#include <thread>

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
  std::mutex mutex_;

  // Timer setup
  int timer_counter_ = 0;
  double total_processing_time_ = 0.0f;
  std::unordered_map<std::string, sl_sensor::timer::Timer> _timers = {
    { "Image Preprocessing", sl_sensor::timer::Timer("Image Preprocessing") },
    { "Image Registration", sl_sensor::timer::Timer("Image Registration") },
    { "Keypoint Detection", sl_sensor::timer::Timer("Keypoint Detection") },
    { "Keypoint Triangulation", sl_sensor::timer::Timer("Keypoint Triangulation") },
    { "Triangulation Filtering", sl_sensor::timer::Timer("Triangulation "
                                                         "Filtering") },
    { "Keypoint Description", sl_sensor::timer::Timer("Keypoint Description") },
    { "Correspondence Matching", sl_sensor::timer::Timer("Correspondence "
                                                         "Matching") },
    { "Correspondence Filtering 2D", sl_sensor::timer::Timer("Corresponde"
                                                             "nce "
                                                             "Filtering "
                                                             "2D") },
    { "Correspondence Filtering 3D", sl_sensor::timer::Timer("Correspon"
                                                             "dence "
                                                             "Filtering"
                                                             " 3D") },
    { "Pose Estimation", sl_sensor::timer::Timer("Pose Estimation") }
  };

  /**
   * @brief Perform the frame processing step
   *
   * @param frame - Output frame, if successful
   * @return true - Successful frame processing
   * @return false - Unsuccessful frame processing
   */
  bool FrameProcessing(VisualOdometryFrame &frame);

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
  bool FrameMatching(VisualOdometryFrame &prev_frame, VisualOdometryFrame &curr_frame,
                     Eigen::Matrix4f &output_transform, pcl::Correspondences &correspondences);

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
  void ImageCb(const sensor_msgs::ImageConstPtr &image_ptr);

  /**
   * @brief Callback when a projector time is received
   *
   * @param time_numbered_ptr
   */
  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr &time_numbered_ptr);

  /**
   * @brief Function that publishes a pose stamped message
   *
   * @param pose - pose to publish
   * @param timestamp
   */
  void PublishPoseStamped(const Eigen::Matrix4f &pose, const ros::Time &timestamp);

  /**
   * @brief Publish 3D points of correspondences used for pose estimation (currently only publishes max 10 points to
   * prevent lagging of pipeline)
   *
   * @param frame - Frame where wi
   * @param correspondences - Correspondences to publish
   * @param pose - Absolute pose of the frame
   * @param timestamp
   */
  void PublishCorrespondences(const VisualOdometryFrame &frame, const pcl::Correspondences &correspondences,
                              const Eigen::Matrix4f &pose, const ros::Time &timestamp);

  /**
   * @brief Visually display correspondences between two frames (Warning: this is a blocking function, only use for
   * input type is kNumberedFrames when inputs are not real-time)
   *
   * @param prev_frame
   * @param curr_frame
   * @param correspondences
   */
  void DisplayFeatureCorrespondences(const VisualOdometryFrame &prev_frame, const VisualOdometryFrame &curr_frame,
                                     const pcl::Correspondences &correspondences);

  /**
   * @brief Attempt to get an image sequence from subsriber inputs
   *
   * @param image_sequence - where image sequence will be written to
   * @param timestamp_vec - timestamps of the images in the sequence
   * @return true - successful
   * @return false - unsuccessful
   */
  bool GetNextImageSequenceFromSubscribers(std::vector<cv::Mat> &image_sequence, std::vector<ros::Time> &timestamp_vec);

  /**
   * @brief Get the Image From Image Buffer object
   *
   * @param target_time
   * @param image_ptr
   * @return true
   * @return false
   */
  bool GetImageFromImageBuffer(const ros::Time &target_time, sensor_msgs::ImageConstPtr &image_ptr);

  /**
   * @brief Remove all images in the buffer that has a smaller timestamp than target_time
   *
   * @param target_time
   */
  void ClearAllImagesFromBufferBeforeTiming(const ros::Time &target_time);

  /**
   * @brief Remove all projector timings in the buffer that has a smaller timestamp than target_time
   *
   * @param target_time
   */
  void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time &target_time);

  /**
   * @brief Get the Median value
   *
   * @tparam T - numerical data type
   * @param values - Vector of elements to find to median in
   * @return T - Median value
   */
  template <typename T>
  T GetMedian(const std::vector<T> &values)
  {
    auto values_sorted = values;
    int N = values.size();
    std::sort(values_sorted.begin(), values_sorted.end());
    return (N % 2 == 0) ? ((values_sorted.at(N / 2 - 1) + values_sorted.at(N / 2))) / 2.0f : (values_sorted.at(N / 2));
  }

  /**
   * @brief Perform phase correlation image registration on an image sequence
   *
   * @param image_sequence - Vector of images
   * @param reference_index - indice specifying which image in image_sequence is the reference image
   * @param shifts - Vector of 2D values indicating the required shifts
   */
  void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, int reference_index,
                                           std::vector<cv::Point2d> &shifts);

  /**
   * @brief Perform phase correlation image registration on an image sequence.
   * Image is subsampled by subsample_factor before registration is performed
   *
   * @param image_sequence - Vector of images
   * @param reference_index - indice specifying which image in image_sequence is the reference image
   * @param shifts - Vector of 2D values indicating the required shifts. Subsample factor has been accounted for (i.e.
   * shifts are for non-subsampled images)
   * @param subsample_factor - Factor to down scale images before registration. Set to a value <= 0.0 to disable
   * subsampling
   */
  void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, int reference_index,
                                           std::vector<cv::Point2d> &shifts, double subsample_factor);

  /**
   * @brief Apply shifts to image sequence
   *
   * @param input_image_sequence
   * @param output_image_sequence
   * @param shifts - Vector of 2D values indicating the amount to translate the images horizontally or vertically
   */
  void ApplyShiftsToImageSequence(std::vector<cv::Mat> &input_image_sequence,
                                  std::vector<cv::Mat> &output_image_sequence, const std::vector<cv::Point2d> &shifts);

  /**
   * @brief Convert rectangle values from MSER detector to a keypoint
   *
   * @param rect_vec - Vector of cv::Rect obtained from MSER detector
   * @param keypoint_vec - Resulting keypoints
   */
  void ConverMserRectsToKeypoints(const std::vector<cv::Rect> &rect_vec, std::vector<cv::KeyPoint> &keypoint_vec);

  /**
   * @brief Detect MSER keypoints
   *
   * @param mser_detector_ptr - Pointer to MSER detector
   * @param img - Image to detect MSER keypoints for
   * @param keypoint_vec - resulting MSER keypoints
   */
  void DetectMserKeypoints(cv::Ptr<cv::MSER> mser_detector_ptr, const cv::Mat &img,
                           std::vector<cv::KeyPoint> &keypoint_vec);

  /**
   * @brief Get the Descriptor Matches between two sets of and descriptors
   *
   * @param matcher_ptr - Pointer to the Descriptor matcher used
   * @param descriptors_static
   * @param descriptors_moving
   * @param good_matches - Vector of good matches
   * @param ratio_thresh - Lowe's ratio
   */
  void GetDescriptorMatches(cv::Ptr<cv::DescriptorMatcher> matcher_ptr, const cv::Mat &descriptors_static,
                            const cv::Mat &descriptors_moving, std::vector<cv::DMatch> &good_matches,
                            float ratio_thresh);

  /**
   * @brief Convert cv::DMatch -es to pcl::Correspondences
   *
   * @param dmatches
   * @param correspondences
   */
  void CvDmatchesToPclCorrespondences(const std::vector<cv::DMatch> &dmatches, pcl::Correspondences &correspondences);

  /**
   * @brief Convert pcl::Correspondences to cv::DMatch -es
   *
   * @param dmatches
   * @param correspondences
   */
  void PclCorrespondencesToCvDmatches(const pcl::Correspondences &correspondences, std::vector<cv::DMatch> &dmatches);

  /**
   * @brief Subsample an image sequence by subsample factor
   *
   * @param image_sequence_input
   * @param image_sequence_output
   * @param subsample_factor
   */
  void SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                              std::vector<cv::Mat> &image_sequence_output, double subsample_factor);

  /**
   * @brief Only matches that are symmetrical are kept in the output symmetric_matches
   *
   * @param matches12
   * @param matches21
   * @param symmetric_matches
   */
  void KeepOnlySymmetricMatches(const std::vector<cv::DMatch> &matches12, const std::vector<cv::DMatch> &matches21,
                                std::vector<cv::DMatch> &symmetric_matches);

  /**
   * @brief Median filtering over the 2D pixels shifts
   *
   * @param static_kps
   * @param moving_kps
   * @param kp_matches
   * @param threshold - in pixels, all points within +- threshold of the median are considered inliers
   */
  void FilterCorrespondencesByMedianVerticalPixelShift(const std::vector<cv::KeyPoint> &static_kps,
                                                       const std::vector<cv::KeyPoint> &moving_kps,
                                                       std::vector<cv::DMatch> &kp_matches, double threshold);

  /**
   * @brief Median filtering of 3D point distances, y distance, z distance and full l2 distance
   *
   * @param static_pc
   * @param moving_pc
   * @param correspondences
   * @param y_threshold
   * @param z_threshold
   * @param l2_threshold
   */
  void FilterCorrespondencesByMedian3dDistances(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                pcl::Correspondences &correspondences, double y_threshold,
                                                double z_threshold, double l2_threshold);
};

}  // namespace vo
}  // namespace sl_sensor