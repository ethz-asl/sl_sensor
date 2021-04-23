#include "sl_sensor_vo/structured_light_odometry.hpp"

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <sl_sensor_conversions/conversions.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>

#include <cstdint>

namespace sl_sensor
{
namespace vo
{
StructuredLightOdometry::StructuredLightOdometry(ros::NodeHandle nh) : nh_(nh)
{
  Init();
}

void StructuredLightOdometry::LoopOnce()
{
  // Timer setup
  std::chrono::duration<double> elapsed;
  auto start = std::chrono::high_resolution_clock::now();

  auto curr_frame_ptr = std::make_unique<VisualOdometryFrame>();
  bool successful_frame_acquisition = FrameProcessing(*curr_frame_ptr);

  // If successful frame acquisition
  if (successful_frame_acquisition)
  {
    pcl::Correspondences correspondences;
    Eigen::Matrix4f relative_pose = Eigen::Matrix4f::Identity();
    bool successful_odometry = false;
    ros::Time curr_frame_ros_time;
    curr_frame_ros_time.fromNSec(curr_frame_ptr->reference_image_timestamp_nsec);

    // If we already received the first frame, we perform odometry
    if (received_first_frame_)
    {
      // If latest frame acquired successfully, we attempt to perform odometry
      successful_odometry = FrameMatching(*prev_frame_ptr_, *curr_frame_ptr, relative_pose, correspondences);

      // Display correspondences between current and previous frame, if desired
      if (display_correspondences_)
      {
        DisplayFeatureCorrespondences(*prev_frame_ptr_, *curr_frame_ptr, correspondences);
      }
    }

    // If successful pose estimate
    if (successful_odometry)
    {
      // std::cout << "Successful odometry" << std::endl;

      // We compute and print the processing time (Frame processing + Odometry)
      auto finish = std::chrono::high_resolution_clock::now();
      elapsed = finish - start;
      total_processing_time_ += elapsed.count();
      timer_counter_++;

      std::cout << "Mean frame processing time for " << timer_counter_
                << " frames [s]: " << total_processing_time_ / (double)timer_counter_ << std::endl;

      // Update current pose
      current_pose_ = current_pose_ * relative_pose;

      // std::cout << "Number correspondences: " << correspondences.size() << std::endl;
      /**
      std::cout << "Relative Pose: " << std::endl;
      std::cout << relative_pose << std::endl;
      **/

      // std::cout << "Current Pose: " << std::endl;
      // std::cout << current_pose_ << std::endl;

      // Publish correspondences for the current frame if desired
      if (publish_correspondences_)
      {
        PublishCorrespondences(*curr_frame_ptr, correspondences, current_pose_, curr_frame_ros_time);
      }
    }

    // Publish results from this iteration if desired
    if (publish_pose_stamped_)
    {
      PublishPoseStamped(current_pose_, curr_frame_ros_time);
    }

    // Write relative pose to csv this iteration (First frame will have identity transformation matrix)
    if (write_output_relative_pose_csv_)
    {
      output_relative_pose_csv_writer_ptr_->WriteNextRow(
          sl_sensor::io::TumPose(curr_frame_ptr->reference_image_timestamp_nsec, relative_pose));
    }

    // Write all timestamps for all images required for this frame to csv
    if (write_output_frame_timings_csv_)
    {
      output_frame_timings_csv_writer_ptr_->WriteNextRow(curr_frame_ptr->timestamps_nsec);
    }

    // If this is the first frame, update that the first frame has been received
    if (!received_first_frame_)
    {
      received_first_frame_ = true;
    }

    // Finally set the current frame to the previous frame for the next iteration
    prev_frame_ptr_.reset();
    prev_frame_ptr_ = std::move(curr_frame_ptr);
  }
}

bool StructuredLightOdometry::FrameProcessing(VisualOdometryFrame &frame)
{
  // Try to get images, if unsuccessful, stop and return false
  bool successful_image_acquisition = false;

  std::vector<ros::Time> timestamp_vec = {};

  if (input_type_ == InputType::kNumberedFrames)
  {
    successful_image_acquisition = numbered_frame_reader_.GetNextImageSequence(frame.image_sequence);
    if (successful_image_acquisition)
    {
      auto current_ros_time = ros::Time::now();
      frame.reference_image_timestamp_nsec = current_ros_time.toNSec();
    }
  }
  else if (input_type_ == InputType::kRosSubscriber)
  {
    successful_image_acquisition = GetNextImageSequenceFromSubscribers(frame.image_sequence, timestamp_vec);

    if (successful_image_acquisition)
    {
      frame.reference_image_timestamp_nsec = timestamp_vec[reference_image_indice_].toNSec();
      frame.timestamps_nsec.clear();
      for (const auto &timestamp : timestamp_vec)
      {
        frame.timestamps_nsec.push_back(timestamp.toNSec());
      }
    }
  }

  else
  {
    std::cout << "Invalid input format!" << std::endl;
    throw;
  }

  // std::cout << "Image acquisition success: " << successful_image_acquisition << std::endl;

  if (!successful_image_acquisition)
  {
    return false;
  }

  // Proceed to process frame, populate required information

  // Rectify Image Sequence
  _timers["Image Preprocessing"].Start();
  image_undistorter_ptr_->UndistortImageSequence(frame.image_sequence, frame.rectified_image_sequence);

  // Create reference image
  frame.rectified_image_sequence[reference_image_indice_].convertTo(frame.reference_image_8uc, CV_8UC1, 255.0f);
  _timers["Image Preprocessing"].End();

  // Compute transformation between images using phase correlation
  _timers["Image Registration"].Start();
  PhaseCorrelateRegisterImageSequence(frame.rectified_image_sequence, reference_image_indice_, frame.shifts,
                                      config_["imageRegistration"]["subsampleFactor"].as<double>());

  // Perform image translations
  ApplyShiftsToImageSequence(frame.rectified_image_sequence, frame.registered_image_sequence, frame.shifts);
  _timers["Image Registration"].End();

  // Identify MSER keypoints
  _timers["Keypoint Detection"].Start();
  DetectMserKeypoints(mser_detector_ptr_, frame.reference_image_8uc, frame.kps);
  _timers["Keypoint Detection"].End();

  // Decoding Phase
  _timers["Keypoint Triangulation"].Start();
  decoder_ptr_->decode_keypoints(frame.registered_image_sequence, frame.kps, frame.phases);

  // Triangulate keypoints using phase
  triangulator_ptr_->triangulate_vp_from_keypoints(frame.kps, frame.phases, *frame.kps_pc_ptr);

  if (scaling_factor_ != 1.0f)
  {  // Apply scaling factor to point cloud
    Eigen::Matrix4f scaling_transform = Eigen::Matrix4f::Identity();
    scaling_transform(0, 0) = scaling_transform(0, 0) * scaling_factor_;
    scaling_transform(1, 1) = scaling_transform(1, 1) * scaling_factor_;
    scaling_transform(2, 2) = scaling_transform(2, 2) * scaling_factor_;
    pcl::transformPointCloud(*(frame.kps_pc_ptr), *(frame.kps_pc_ptr), scaling_transform);
  }
  _timers["Keypoint Triangulation"].End();

  // Remove any 2D and 3D points in the current frame that have unreliable values in the 3D coordinates (nan entries
  // or outside of bounding box)
  _timers["Triangulation Filtering"].Start();
  frame.RemoverUnreliableDepthKeypoints(
      config_["pcFilter"]["boundingBox"]["xMin"].as<double>(), config_["pcFilter"]["boundingBox"]["xMax"].as<double>(),
      config_["pcFilter"]["boundingBox"]["yMin"].as<double>(), config_["pcFilter"]["boundingBox"]["yMax"].as<double>(),
      config_["pcFilter"]["boundingBox"]["zMin"].as<double>(), config_["pcFilter"]["boundingBox"]["zMax"].as<double>());
  _timers["Triangulation Filtering"].End();

  // Generate SURF descriptors for keypoints with valid depth information
  _timers["Keypoint Description"].Start();
  surf_descriptor_ptr_->compute(frame.reference_image_8uc, frame.kps, frame.descriptors);
  _timers["Keypoint Description"].End();

  return true;
}

bool StructuredLightOdometry::FrameMatching(VisualOdometryFrame &prev_frame, VisualOdometryFrame &curr_frame,
                                            Eigen::Matrix4f &output_transform, pcl::Correspondences &correspondences)
{
  // Clear correspondences just in case it is not empty
  correspondences.clear();

  // Kp descriptor matching
  std::vector<cv::DMatch> kp_matches = {};

  _timers["Correspondence Matching"].Start();
  double ratio_thresh = config_["featureMatcher"]["ratioThreshold"].as<float>();
  if (config_["featureMatcher"]["keepOnlySymmetricMatches"].as<bool>())
  {
    std::vector<cv::DMatch> matches_prev_curr, matches_curr_prev;

    GetDescriptorMatches(feature_matcher_ptr_, prev_frame.descriptors, curr_frame.descriptors, matches_prev_curr,
                         ratio_thresh);
    GetDescriptorMatches(feature_matcher_ptr_, curr_frame.descriptors, prev_frame.descriptors, matches_curr_prev,
                         ratio_thresh);
    KeepOnlySymmetricMatches(matches_prev_curr, matches_curr_prev, kp_matches);
  }
  else
  {
    GetDescriptorMatches(feature_matcher_ptr_, prev_frame.descriptors, curr_frame.descriptors, kp_matches,
                         ratio_thresh);
  }
  _timers["Correspondence Matching"].End();

  _timers["Correspondence Filtering 2D"].Start();
  FilterCorrespondencesByMedianVerticalPixelShift(
      prev_frame.kps, curr_frame.kps, kp_matches,
      config_["correspondenceFiltering"]["2D"]["verticalShiftDeviation"].as<double>());
  _timers["Correspondence Filtering 2D"].End();

  // Convert DMatch to Correspondences
  _timers["Correspondence Filtering 3D"].Start();

  CvDmatchesToPclCorrespondences(kp_matches, correspondences);

  FilterCorrespondencesByMedian3dDistances(
      *(prev_frame.kps_pc_ptr), *(curr_frame.kps_pc_ptr), correspondences,
      config_["correspondenceFiltering"]["3D"]["yDistanceDeviation"].as<double>(),
      config_["correspondenceFiltering"]["3D"]["zDistanceDeviation"].as<double>(),
      config_["correspondenceFiltering"]["3D"]["l2DistanceDeviation"].as<double>());

  if (config_["correspondenceFiltering"]["3D"]["ransac"]["enable"].as<bool>())
  {
    ransac_rejector_.setInputSource(prev_frame.kps_pc_ptr);
    ransac_rejector_.setInputTarget(curr_frame.kps_pc_ptr);
    ransac_rejector_.getRemainingCorrespondences(correspondences, correspondences);
  }

  _timers["Correspondence Filtering 3D"].End();

  bool has_min_inliers = (int)correspondences.size() >= config_["poseRejector"]["minInliers"].as<int>();
  if (!has_min_inliers)
  {
    return false;
  }

  _timers["Pose Estimation"].Start();
  std::string pose_estimation_type = config_["poseEstimator"]["name"].as<std::string>();

  Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();

  // std::cout << "Number correspondences: " << correspondences.size() << std::endl;

  if (pose_estimation_type == "svd")
  {
    // Transformation estimate SVD
    Eigen::Matrix4f svd_transformation;
    transform_estimator_svd_.estimateRigidTransformation(*prev_frame.kps_pc_ptr, *curr_frame.kps_pc_ptr,
                                                         correspondences, svd_transformation);
    current_transform = sl_sensor::conversions::InvertTransformationMatrix(svd_transformation);
  }
  else if (pose_estimation_type == "dq")
  {
    // Transformation estimate DQ
    Eigen::Matrix4f dq_transformation;

    transform_estimator_dq_.estimateRigidTransformation(*prev_frame.kps_pc_ptr, *curr_frame.kps_pc_ptr, correspondences,
                                                        dq_transformation);
    current_transform = sl_sensor::conversions::InvertTransformationMatrix(dq_transformation);
  }
  else if (pose_estimation_type == "2d")
  {
    Eigen::Matrix4f two_dim_transformation;

    Eigen::Matrix4f cam_vehicle_transformation;
    cam_vehicle_transformation << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f;

    pcl::PointCloud<pcl::PointXYZ> prev_frame_pc_vehicle_frame;
    pcl::PointCloud<pcl::PointXYZ> curr_frame_pc_vehicle_frame;

    pcl::transformPointCloud(*prev_frame.kps_pc_ptr, prev_frame_pc_vehicle_frame, cam_vehicle_transformation);
    pcl::transformPointCloud(*curr_frame.kps_pc_ptr, curr_frame_pc_vehicle_frame, cam_vehicle_transformation);

    transform_estimator_2d_.estimateRigidTransformation(prev_frame_pc_vehicle_frame, curr_frame_pc_vehicle_frame,
                                                        correspondences, two_dim_transformation);

    two_dim_transformation = sl_sensor::conversions::SwapFramesMatrix(cam_vehicle_transformation) *
                             two_dim_transformation * cam_vehicle_transformation;

    current_transform = sl_sensor::conversions::InvertTransformationMatrix(two_dim_transformation);
  }
  _timers["Pose Estimation"].End();

  output_transform = current_transform;

  // Check whether to reject pose estimate or not
  bool within_vertical_shift_tolerance =
      std::abs(current_transform(1, 3)) <= config_["poseRejector"]["maxVerticalShift"].as<double>();
  bool has_no_nan = !current_transform.hasNaN();
  bool successful_pose_estimation = (has_no_nan && has_min_inliers && within_vertical_shift_tolerance);

  return successful_pose_estimation;
}

void StructuredLightOdometry::Init()
{
  async_spinnter_ptr_ = std::make_unique<ros::AsyncSpinner>(2);
  async_spinnter_ptr_->start();

  int input_type_int = 2;
  nh_.param<int>("input_type", input_type_int, input_type_int);

  input_type_ = (InputType)input_type_int;

  if (input_type_ == InputType::kNumberedFrames)
  {
    InitNumberedFrameReader();
  }
  else if (input_type_ == InputType::kRosSubscriber)
  {
    InitRosSubscribers();
  }
  else
  {
    std::cout << "Invalid input format!" << std::endl;
    throw;
  }

  nh_.param<bool>("display_correspondences", display_correspondences_, display_correspondences_);

  InitRosPublishers();
  InitYaml();
  InitStructuredLight();
  InitOdometry();
  InitCsvOutput();
}

void StructuredLightOdometry::InitNumberedFrameReader()
{
  std::string folder_directory;
  std::string file_title;
  std::string file_format;
  int frame_no_start;
  int frame_no_end;
  int pat_no_min;
  int pat_no_max;
  int frame_increment = 1;

  nh_.param<std::string>("folder_directory", folder_directory, folder_directory);
  nh_.param<std::string>("file_title", file_title, file_title);
  nh_.param<std::string>("file_format", file_format, file_format);
  nh_.param<int>("frame_no_start", frame_no_start, frame_no_start);
  nh_.param<int>("frame_no_end", frame_no_end, frame_no_end);
  nh_.param<int>("pat_no_min", pat_no_min, pat_no_min);
  nh_.param<int>("pat_no_max", pat_no_max, pat_no_max);
  nh_.param<int>("frame_increment", frame_increment, frame_increment);

  numbered_frame_reader_ = sl_sensor::io::NumberedFrameReader(folder_directory, file_title, file_format, frame_no_start,
                                                              frame_no_end, frame_increment, pat_no_min, pat_no_max);
}

void StructuredLightOdometry::InitRosSubscribers()
{
  // Get subscribe topics
  nh_.param<std::string>("projector_trigger_topic", projector_trigger_topic_, projector_trigger_topic_);
  nh_.param<std::string>("image_topic", image_topic_, image_topic_);

  // Setup subscribers
  image_subscriber_ = nh_.subscribe(image_topic_, 10, &StructuredLightOdometry::ImageCb, this);
  projector_trigger_time_subscriber_ =
      nh_.subscribe(projector_trigger_topic_, 5, &StructuredLightOdometry::ProjectorTimeCb, this);

  // Some other params that are required to get an image sequence for a single frame
  nh_.param<int>("number_patterns_per_frame", number_patterns_per_frame_, number_patterns_per_frame_);
  nh_.param<double>("image_tolerance_time", image_tolerance_time_, image_tolerance_time_);
  nh_.param<double>("time_per_pattern", time_per_pattern_, time_per_pattern_);
}

void StructuredLightOdometry::InitRosPublishers()
{
  nh_.param<bool>("publish_pose_stamped", publish_pose_stamped_, publish_pose_stamped_);
  nh_.param<bool>("publish_correspondences", publish_correspondences_, publish_correspondences_);

  // Publisher for registered point cloud
  if (publish_correspondences_)
  {
    kp_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(keypoints_topic_, 1);
  }

  // Publisher for odometry
  if (publish_pose_stamped_)
  {
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
  }
}

void StructuredLightOdometry::InitYaml()
{
  std::string yaml_directory;
  nh_.param<std::string>("yaml_directory", yaml_directory, yaml_directory);
  config_ = YAML::LoadFile(yaml_directory);
}

void StructuredLightOdometry::InitStructuredLight()
{
  // Load calibration params
  std::string calibration_data_filename;
  nh_.param<std::string>("calibration_filename", calibration_data_filename, calibration_data_filename);
  calibration_data_.loadXML(calibration_data_filename);

  // Image undistorter setup
  image_undistorter_ptr_ = std::make_unique<sl_sensor::slstudio::ImageUndistorter>(calibration_data_);

  // Decoder setup
  unsigned int screen_cols = 2 * calibration_data_.screenResX;  // Multiply by 2 because diamond pixels
  unsigned int screen_rows = calibration_data_.screenResY;
  decoder_ptr_ = std::make_unique<sl_sensor::slstudio::DecoderPhaseShift2p1Tpu>(screen_cols, screen_rows,
                                                                                sl_sensor::slstudio::CodecDirVertical);

  // Triangulator setup
  triangulator_ptr_ = std::make_unique<sl_sensor::slstudio::Triangulator>(calibration_data_);

  // Point cloud scaling factor after triangulation
  nh_.param<double>("scaling_factor", scaling_factor_, scaling_factor_);
}

void StructuredLightOdometry::InitOdometry()
{
  // MSER detector setup
  int delta = config_["featureDetector"]["mser"]["delta"].as<int>();
  int min_area = config_["featureDetector"]["mser"]["minArea"].as<int>();
  int max_area = config_["featureDetector"]["mser"]["maxArea"].as<int>();
  double max_variation = config_["featureDetector"]["mser"]["maxVariation"].as<double>();

  mser_detector_ptr_ = cv::MSER::create(delta, min_area, max_area, max_variation);

  // SURF descriptor setup
  double hessianThreshold = config_["featureDescriptor"]["surf"]["hessianThreshold"].as<double>();
  int nOctaves = config_["featureDescriptor"]["surf"]["nOctaves"].as<int>();
  int nOctaveLayers = config_["featureDescriptor"]["surf"]["nOctaveLayers"].as<int>();
  bool extended = config_["featureDescriptor"]["surf"]["extended"].as<bool>();
  bool upright = config_["featureDescriptor"]["surf"]["upright"].as<bool>();
  surf_descriptor_ptr_ = cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);

  // Descriptor Matcher Setup
  feature_matcher_ptr_ = cv::FlannBasedMatcher::create();

  // Ransac rejector setup
  double inlier_threshold = config_["correspondenceFiltering"]["3D"]["ransac"]["threshold"].as<double>();  // in mm
  ransac_rejector_.setInlierThreshold(inlier_threshold);
}

void StructuredLightOdometry::DisplayFeatureCorrespondences(const VisualOdometryFrame &prev_frame,
                                                            const VisualOdometryFrame &curr_frame,
                                                            const pcl::Correspondences &correspondences)
{
  std::vector<cv::DMatch> better_matches;
  PclCorrespondencesToCvDmatches(correspondences, better_matches);
  cv::Mat better_img_matches;
  cv::drawMatches(prev_frame.reference_image_8uc, prev_frame.kps, curr_frame.reference_image_8uc, curr_frame.kps,
                  better_matches, better_img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow("Matches", better_img_matches);
  cv::waitKey(0);
}

void StructuredLightOdometry::PublishPoseStamped(const Eigen::Matrix4f &pose, const ros::Time &timestamp)
{
  pose_publisher_.publish(sl_sensor::conversions::EigenToPoseStamped(pose, timestamp, frame_name_));
}

void StructuredLightOdometry::PublishCorrespondences(const VisualOdometryFrame &frame,
                                                     const pcl::Correspondences &correspondences,
                                                     const Eigen::Matrix4f &pose, const ros::Time &timestamp)
{
  // Create point cloud where there are only correspondences
  std::vector<int> correspondence_indices;

  int max = 10;

  int counter = 0;

  for (const auto &correspondence : correspondences)
  {
    correspondence_indices.push_back(correspondence.index_match);

    counter++;
    if (counter >= max)
    {
      break;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pub_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(*frame.kps_pc_ptr, correspondence_indices));
  pcl::transformPointCloud(*pub_pc_ptr, *pub_pc_ptr, pose);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*pub_pc_ptr, pc_msg);
  pc_msg.header.frame_id = frame_name_;
  pc_msg.header.stamp = timestamp;
  kp_publisher_.publish(pc_msg);
}

bool StructuredLightOdometry::GetNextImageSequenceFromSubscribers(std::vector<cv::Mat> &image_sequence,
                                                                  std::vector<ros::Time> &timestamp_vec)
{
  bool success = false;

  boost::mutex::scoped_lock lock(mutex_);

  // If not enough projector timings / images we do not attempt any matching
  if (projector_time_buffer_.empty() || (int)image_ptr_buffer_.size() < number_patterns_per_frame_)
  {
    return false;
  }

  std::vector<sensor_msgs::ImageConstPtr> temp_cv_img_ptr_vec = {};
  ros::Time successful_projector_time;

  for (const auto &projector_time : projector_time_buffer_)
  {
    temp_cv_img_ptr_vec.clear();

    // We check if we can retrieve all images that make up a frame for the current projector time
    for (int i = 0; i < number_patterns_per_frame_; i++)
    {
      auto target_time = projector_time + ros::Duration(i * time_per_pattern_);

      sensor_msgs::ImageConstPtr temp_ptr;

      bool image_acquisition_successful = GetImageFromImageBuffer(target_time, temp_ptr);

      if (image_acquisition_successful)
      {
        // Successful frame acquisition, store the image pointer in temp_cv_img_ptr_vec
        temp_cv_img_ptr_vec.push_back(temp_ptr);
        continue;
      }
      else
      {
        // If not successful, we stop the looking for next image
        // std::cout << "Unsuccessful at the " << i + 1 << "th image" << std::endl;
        break;
      }
    }

    // if successful (all image pointer for a frame in temp_cv_img_ptr_vec)
    if ((int)temp_cv_img_ptr_vec.size() == number_patterns_per_frame_)
    {
      successful_projector_time = projector_time;
      success = true;
      // Exit while loop
      break;
    }
  }

  // Convert to CV_32F format from MONO8 which is the required format for processing
  if (success)
  {
    timestamp_vec.clear();

    for (const auto img_ptr : temp_cv_img_ptr_vec)
    {
      auto cv_img_ptr = cv_bridge::toCvShare(img_ptr, sensor_msgs::image_encodings::MONO8);
      image_sequence.push_back(cv::Mat(cv_img_ptr->image.size(), CV_32F));
      (cv_img_ptr->image).convertTo(image_sequence.back(), CV_32F, 1.0f / 255.0f);

      timestamp_vec.push_back(img_ptr->header.stamp);
    }

    // Clear all images before and including this frame
    ClearAllImagesFromBufferBeforeTiming(temp_cv_img_ptr_vec.back()->header.stamp);

    // Clear all projector times before and including this frame
    ClearAllProjectorTimingsFromBufferBeforeTiming(successful_projector_time);
  }

  return success;
}

void StructuredLightOdometry::ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time &target_time)
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

void StructuredLightOdometry::ClearAllImagesFromBufferBeforeTiming(const ros::Time &target_time)
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

bool StructuredLightOdometry::GetImageFromImageBuffer(const ros::Time &target_time,
                                                      sensor_msgs::ImageConstPtr &image_ptr)
{
  bool result = false;

  for (const auto &cv_img_ptr : image_ptr_buffer_)
  {
    double delta_t = (cv_img_ptr->header.stamp - target_time).toSec();
    if (delta_t >= -1.0 * image_tolerance_time_ && delta_t <= image_tolerance_time_)
    {
      // If we find a image that satisfies the requirement, write the pointer to image_ptr and break the loop
      image_ptr = cv_img_ptr;
      result = true;
      break;
    }
    else if (delta_t > image_tolerance_time_)
    {
      // If image is in the future of target time, we stop looking
      result = false;
      break;
    }
  }

  return result;
}

void StructuredLightOdometry::ImageCb(const sensor_msgs::ImageConstPtr &image_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  image_ptr_buffer_.push_back(image_ptr);
}

void StructuredLightOdometry::ProjectorTimeCb(const versavis::TimeNumberedConstPtr &time_numbered_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  projector_time_buffer_.push_back(time_numbered_ptr->time);
}

void StructuredLightOdometry::InitCsvOutput()
{
  // Csv file for relative pose between frames
  nh_.param<std::string>("output_relative_pose_csv_filename", output_relative_pose_csv_filename_,
                         output_relative_pose_csv_filename_);

  if (!output_relative_pose_csv_filename_.empty())
  {
    write_output_relative_pose_csv_ = true;
    output_relative_pose_csv_writer_ptr_ =
        std::make_unique<sl_sensor::io::TumCsvWriter>(output_relative_pose_csv_filename_);
  }

  // Csv file for image timings for each successful frame
  nh_.param<std::string>("output_frame_timings_csv_filename", output_frame_timings_csv_filename_,
                         output_frame_timings_csv_filename_);

  if (!output_frame_timings_csv_filename_.empty() && input_type_ == InputType::kRosSubscriber)
  {
    write_output_frame_timings_csv_ = true;
    output_frame_timings_csv_writer_ptr_ =
        std::make_unique<sl_sensor::io::FrameTimingsCsvWriter>(output_frame_timings_csv_filename_);
  }
}

void StructuredLightOdometry::PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                                                  int reference_indice,
                                                                  std::vector<cv::Point2d> &shifts)
{
  shifts.clear();

  for (int i = 0; i < (int)image_sequence.size(); i++)
  {
    if (i != reference_indice)
    {
      shifts.push_back(cv::phaseCorrelate(image_sequence[reference_indice], image_sequence[i]));
    }
    else
    {
      shifts.push_back(cv::Point2d(0.0f, 0.0f));
    }
  }
}

void StructuredLightOdometry::PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                                                  int reference_indice,
                                                                  std::vector<cv::Point2d> &shifts,
                                                                  double subsample_factor)
{
  if (subsample_factor <= 0.0f)
  {
    PhaseCorrelateRegisterImageSequence(image_sequence, reference_indice, shifts);
  }
  else
  {
    std::vector<cv::Mat> subsampled_image_sequence = {};
    SubsampleImageSequence(image_sequence, subsampled_image_sequence, subsample_factor);
    PhaseCorrelateRegisterImageSequence(subsampled_image_sequence, reference_indice, shifts);

    for (auto &shift : shifts)
    {
      shift /= subsample_factor;
    }
  }
}

void StructuredLightOdometry::ApplyShiftsToImageSequence(std::vector<cv::Mat> &input_image_sequence,
                                                         std::vector<cv::Mat> &output_image_sequence,
                                                         const std::vector<cv::Point2d> &shifts)
{
  output_image_sequence.clear();

  for (int i = 0; i < (int)shifts.size(); i++)
  {
    output_image_sequence.push_back(cv::Mat::zeros(input_image_sequence[i].size(), input_image_sequence[i].depth()));

    float x = shifts[i].x;
    float y = shifts[i].y;
    bool non_zero_shift = !(x == 0.0f && y == 0.0f);

    // std::cout << "Shifts: " << x << " | " << y << std::endl;

    if (non_zero_shift)
    {
      cv::Mat M = (cv::Mat_<float>(2, 3) << 1.0f, 0.0f, x, 0.0f, 1.0f, y);
      // std::cout << M << std::endl;
      cv::warpAffine(input_image_sequence[i], output_image_sequence[i], M, input_image_sequence[i].size(),
                     cv::WARP_INVERSE_MAP);
    }
    else
    {
      input_image_sequence[i].copyTo(output_image_sequence[i]);
    }
  }
}

void StructuredLightOdometry::ConverMserRectsToKeypoints(const std::vector<cv::Rect> &rect_vec,
                                                         std::vector<cv::KeyPoint> &keypoint_vec)
{
  keypoint_vec.clear();

  for (const auto &rect : rect_vec)
  {
    keypoint_vec.push_back(
        cv::KeyPoint(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f, std::max(rect.width, rect.height)));
  }
}

void StructuredLightOdometry::DetectMserKeypoints(cv::Ptr<cv::MSER> mser_detector_ptr, const cv::Mat &img,
                                                  std::vector<cv::KeyPoint> &keypoint_vec)
{
  // cv::Mat img_8uc;
  // img.convertTo(img_8uc, CV_8UC1, 255.0f);
  std::vector<std::vector<cv::Point>> regions;
  std::vector<cv::Rect> mser_rects;
  mser_detector_ptr->detectRegions(img, regions, mser_rects);
  ConverMserRectsToKeypoints(mser_rects, keypoint_vec);
}

void StructuredLightOdometry::GetDescriptorMatches(cv::Ptr<cv::DescriptorMatcher> matcher_ptr,
                                                   const cv::Mat &descriptors_static, const cv::Mat &descriptors_moving,
                                                   std::vector<cv::DMatch> &good_matches, float ratio_thresh)
{
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher_ptr->knnMatch(descriptors_static, descriptors_moving, knn_matches, 2);

  //-- Filter matches using the Lowe's ratio test
  for (size_t i = 0; i < (size_t)knn_matches.size(); i++)
  {
    if (knn_matches[i].size() == 2)
    {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
      {
        good_matches.push_back(knn_matches[i][0]);
      }
    }
  }
}

void StructuredLightOdometry::CvDmatchesToPclCorrespondences(const std::vector<cv::DMatch> &dmatches,
                                                             pcl::Correspondences &correspondences)
{
  correspondences.clear();

  for (const auto &dmatch : dmatches)
  {
    correspondences.push_back(pcl::Correspondence(dmatch.queryIdx, dmatch.trainIdx, dmatch.distance));
  }
}

void StructuredLightOdometry::PclCorrespondencesToCvDmatches(const pcl::Correspondences &correspondences,
                                                             std::vector<cv::DMatch> &dmatches)
{
  dmatches.clear();

  for (const auto &correspondence : correspondences)
  {
    dmatches.push_back(cv::DMatch(correspondence.index_query, correspondence.index_match, correspondence.distance));
  }
}

void StructuredLightOdometry::SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                                                     std::vector<cv::Mat> &image_sequence_output,
                                                     double subsample_factor)
{
  image_sequence_output.clear();

  for (const auto &input_image : image_sequence_input)
  {
    image_sequence_output.push_back(cv::Mat{ input_image.size(), input_image.type() });
    cv::resize(input_image, image_sequence_output.back(), cv::Size(), subsample_factor, subsample_factor,
               cv::InterpolationFlags::INTER_NEAREST);
  }
}

void StructuredLightOdometry::KeepOnlySymmetricMatches(const std::vector<cv::DMatch> &matches12,
                                                       const std::vector<cv::DMatch> &matches21,
                                                       std::vector<cv::DMatch> &symmetric_matches)
{
  symmetric_matches.clear();

  for (const auto &match12 : matches12)
  {
    for (const auto &match21 : matches21)
    {
      // Match symmetry test
      if ((match12.queryIdx == match21.trainIdx) && (match12.queryIdx == match21.trainIdx))
      {
        // add symmetrical match
        symmetric_matches.push_back(cv::DMatch(match12.queryIdx, match12.trainIdx, match12.distance));
        break;
      }
    }
  }
}

void StructuredLightOdometry::FilterCorrespondencesByMedianVerticalPixelShift(
    const std::vector<cv::KeyPoint> &static_kps, const std::vector<cv::KeyPoint> &moving_kps,
    std::vector<cv::DMatch> &kp_matches, double threshold)
{
  // We do not process if correspondences is empty (get_median function will complain)
  if (kp_matches.empty())
  {
    return;
  }

  std::vector<double> vertical_pixel_shifts;

  for (const auto &kp_match : kp_matches)
  {
    vertical_pixel_shifts.push_back(static_kps[kp_match.queryIdx].pt.y - moving_kps[kp_match.trainIdx].pt.y);
  }

  double median_shift = GetMedian(vertical_pixel_shifts);

  size_t j = 0;
  for (size_t i = 0; i < (size_t)vertical_pixel_shifts.size(); ++i)
  {
    if (vertical_pixel_shifts[i] > median_shift + threshold || vertical_pixel_shifts[i] < median_shift - threshold)
      continue;
    kp_matches[j] = kp_matches[i];
    j++;
  }
  kp_matches.resize(j);
}

void StructuredLightOdometry::FilterCorrespondencesByMedian3dDistances(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                                       const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                                       pcl::Correspondences &correspondences,
                                                                       double y_threshold, double z_threshold,
                                                                       double l2_threshold)
{
  // We do not process if correspondences is empty (get_median function will complain)
  if (correspondences.empty())
  {
    return;
  }

  std::vector<double> y_shifts = {};
  std::vector<double> z_shifts = {};
  std::vector<double> l2_shifts = {};

  // Get vector of euclidian distances
  for (const auto &correspondence : correspondences)
  {
    pcl::PointXYZ source_pt = static_pc.at(correspondence.index_query);
    pcl::PointXYZ target_pt = moving_pc.at(correspondence.index_match);
    float l2_dist = std::sqrt(pow(source_pt.x - target_pt.x, 2) + pow(source_pt.y - target_pt.y, 2) +
                              pow(source_pt.z - target_pt.z, 2));
    l2_shifts.push_back(l2_dist);
    y_shifts.push_back(source_pt.y - target_pt.y);
    z_shifts.push_back(source_pt.z - target_pt.z);
  }

  double median_l2 = GetMedian(l2_shifts);
  double median_y = GetMedian(y_shifts);
  double median_z = GetMedian(z_shifts);

  size_t j = 0;
  for (size_t i = 0; i < (size_t)y_shifts.size(); ++i)
  {
    bool l2_outside_threshold = l2_shifts[i] > median_l2 + l2_threshold || l2_shifts[i] < median_l2 - l2_threshold;
    bool y_shift_outside_threshold = y_shifts[i] > median_y + y_threshold || y_shifts[i] < median_y - y_threshold;
    bool z_shift_outside_threshold = z_shifts[i] > median_z + z_threshold || z_shifts[i] < median_z - z_threshold;

    if (l2_outside_threshold || y_shift_outside_threshold || z_shift_outside_threshold)
      continue;

    correspondences[j] = correspondences[i];
    j++;
  }

  correspondences.resize(j);
}

}  // namespace vo
}  // namespace sl_sensor