#include <cmath>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>

#include <sl_sensor_conversions/conversions.hpp>
#include <sl_sensor_csv/tum_csv_reader.hpp>
#include <sl_sensor_csv/tum_csv_writer.hpp>
#include <sl_sensor_timer/timer.hpp>
#include "sl_sensor_registration/pc_utils.hpp"
#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"
#include "sl_sensor_registration/point_cloud_registration_algorithm_factory.hpp"

using namespace sl_sensor::registration;
using namespace sl_sensor::csv;
using namespace sl_sensor::conversions;
using namespace sl_sensor::timer;

inline bool FileExists(const std::string& name)
{
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "pairwise_registration_node");
  ros::NodeHandle nh;

  // Get file information
  std::cout << "Loading params from ROS node ..." << std::endl;
  std::string registration_pc_directory;
  nh.param<std::string>("registration_pc_directory", registration_pc_directory, "");

  std::string stitch_pc_directory;
  nh.param<std::string>("stitch_pc_directory", stitch_pc_directory, "");

  std::string output_pc_directory;
  nh.param<std::string>("output_pc_directory", output_pc_directory, "");

  std::string relative_pose_directory;
  nh.param<std::string>("relative_pose_directory", relative_pose_directory, "");

  std::string absolute_pose_directory;
  nh.param<std::string>("absolute_pose_directory", absolute_pose_directory, "");

  std::string input_pc_file_format;
  nh.param<std::string>("input_pc_file_format", input_pc_file_format, "");

  std::string output_pc_file_format;
  nh.param<std::string>("output_pc_file_format", output_pc_file_format, "");

  bool save_registered_pc;
  nh.param<bool>("save_registered_pc", save_registered_pc, false);

  int row_min;
  nh.param<int>("row_min", row_min, -1);

  int row_max;
  nh.param<int>("row_max", row_max, 100000);

  // Publisher topics
  std::cout << "Setting up ROS publishers ..." << std::endl;

  std::string pc_topic = "/registered_pc";
  std::string pose_topic = "/registered_pose";
  std::string frame = "/map";
  std::string vo_topic = "/vo_pose";

  // Publisher for registered point cloud
  double rviz_leafsize = 0.01f;
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  // Publisher for odometry
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);

  // Publisher for initial pose estimates
  ros::Publisher vo_pub = nh.advertise<geometry_msgs::PoseStamped>(vo_topic, 1);

  // Setup PC registration algo
  std::cout << "Setting up point cloud registration algorithm ..." << std::endl;

  std::string algo_name;
  nh.param<std::string>("pc_algo", algo_name, "");
  auto pc_registration_ptr = PointCloudRegistrationAlgorithmFactory::GetInstance(algo_name);
  pc_registration_ptr->LoadSettingsFromROSNodeHandle(nh);
  pc_registration_ptr->Init();
  pc_registration_ptr->PrintSettings();

  // Timer to compute mean time taken per registration
  Timer timer("Pairwise Point Cloud Registration (" + algo_name + ")");

  // Create csv viewer
  TumCsvReader csv_reader{ relative_pose_directory };
  TumPose initial_pose_guess;

  // Create csv writer
  TumCsvWriter csv_writer{ absolute_pose_directory };

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;

  int row = 1;

  Eigen::Matrix4f vo_pose = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f initial_guess_transform = Eigen::Matrix4f::Identity();

  std::cout << "Start point cloud registration" << std::endl;

  while (csv_reader.GetNextRow(initial_pose_guess))
  {
    if (row < row_min || row > row_max)
    {
      row++;
      continue;
    }

    // Point clouds are in mm, convert to m
    double scale = 0.001f;
    initial_pose_guess.ScaleTransform(scale);
    auto image_timestamp = initial_pose_guess.timestamp;
    std::cout << "Processing frame at image_timestamp " << image_timestamp << " ("
              << "Row " << row << " of csv file"
              << ")" << std::endl;
    Eigen::Matrix4f initial_guess_transform_curr_frame;
    initial_pose_guess.GetTransformationMatrix(initial_guess_transform_curr_frame);

    // Read point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitch_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string registration_pc_full_directory =
        registration_pc_directory + std::to_string(image_timestamp) + input_pc_file_format;
    std::string stitch_pc_full_directory = stitch_pc_directory + std::to_string(image_timestamp) + input_pc_file_format;

    // Accumulate initial guess poses
    // If registration is successful in previous frame, initial_guess_transform is reset to identity at the end of the
    // loop, if not we accumulate guess transforms
    initial_guess_transform = initial_guess_transform * initial_guess_transform_curr_frame;

    // If pc file does not exist, skip this frame
    if (!FileExists(registration_pc_full_directory) || !FileExists(stitch_pc_full_directory))
    {
      // We accumlate the pose estimates
      continue;
    }

    if (input_pc_file_format == ".pcd")
    {
      pcl::io::loadPCDFile(registration_pc_full_directory, *registration_pc_ptr);
      pcl::io::loadPCDFile(stitch_pc_full_directory, *stitch_pc_ptr);
    }
    else if (input_pc_file_format == ".ply")
    {
      pcl::io::loadPLYFile(registration_pc_full_directory, *registration_pc_ptr);
      pcl::io::loadPLYFile(stitch_pc_full_directory, *stitch_pc_ptr);
    }
    else
    {
      std::cout << "Invalid point cloud format: " << input_pc_file_format << std::endl;
      throw;
    }

    // Scale point clouds
    stitch_pc_ptr = ScalePointCloud<pcl::PointXYZRGB>(stitch_pc_ptr, scale);
    registration_pc_ptr = ScalePointCloud<pcl::PointXYZRGB>(registration_pc_ptr, scale);

    std::vector<int> register_nan_indices;
    pcl::removeNaNFromPointCloud(*registration_pc_ptr, *registration_pc_ptr, register_nan_indices);
    sor_filter.setInputCloud(registration_pc_ptr);
    // sor_filter.setMeanK(50);
    // sor_filter.setStddevMulThresh(0.03);
    sor_filter.setMeanK(30);
    sor_filter.setStddevMulThresh(1.0);
    sor_filter.filter(*registration_pc_ptr);

    std::cout << registration_pc_ptr->size() << std::endl;

    std::cout << "Initial guess transform: \n" << std::endl;
    std::cout << initial_guess_transform << std::endl;

    vo_pose = vo_pose * initial_guess_transform;

    // Perform PC registraion using feature guess
    timer.Start();
    pc_registration_ptr->RegisterPointCloud(registration_pc_ptr, initial_guess_transform);
    timer.End();
    timer.PrintAverageTiming();

    // Get transforms from registration
    auto relative_transform = pc_registration_ptr->GetRelativeTransform();
    auto current_transform = pc_registration_ptr->GetAbsoluteTransform();

    std::cout << "Relative transform: \n" << std::endl;
    std::cout << relative_transform << std::endl;

    std::cout << "Current transform: \n" << std::endl;
    std::cout << current_transform << std::endl;

    // Transform and save final point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string final_pc_full_directory = output_pc_directory + std::to_string(image_timestamp) + output_pc_file_format;
    pcl::transformPointCloud(*stitch_pc_ptr, *final_pc_ptr, current_transform);

    std::vector<int> stitch_nan_indices;
    pcl::removeNaNFromPointCloud(*final_pc_ptr, *final_pc_ptr, stitch_nan_indices);
    sor_filter.setInputCloud(final_pc_ptr);
    sor_filter.setMeanK(30);
    sor_filter.setStddevMulThresh(0.1);
    sor_filter.filter(*final_pc_ptr);

    if (save_registered_pc)
    {
      if (output_pc_file_format == ".pcd")
      {
        pcl::io::savePCDFileASCII(final_pc_full_directory, *final_pc_ptr);
      }
      else if (output_pc_file_format == ".ply")
      {
        pcl::io::savePLYFileASCII(final_pc_full_directory, *final_pc_ptr);
      }
      else
      {
        std::cout << "Invalid point cloud format: " << input_pc_file_format << std::endl;
        throw;
      }

      std::cout << "Saved transformed point cloud to : " << final_pc_full_directory << std::endl;
    }

    // Publish pose
    ros::Time ros_timestamp = ros::Time::now();
    pose_pub.publish(EigenToPoseStamped(current_transform, ros_timestamp, frame));
    vo_pub.publish(EigenToPoseStamped(vo_pose, ros_timestamp, frame));

    // We subsample point cloud to get a sparser one for visualisation so it does not lag rviz
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*GetSubsampledPointCloud<pcl::PointXYZRGB>(final_pc_ptr, rviz_leafsize), pc_msg);
    pc_msg.header.frame_id = frame;
    pc_pub.publish(pc_msg);

    // Write absolute pose to csv file
    TumPose absolute_pose{ image_timestamp, current_transform };
    csv_writer.WriteNextRow(absolute_pose);

    // Update counter
    row++;

    // Since successful registration, reset guess transformation
    initial_guess_transform = Eigen::Matrix4f::Identity();
  }

  ros::shutdown();
  return 0;
}