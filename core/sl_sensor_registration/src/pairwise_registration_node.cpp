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

#include <sl_sensor_timer/timer.hpp>
#include "sl_sensor_registration/pc_utilities.hpp"
#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"
#include "sl_sensor_registration/point_cloud_registration_algorithm_factory.hpp"

using namespace sl_sensor::registration;
using namespace sl_sensor::timer;

bool FileExists(const std::string &name);
geometry_msgs::Pose EigenToPose(const Eigen::Matrix4f &matrix);
geometry_msgs::PoseStamped EigenToPoseStamped(const Eigen::Matrix4f &matrix, const ros::Time &time,
                                              const std::string &frame_id);

int main(int argc, char **argv) {
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

  std::string header_name = "log_";
  nh.param<std::string>("header_name", header_name, header_name);

  bool enable_sor = true;
  nh.param<bool>("enable_sor", enable_sor, enable_sor);

  int nearest_k = 30;
  nh.param<int>("nearest_k", nearest_k, nearest_k);

  double std_dev = 1.0;
  nh.param<double>("std_dev", std_dev, std_dev);

  std::string input_pc_file_format = ".pcd";
  std::string output_pc_file_format = ".pcd";
  bool save_registered_pc = true;

  int row_min;
  nh.param<int>("row_min", row_min, 0);

  int row_max;
  nh.param<int>("row_max", row_max, 1000);

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

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter_xyzi;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;

  Eigen::Matrix4f vo_pose = Eigen::Matrix4f::Identity();

  std::cout << "Start point cloud registration" << std::endl;

  for (size_t row = (size_t)row_min; row <= (size_t)row_max; row++) {
    // Read point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr stitch_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr registration_pc_xyzi_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration_pc_ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string registration_pc_full_directory =
        registration_pc_directory + header_name + std::to_string(row) + input_pc_file_format;
    std::string stitch_pc_full_directory =
        stitch_pc_directory + header_name + std::to_string(row) + input_pc_file_format;

    std::cout << "trying to open " << registration_pc_full_directory << std::endl;

    // If pc file does not exist, skip this frame
    if (!FileExists(registration_pc_full_directory) || !FileExists(stitch_pc_full_directory)) {
      continue;
    }

    if (input_pc_file_format == ".pcd") {
      pcl::io::loadPCDFile(registration_pc_full_directory, *registration_pc_xyzi_ptr);
      pcl::io::loadPCDFile(stitch_pc_full_directory, *stitch_pc_ptr);
    } else if (input_pc_file_format == ".ply") {
      pcl::io::loadPLYFile(registration_pc_full_directory, *registration_pc_xyzi_ptr);
      pcl::io::loadPLYFile(stitch_pc_full_directory, *stitch_pc_ptr);
    } else {
      std::cout << "Invalid point cloud format: " << input_pc_file_format << std::endl;
      throw;
    }

    copyPointCloud(*registration_pc_xyzi_ptr, *registration_pc_ptr);

    // Iterate over each point
    for (size_t i = 0; i < registration_pc_xyzi_ptr->size(); ++i) {
      registration_pc_ptr->points[i].r = registration_pc_xyzi_ptr->points[i].intensity;
      registration_pc_ptr->points[i].g = registration_pc_xyzi_ptr->points[i].intensity;
      registration_pc_ptr->points[i].b = registration_pc_xyzi_ptr->points[i].intensity;
    }

    std::vector<int> register_nan_indices;
    pcl::removeNaNFromPointCloud(*registration_pc_ptr, *registration_pc_ptr, register_nan_indices);

    if (enable_sor) {
      sor_filter.setInputCloud(registration_pc_ptr);
      sor_filter.setMeanK(nearest_k);
      sor_filter.setStddevMulThresh(std_dev);
      sor_filter.filter(*registration_pc_ptr);
    }

    // Perform PC registraion using feature guess
    timer.Start();
    pc_registration_ptr->RegisterPointCloud(registration_pc_ptr);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::string final_pc_full_directory =
        output_pc_directory + std::to_string(row) + output_pc_file_format;
    pcl::transformPointCloud(*stitch_pc_ptr, *final_pc_ptr, current_transform);

    std::vector<int> stitch_nan_indices;
    pcl::removeNaNFromPointCloud(*final_pc_ptr, *final_pc_ptr, stitch_nan_indices);

    if (enable_sor) {
      sor_filter_xyzi.setInputCloud(final_pc_ptr);
      sor_filter_xyzi.setMeanK(nearest_k);
      sor_filter_xyzi.setStddevMulThresh(std_dev);
      sor_filter_xyzi.filter(*final_pc_ptr);
    }

    if (save_registered_pc) {
      if (output_pc_file_format == ".pcd") {
        pcl::io::savePCDFileASCII(final_pc_full_directory, *final_pc_ptr);
      } else if (output_pc_file_format == ".ply") {
        pcl::io::savePLYFileASCII(final_pc_full_directory, *final_pc_ptr);
      } else {
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
    pcl::toROSMsg(*GetSubsampledPointCloud<pcl::PointXYZI>(final_pc_ptr, rviz_leafsize), pc_msg);
    pc_msg.header.frame_id = frame;
    pc_pub.publish(pc_msg);
  }

  ros::shutdown();
  return 0;
}

bool FileExists(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

geometry_msgs::PoseStamped EigenToPoseStamped(const Eigen::Matrix4f &matrix, const ros::Time &time,
                                              const std::string &frame_id) {
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = time;
  pose_stamped.pose = EigenToPose(matrix);

  return pose_stamped;
};

geometry_msgs::Pose EigenToPose(const Eigen::Matrix4f &matrix) {
  geometry_msgs::Pose pose_output;

  auto md = matrix.cast<double>();

  tf::Matrix3x3 tf3d;
  tf3d.setValue(md(0, 0), md(0, 1), md(0, 2), md(1, 0), md(1, 1), md(1, 2), md(2, 0), md(2, 1),
                md(2, 2));

  tf::Quaternion quat;
  tf3d.getRotation(quat);

  pose_output.position.x = md(0, 3);
  pose_output.position.y = md(1, 3);
  pose_output.position.z = md(2, 3);
  pose_output.orientation.x = quat.getX();
  pose_output.orientation.y = quat.getY();
  pose_output.orientation.z = quat.getZ();
  pose_output.orientation.w = quat.getW();

  return pose_output;
};