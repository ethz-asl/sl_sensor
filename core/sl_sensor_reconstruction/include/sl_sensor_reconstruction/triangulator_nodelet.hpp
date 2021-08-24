#pragma once

#include <nodelet/nodelet.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <memory>

#include "sl_sensor_reconstruction/triangulator.hpp"

namespace sl_sensor
{
namespace reconstruction
{
/**
 * @brief Nodelet that generates point clouds from decoded images
 *
 */
class TriangulatorNodelet : public nodelet::Nodelet
{
public:
  TriangulatorNodelet();

private:
  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pc_pub_;
  ros::Subscriber image_array_sub_;

  std::string image_array_sub_topic_ = "/decoded_images_input";
  std::string pc_pub_topic_ = "/point_cloud_output";
  std::string camera_parameters_filename_ = "";
  std::string projector_parameters_filename_ = "";
  std::string triangulation_camera_parameters_filename_ = "";
  std::string colour_camera_parameters_filename_ = "";
  std::string frame_camera_parameters_filename_ =
      "";  // Camera parameters file for the that we would transform the point cloud to

  std::unique_ptr<Triangulator> triangulator_ptr_;

  // Transform between camera that performs the triangulation and the desired sensor frame, only used if
  // frame_camera_parameters_filename_ is specified
  Eigen::Matrix4f transform_sensor_tri_ = Eigen::Matrix4f::Identity();

  bool apply_crop_box_ = false;
  float crop_box_x_min_ = -1.0e8;
  float crop_box_y_min_ = -1.0e8;
  float crop_box_z_min_ = -1.0e8;
  float crop_box_x_max_ = 1.0e8;
  float crop_box_y_max_ = 1.0e8;
  float crop_box_z_max_ = 1.0e8;

  bool colour_shading_enabled_ = false;
  bool frame_camera_provided_ = false;

  template <typename PointT>
  void ApplyCropBox(typename pcl::PointCloud<PointT>::Ptr pc_ptr)
  {
    pcl::CropBox<PointT> crop_box;
    crop_box.setMin(Eigen::Vector4f(crop_box_x_min_, crop_box_y_min_, crop_box_z_min_, 1.0));
    crop_box.setMax(Eigen::Vector4f(crop_box_x_max_, crop_box_y_max_, crop_box_z_max_, 1.0));
    crop_box.setInputCloud(pc_ptr);
    crop_box.filter(*pc_ptr);
  };

  template <typename PointT>
  void PublishPointCloud(typename pcl::PointCloud<PointT>::Ptr pc_ptr,
                         const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr)
  {
    // Publish point cloud
    sensor_msgs::PointCloud2Ptr pc_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*pc_ptr, *pc_msg_ptr);
    pc_msg_ptr->header.frame_id = image_array_ptr->header.frame_id;
    pc_msg_ptr->header.stamp = image_array_ptr->header.stamp;
    pc_pub_.publish(pc_msg_ptr);
  };

  template <typename PointT>
  void PostProcessAndPublishPointCloud(typename pcl::PointCloud<PointT>::Ptr pc_ptr,
                                       const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr)
  {
    // Apply box filter if specified (point cloud will no longer be organised!)
    if (apply_crop_box_)
    {
      ApplyCropBox<PointT>(pc_ptr);
    }

    // Transform to the coordinate frame of the specified camera, if specified
    if (frame_camera_provided_)
    {
      pcl::transformPointCloud(*pc_ptr, *pc_ptr, transform_sensor_tri_);
    }

    // Publish point cloud
    PublishPointCloud<PointT>(pc_ptr, image_array_ptr);
  }
};

}  // namespace reconstruction
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::reconstruction::TriangulatorNodelet, nodelet::Nodelet);
