/***************************************************************************************************
 * This file is part of sl_sensor.
 *
 * sl_sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sl_sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sl_sensor.  If not, see <https://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#ifndef SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_NODELET_HPP_
#define SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_NODELET_HPP_

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

namespace sl_sensor {
namespace reconstruction {
/**
 * @brief Nodelet that generates point clouds from decoded images
 * Takes in an image array message that has either 4 or 5 images
 * For standard cam-projector triangulation, we expect 4 images in sequence (up, vp, mask, shading)
 * For cam-projector triangulation + colour information from colour camera, we expect 5 images (up,
 * vp, mask, shading, rgb image from colour camera) --> Please provide colour camera intrinsic
 * parameters file directory
 *  Nodelet also performs some post point cloud processing:
 *  - Bounding box filter so output point cloud only has points within certain bounds
 *  - Scaling to convert point clouds in mm to m (useful when calibration is done in mm but tf
 * messages from ROS are in m)
 *  - Translating point cloud to another camera's frame (useful when you have two cameras and one of
 * them is set to be the sensor frame and you are triangulating from the other camera) --> Please
 * provide frame_camera camera intrinsic parameters file directory
 */
class TriangulatorNodelet : public nodelet::Nodelet {
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

  std::unique_ptr<Triangulator> triangulator_ptr_;

  // Crop box parameters
  bool apply_crop_box_ = false;
  float crop_box_x_min_ = -1.0e8;
  float crop_box_y_min_ = -1.0e8;
  float crop_box_z_min_ = -1.0e8;
  float crop_box_x_max_ = 1.0e8;
  float crop_box_y_max_ = 1.0e8;
  float crop_box_z_max_ = 1.0e8;

  // Colour shading parameters
  bool colour_shading_enabled_ = false;
  std::string colour_camera_parameters_filename_ = "";

  // Transform to sensor frame parameters
  bool frame_camera_provided_ = false;
  std::string frame_camera_parameters_filename_ =
      "";  // Camera parameters file for the that we would transform the point cloud to
  Eigen::Matrix4f transform_sensor_tri_ =
      Eigen::Matrix4f::Identity();  // Transform between camera that performs the triangulation and
                                    // the desired sensor frame, only used if
  // frame_camera_parameters_filename_ is specified

  // Scaling parameters
  bool apply_scaling_ = false;
  float scaling_factor_ = 1.0;

  /**
   * @brief Apply crop box filter
   *
   * @tparam PointT PCL point type
   * @param pc_ptr - Point to pcl point cloud to be filtered
   */
  template <typename PointT>
  void ApplyCropBox(typename pcl::PointCloud<PointT>::Ptr pc_ptr) {
    pcl::CropBox<PointT> crop_box;
    crop_box.setMin(Eigen::Vector4f(crop_box_x_min_, crop_box_y_min_, crop_box_z_min_, 1.0));
    crop_box.setMax(Eigen::Vector4f(crop_box_x_max_, crop_box_y_max_, crop_box_z_max_, 1.0));
    crop_box.setInputCloud(pc_ptr);
    crop_box.filter(*pc_ptr);
  };

  /**
   * @brief Publish a triangulated point cloud
   *
   * @tparam PointT PCL point type
   * @param pc_ptr - Point to pcl point cloud to be published
   * @param image_array_ptr - Original image array message that has the images from which the point
   * cloud was triangulated
   */
  template <typename PointT>
  void PublishPointCloud(typename pcl::PointCloud<PointT>::Ptr pc_ptr,
                         const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr) {
    // Publish point cloud
    sensor_msgs::PointCloud2Ptr pc_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*pc_ptr, *pc_msg_ptr);
    // Assign used camera as frame_id
    if(triangulation_camera_parameters_filename_.find("primary") != std::string::npos){
      pc_msg_ptr->header.frame_id = "cam1";
    }else{
      pc_msg_ptr->header.frame_id = "cam2";
    }
    pc_msg_ptr->header.stamp = image_array_ptr->header.stamp;
    pc_pub_.publish(pc_msg_ptr);
  };

  /**
   * @brief Scale a point cloud, input is a smart pointer of original point cloud
   *
   * @tparam PointT - Type of pcl point
   * @param cloud_in_ptr - smart pointer of original point cloud
   * @param scale -scaling factor
   * @return pcl::PointCloud<PointT>::Ptr - Smart pointer of output point cloud
   */
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr ScalePointCloud(
      typename pcl::PointCloud<PointT>::Ptr cloud_in_ptr, double scale) {
    typedef typename pcl::PointCloud<PointT>::Ptr point_t_pc_ptr;
    typedef typename pcl::PointCloud<PointT> point_t_pc;

    point_t_pc_ptr transformed_ptr(new point_t_pc);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = scale;
    transform(1, 1) = scale;
    transform(2, 2) = scale;
    pcl::transformPointCloud(*cloud_in_ptr, *transformed_ptr, transform);

    return transformed_ptr;
  }

  /**
   * @brief Perform post processing of a point cloud before publishing
   *
   * @tparam PointT - Type of pcl point
   * @param pc_ptr - Point to pcl point cloud to be published
   * @param image_array_ptr - Original image array message that has the images from which the point
   * cloud was triangulated
   **/
  template <typename PointT>
  void PostProcessAndPublishPointCloud(
      typename pcl::PointCloud<PointT>::Ptr pc_ptr,
      const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr) {
    // Apply box filter if specified (point cloud will no longer be organised!)
    if (apply_crop_box_) {
      ApplyCropBox<PointT>(pc_ptr);
    }

    // Transform to the coordinate frame of the specified camera, if specified
    if (frame_camera_provided_) {
      pcl::transformPointCloud(*pc_ptr, *pc_ptr, transform_sensor_tri_);
    }

    // Scale point cloud if desired
    if (apply_scaling_) {
      pc_ptr = ScalePointCloud<PointT>(pc_ptr, scaling_factor_);
    }

    // Publish point cloud
    PublishPointCloud<PointT>(pc_ptr, image_array_ptr);
  }
};

}  // namespace reconstruction
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::reconstruction::TriangulatorNodelet, nodelet::Nodelet);

#endif  // SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_NODELET_HPP_