#include "sl_sensor_reconstruction/triangulator_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <sl_sensor_calibration/camera_parameters.hpp>
#include <sl_sensor_calibration/projector_parameters.hpp>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

using namespace sl_sensor::calibration;
using namespace sl_sensor::image_acquisition;

namespace sl_sensor
{
namespace reconstruction
{
TriangulatorNodelet::TriangulatorNodelet(){};

void TriangulatorNodelet::onInit()
{
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", image_array_sub_topic_, image_array_sub_topic_);
  private_nh_.param<std::string>("output_topic", pc_pub_topic_, pc_pub_topic_);
  private_nh_.param<std::string>("camera_parameters_filename", camera_parameters_filename_,
                                 camera_parameters_filename_);
  private_nh_.param<std::string>("projector_parameters_filename", projector_parameters_filename_,
                                 projector_parameters_filename_);
  private_nh_.param<int>("number_cameras", number_cameras_, number_cameras_);
  private_nh_.param<bool>("apply_crop_box", apply_crop_box_, apply_crop_box_);
  private_nh_.param<float>("crop_box_x_min", crop_box_x_min_, crop_box_x_min_);
  private_nh_.param<float>("crop_box_y_min", crop_box_y_min_, crop_box_y_min_);
  private_nh_.param<float>("crop_box_z_min", crop_box_z_min_, crop_box_z_min_);
  private_nh_.param<float>("crop_box_x_max", crop_box_x_max_, crop_box_x_max_);
  private_nh_.param<float>("crop_box_y_max", crop_box_y_max_, crop_box_y_max_);
  private_nh_.param<float>("crop_box_z_max", crop_box_z_max_, crop_box_z_max_);

  // Load Camera Calibration Data
  CameraParameters camera_parameters;
  if (camera_parameters.Load(camera_parameters_filename_))
  {
    ROS_INFO("[TriangulatorNodelet] Camera parameters loaded successfully.");
  }
  else
  {
    ROS_ERROR("[TriangulatorNodelet] Failed to load camera parameters!");
  }

  ROS_INFO("[TriangulatorNodelet] Loaded Camera parameters: ");
  std::cout << camera_parameters << std::endl;

  ProjectorParameters projector_parameters;
  if (projector_parameters.Load(projector_parameters_filename_))
  {
    ROS_INFO("[TriangulatorNodelet] Projector parameters loaded successfully.");
  }
  else
  {
    ROS_ERROR("[TriangulatorNodelet] Failed to load projector parameters!");
  }

  ROS_INFO("[TriangulatorNodelet] Loaded projector parameters: ");
  std::cout << projector_parameters << std::endl;

  // Setup Triangulator
  triangulator_ptr_ = std::make_unique<Triangulator>(projector_parameters, camera_parameters);

  // Setup publisher and subscriber
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &TriangulatorNodelet::ImageArrayCb, this);
};

void TriangulatorNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr)
{
  // Do not continue if image array does not contain the expected number of images
  if (image_array_ptr->data.size() != (unsigned int)(number_cameras_ * 4))
  {
    ROS_INFO("[TriangulatorNodelet] Number of images in image array does not correspond to (number of images required "
             "for "
             "4* number cameras). Ignoring this image array message");
    return;
  }

  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
  ConvertImgArrToCvPtrVec(image_array_ptr, cv_img_ptr_vec);

  // Triangulate (currently only supports one camera for now)
  auto pc_ptr = triangulator_ptr_->TriangulateMonochrome(cv_img_ptr_vec[0]->image, cv_img_ptr_vec[1]->image,
                                                         cv_img_ptr_vec[2]->image, cv_img_ptr_vec[3]->image);

  // Apply box filter if specified (point cloud will no longer be organised!)
  if (apply_crop_box_)
  {
    pcl::CropBox<pcl::PointXYZI> crop_box;
    crop_box.setMin(Eigen::Vector4f(crop_box_x_min_, crop_box_y_min_, crop_box_z_min_, 1.0));
    crop_box.setMax(Eigen::Vector4f(crop_box_x_max_, crop_box_y_max_, crop_box_z_max_, 1.0));
    crop_box.setInputCloud(pc_ptr);
    crop_box.filter(*pc_ptr);
  }

  // Publish point cloud
  sensor_msgs::PointCloud2Ptr pc_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*pc_ptr, *pc_msg_ptr);
  pc_msg_ptr->header.frame_id = image_array_ptr->header.frame_id;
  pc_msg_ptr->header.stamp = image_array_ptr->header.stamp;
  pc_pub_.publish(pc_msg_ptr);
};

}  // namespace reconstruction
}  // namespace sl_sensor
