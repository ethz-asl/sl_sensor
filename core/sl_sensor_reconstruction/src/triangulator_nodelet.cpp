#include "sl_sensor_reconstruction/triangulator_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <sl_sensor_calibration/camera_parameters.hpp>
#include <sl_sensor_calibration/projector_parameters.hpp>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

#include <pcl/visualization/cloud_viewer.h>  // for debugging

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
  private_nh_.param<std::string>("primary_camera_parameters_filename", primary_camera_parameters_filename_,
                                 primary_camera_parameters_filename_);
  private_nh_.param<std::string>("secondary_camera_parameters_filename", secondary_camera_parameters_filename_,
                                 secondary_camera_parameters_filename_);
  private_nh_.param<std::string>("projector_parameters_filename", projector_parameters_filename_,
                                 projector_parameters_filename_);
  private_nh_.param<bool>("apply_crop_box", apply_crop_box_, apply_crop_box_);
  private_nh_.param<float>("crop_box_x_min", crop_box_x_min_, crop_box_x_min_);
  private_nh_.param<float>("crop_box_y_min", crop_box_y_min_, crop_box_y_min_);
  private_nh_.param<float>("crop_box_z_min", crop_box_z_min_, crop_box_z_min_);
  private_nh_.param<float>("crop_box_x_max", crop_box_x_max_, crop_box_x_max_);
  private_nh_.param<float>("crop_box_y_max", crop_box_y_max_, crop_box_y_max_);
  private_nh_.param<float>("crop_box_z_max", crop_box_z_max_, crop_box_z_max_);

  private_nh_.param<bool>("colour_shading_enabled", colour_shading_enabled_, colour_shading_enabled_);

  // Load Camera Calibration Data
  CameraParameters primary_camera_parameters;
  if (primary_camera_parameters.Load(primary_camera_parameters_filename_))
  {
    ROS_INFO("[TriangulatorNodelet] Primary camera parameters loaded successfully.");
  }
  else
  {
    ROS_ERROR("[TriangulatorNodelet] Failed to load primary camera parameters!");
  }

  ROS_INFO("[TriangulatorNodelet] Loaded primary camera parameters: ");
  std::cout << primary_camera_parameters << std::endl;

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

  CameraParameters secondary_camera_parameters;
  if (colour_shading_enabled_)
  {
    if (secondary_camera_parameters.Load(secondary_camera_parameters_filename_))
    {
      ROS_INFO("[TriangulatorNodelet] Secondary camera parameters loaded successfully.");
    }
    else
    {
      ROS_ERROR("[TriangulatorNodelet] Failed to load secondary camera parameters!");
    }

    ROS_INFO("[TriangulatorNodelet] Loaded secondary camera parameters: ");
    std::cout << secondary_camera_parameters << std::endl;
  }

  // Setup Triangulator
  if (colour_shading_enabled_)
  {
    triangulator_ptr_ =
        std::make_unique<Triangulator>(projector_parameters, primary_camera_parameters, secondary_camera_parameters);
  }
  else
  {
    triangulator_ptr_ = std::make_unique<Triangulator>(projector_parameters, primary_camera_parameters);
  }

  // Setup publisher and subscriber
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &TriangulatorNodelet::ImageArrayCb, this);
};

void TriangulatorNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr)
{
  // If camera-projector setup, we expect 4 images. If there is a secondary camera providing colour shading, we expect 5
  // images
  size_t expected_images = (colour_shading_enabled_) ? 5 : 4;
  if (expected_images != image_array_ptr->data.size())
  {
    ROS_INFO("[TriangulatorNodelet] Number of images in image array does not match requirements for triangulator. "
             "Ignoring "
             "this "
             "image array message");
    return;
  }

  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
  ConvertImgArrToCvPtrVec(image_array_ptr, cv_img_ptr_vec);

  if (colour_shading_enabled_)
  {
    // Triangulate (coloured shading image should be contained as the last image in the image array message)
    auto pc_ptr = triangulator_ptr_->TriangulateColour(cv_img_ptr_vec.at(0)->image, cv_img_ptr_vec.at(1)->image,
                                                       cv_img_ptr_vec.at(2)->image, cv_img_ptr_vec.at(4)->image);

    // Apply box filter if specified (point cloud will no longer be organised!)
    if (apply_crop_box_)
    {
      ApplyCropBox<pcl::PointXYZRGB>(pc_ptr);
    }

    pcl::PCDWriter file_writer;
    file_writer.write("/home/ltf/test.pcd,", *pc_ptr);

    // Publish point cloud
    PublishPointCloud<pcl::PointXYZRGB>(pc_ptr, image_array_ptr);
  }
  else
  {
    // Triangulate (currently only supports one camera for now)
    auto pc_ptr = triangulator_ptr_->TriangulateMonochrome(cv_img_ptr_vec.at(0)->image, cv_img_ptr_vec.at(1)->image,
                                                           cv_img_ptr_vec.at(2)->image, cv_img_ptr_vec.at(3)->image);

    // Apply box filter if specified (point cloud will no longer be organised!)
    if (apply_crop_box_)
    {
      ApplyCropBox<pcl::PointXYZI>(pc_ptr);
    }

    // Publish point cloud
    PublishPointCloud<pcl::PointXYZI>(pc_ptr, image_array_ptr);
  }
}

}  // namespace reconstruction
}  // namespace sl_sensor
