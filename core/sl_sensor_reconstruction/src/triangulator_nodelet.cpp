#include "sl_sensor_reconstruction/triangulator_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <opencv2/core/eigen.hpp>
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
  private_nh_.param<std::string>("triangulation_camera_parameters_filename", triangulation_camera_parameters_filename_,
                                 triangulation_camera_parameters_filename_);
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
  private_nh_.param<std::string>("colour_camera_parameters_filename", colour_camera_parameters_filename_,
                                 colour_camera_parameters_filename_);

  private_nh_.param<std::string>("frame_camera_parameters_filename", frame_camera_parameters_filename_,
                                 frame_camera_parameters_filename_);

  private_nh_.param<bool>("apply_scaling", apply_scaling_, apply_scaling_);
  private_nh_.param<float>("scaling_factor", scaling_factor_, scaling_factor_);

  // Load camera and projector parameters
  CameraParameters triangulation_camera_parameters;
  if (triangulation_camera_parameters.Load(triangulation_camera_parameters_filename_))
  {
    ROS_INFO("[TriangulatorNodelet] Primary camera parameters loaded successfully.");
  }
  else
  {
    ROS_ERROR("[TriangulatorNodelet] Failed to load triangulation camera parameters!");
  }

  ROS_INFO("[TriangulatorNodelet] Loaded triangulation camera parameters: ");
  std::cout << triangulation_camera_parameters << std::endl;

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

  CameraParameters colour_camera_parameters;
  if (colour_shading_enabled_)
  {
    if (colour_camera_parameters.Load(colour_camera_parameters_filename_))
    {
      ROS_INFO("[TriangulatorNodelet] Colour camera parameters loaded successfully.");
    }
    else
    {
      ROS_ERROR("[TriangulatorNodelet] Failed to load colour camera parameters!");
    }

    ROS_INFO("[TriangulatorNodelet] Loaded colour camera parameters: ");
    std::cout << colour_camera_parameters << std::endl;
  }

  // Setup Triangulator
  if (colour_shading_enabled_)
  {
    triangulator_ptr_ =
        std::make_unique<Triangulator>(projector_parameters, triangulation_camera_parameters, colour_camera_parameters);
  }
  else
  {
    triangulator_ptr_ = std::make_unique<Triangulator>(projector_parameters, triangulation_camera_parameters);
  }

  CameraParameters frame_camera_parameters;  // This contains transformation information between the projector and the
                                             // camera in which you want to set as the sensor frame
  if (!frame_camera_parameters_filename_.empty())
  {
    if (frame_camera_parameters.Load(frame_camera_parameters_filename_))
    {
      // Compute transform between camera that points are triangulated in wrt sensor frame
      cv::Mat cv_mat_transform_frame_tri = frame_camera_parameters.GetInverseTransformationMatrix() *
                                           triangulation_camera_parameters.GetTransformationMatrix();

      // Store as eigen matrix for use during callback
      cv::cv2eigen(cv_mat_transform_frame_tri, transform_sensor_tri_);

      frame_camera_provided_ = true;

      ROS_INFO("[TriangulatorNodelet] Loaded frame camera parameters: ");
      std::cout << frame_camera_parameters << std::endl;
    }
    else
    {
      ROS_ERROR("[TriangulatorNodelet] Failed to load frame camera parameters!");
    }
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
             "Ignoring this image array message");
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

    // Additional processing and then publish point cloud
    PostProcessAndPublishPointCloud<pcl::PointXYZRGB>(pc_ptr, image_array_ptr);
  }
  else
  {
    // Triangulate (currently only supports one camera for now)
    auto pc_ptr = triangulator_ptr_->TriangulateMonochrome(cv_img_ptr_vec.at(0)->image, cv_img_ptr_vec.at(1)->image,
                                                           cv_img_ptr_vec.at(2)->image, cv_img_ptr_vec.at(3)->image);

    // Additional processing and then publish point cloud
    PostProcessAndPublishPointCloud<pcl::PointXYZI>(pc_ptr, image_array_ptr);
  }
}

}  // namespace reconstruction
}  // namespace sl_sensor
