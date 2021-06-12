#include "sl_sensor_reconstruction/triangulator_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sl_sensor_calibration/calibration_data.hpp>
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
  private_nh_.param<std::string>("calibration_file", calibration_filename_, calibration_filename_);
  private_nh_.param<int>("number_cameras", number_cameras_, number_cameras_);

  // Load Calibration Data
  CalibrationData calibration_data;
  if (calibration_data.Load(calibration_filename_))
  {
    ROS_INFO("[TriangulatorNodelet] Calibration data loaded successfully.");
  }
  else
  {
    ROS_ERROR("[TriangulatorNodelet] Failed to load calibration data!");
  }

  // Setup Triangulator
  triangulator_ptr_ = std::make_unique<Triangulator>(calibration_data);

  // Setup publisher and subscriber
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &TriangulatorNodelet::ImageArrayCb, this);
};

void TriangulatorNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr &image_array_ptr)
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
  auto pc_ptr = triangulator_ptr_->Triangulate(cv_img_ptr_vec[0]->image, cv_img_ptr_vec[1]->image,
                                               cv_img_ptr_vec[2]->image, cv_img_ptr_vec[3]->image);

  // Publish point cloud
  sensor_msgs::PointCloud2Ptr pc_msg_ptr;
  pcl::toROSMsg(*pc_ptr, *pc_msg_ptr);
  pc_msg_ptr->header.frame_id = image_array_ptr->header.frame_id;
  pc_msg_ptr->header.stamp = image_array_ptr->header.stamp;
  pc_pub_.publish(pc_msg_ptr);
};

}  // namespace reconstruction
}  // namespace sl_sensor
