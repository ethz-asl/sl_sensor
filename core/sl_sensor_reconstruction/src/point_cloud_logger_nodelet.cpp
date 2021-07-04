#include "sl_sensor_reconstruction/point_cloud_logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <sl_sensor_calibration/calibration_data.hpp>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

using namespace sl_sensor::calibration;
using namespace sl_sensor::image_acquisition;

namespace sl_sensor
{
namespace reconstruction
{
PointCloudLoggerNodelet::PointCloudLoggerNodelet(){};

void PointCloudLoggerNodelet::onInit()
{
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", pc_sub_topic_, pc_sub_topic_);
  private_nh_.param<std::string>("save_folder", save_folder_, save_folder_);
  private_nh_.param<std::string>("header", header_, header_);

  // Setup publisher and subscriber
  pc_sub_ = nh_.subscribe(pc_sub_topic_, 10, &PointCloudLoggerNodelet::PointCloudCb, this);
};

void PointCloudLoggerNodelet::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pc_msg_ptr, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_ptr);
  std::string final_pc_full_directory = save_folder_ + header_ + "_" + std::to_string(counter_) + ".pcd";
  pcl::io::savePCDFileASCII(final_pc_full_directory, *pc_ptr);

  counter_++;
};

}  // namespace reconstruction
}  // namespace sl_sensor
