#include "sl_sensor_logger/point_cloud_logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

using namespace sl_sensor::image_acquisition;

namespace sl_sensor
{
namespace logger
{
PointCloudLoggerNodelet::PointCloudLoggerNodelet(){};

void PointCloudLoggerNodelet::onInit()
{
  // Call Base class onInit()
  LoggerNodelet::onInit();

  // Obtain information from private node handle
  private_nh_.param<std::string>("log_topic", pc_sub_topic_, pc_sub_topic_);
  private_nh_.param<std::string>("log_directory", save_folder_, save_folder_);
  private_nh_.param<std::string>("file_header", header_, header_);

  // Setup publisher and subscriber
  pc_sub_ = nh_.subscribe(pc_sub_topic_, 10, &PointCloudLoggerNodelet::PointCloudCb, this);
};

void PointCloudLoggerNodelet::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr)
{
  if (enabled_)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc_msg_ptr, pcl_pc2);

    std::string array_time = std::to_string(pc_msg_ptr->header.stamp.toNSec());
    std::string final_pc_full_directory =
        save_folder_ + header_ + "_" + array_time + "_" + std::to_string(counter_) + ".pcd";

    pcl::PCDWriter file_writer;
    file_writer.write(final_pc_full_directory, pcl_pc2);

    counter_++;
  }
};

}  // namespace logger
}  // namespace sl_sensor
