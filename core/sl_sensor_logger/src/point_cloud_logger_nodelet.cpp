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

#include "sl_sensor_logger/point_cloud_logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <algorithm>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

using namespace sl_sensor::image_acquisition;

namespace sl_sensor {
namespace logger {
PointCloudLoggerNodelet::PointCloudLoggerNodelet(){};

void PointCloudLoggerNodelet::onInit() {
  // Call Base class onInit()
  LoggerNodelet::onInit();

  // Obtain information from private node handle
  private_nh_.param<std::string>("log_topic", pc_sub_topic_, pc_sub_topic_);
  private_nh_.param<std::string>("log_directory", save_folder_, save_folder_);
  private_nh_.param<std::string>("file_header", header_, header_);
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, base_frame_id_);
  private_nh_.param<bool>("include_timestamp", include_timestamp_, include_timestamp_);

  // Setup publisher and subscriber
  pc_sub_ = nh_.subscribe(pc_sub_topic_, 10, &PointCloudLoggerNodelet::PointCloudCb, this);

  // Setup tf listener if base_frame_id is specified
  if (!base_frame_id_.empty()) {
    tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
  }
};

void PointCloudLoggerNodelet::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr) {
  if (enabled_) {
    pcl::PCLPointCloud2 pcl_pc2;

    // If specified, we transform the point cloud to base_frame_id_ before saving
    if (tf_listener_ptr_) {
      sensor_msgs::PointCloud2 transformed_cloud;
      pcl_ros::transformPointCloud(base_frame_id_, *pc_msg_ptr, transformed_cloud,
                                   *tf_listener_ptr_);
      pcl_conversions::toPCL(transformed_cloud, pcl_pc2);
    } else {
      pcl_conversions::toPCL(*pc_msg_ptr, pcl_pc2);
    }

    std::string array_time = std::to_string(pc_msg_ptr->header.stamp.toNSec());
    std::string final_pc_full_directory =
        (include_timestamp_)
            ? (save_folder_ + header_ + "_" + array_time + "_" + std::to_string(counter_) + ".pcd")
            : (save_folder_ + header_ + "_" + std::to_string(counter_) + ".pcd");

    pcl::PCDWriter file_writer;
    file_writer.write(final_pc_full_directory, pcl_pc2);

    counter_++;
  }
};

}  // namespace logger
}  // namespace sl_sensor
