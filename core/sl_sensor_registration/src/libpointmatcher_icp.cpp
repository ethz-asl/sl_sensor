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

#include "sl_sensor_registration/libpointmatcher_icp.hpp"
#include "sl_sensor_registration/pc_utilities.hpp"

namespace sl_sensor {
namespace registration {
LibpointmatcherICP::LibpointmatcherICP() { settings_ = default_settings_; };

LibpointmatcherICP::DP LibpointmatcherICP::PCLToLPBPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& pcl_pc) {
  sensor_msgs::PointCloud2 ros_pc;
  pcl::toROSMsg(pcl_pc, ros_pc);
  return PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ros_pc);
}

LibpointmatcherICP::PM::TransformationParameters LibpointmatcherICP::EigenToTransformationParameter(
    const Eigen::Matrix4f& fs_matrix) {
  LibpointmatcherICP::PM::TransformationParameters output =
      LibpointmatcherICP::PM::TransformationParameters::Identity(fs_matrix.rows(),
                                                                 fs_matrix.cols());

  for (int r = 0; r < fs_matrix.rows(); r++) {
    for (int c = 0; c < fs_matrix.cols(); c++) {
      output(r, c) = fs_matrix(r, c);
    }
  }

  return output;
}

Eigen::Matrix4f LibpointmatcherICP::TransformationParameterToEigen(
    const LibpointmatcherICP::PM::TransformationParameters& tp) {
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();

  for (int r = 0; r < output.rows(); r++) {
    for (int c = 0; c < output.cols(); c++) {
      output(r, c) = tp(r, c);
    }
  }

  return output;
}

void LibpointmatcherICP::LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) {
  nh.param<std::string>("yaml_directory", yaml_directory_, "");

  PointCloudRegistrationAlgorithm::LoadSettingsFromROSNodeHandle(nh);
};

void LibpointmatcherICP::Init() {
  if (yaml_directory_.empty()) {
    // See the implementation of setDefault() to create a custom ICP algorithm
    icp_.setDefault();
  } else {
    // load YAML config
    std::ifstream ifs(yaml_directory_.c_str());
    if (!ifs.good()) {
      std::cout << "Cannot find YAML file: " << yaml_directory_ << std::endl;
      throw;
    } else {
      std::cout << "Loaded ICP params from file: " << yaml_directory_ << std::endl;
      icp_.loadFromYaml(ifs);
    }
  }
}

void LibpointmatcherICP::RegisterPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_ptr, const Eigen::Matrix4f& guess) {
  curr_cloud_ptr_.reset();
  curr_cloud_ptr_ = std::make_shared<LibpointmatcherICP::DP>();
  *curr_cloud_ptr_ = PCLToLPBPointCloud(
      *GetSubsampledPointCloud<pcl::PointXYZ>(reference_pc_ptr, settings_.at("leafsi"
                                                                             "z"
                                                                             "e")));

  if (!received_first_point_cloud_) {
    received_first_point_cloud_ = true;
  } else {
    LibpointmatcherICP::PM::TransformationParameters tp_init =
        EigenToTransformationParameter(guess);
    LibpointmatcherICP::PM::TransformationParameters tp_result =
        icp_.compute(*curr_cloud_ptr_, *prev_cloud_ptr_, tp_init);

    // Output Transformation is T[prev->curr]
    relative_transform_ = TransformationParameterToEigen(tp_result);
    // T[0->curr] = T[0->prev] * T[prev->curr]
    absolute_transform_ = absolute_transform_ * relative_transform_;
  }

  std::swap(prev_cloud_ptr_, curr_cloud_ptr_);
};

}  // namespace registration
}  // namespace sl_sensor