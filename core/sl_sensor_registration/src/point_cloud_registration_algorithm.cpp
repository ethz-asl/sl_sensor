#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"

namespace sl_sensor {
namespace registration {
void PointCloudRegistrationAlgorithm::LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) {
  for (auto& [setting_name, value] : settings_) {
    auto default_value = value;
    nh.param<double>(setting_name, settings_[setting_name], default_value);
  }
};

void PointCloudRegistrationAlgorithm::RegisterPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_pc, const Eigen::Matrix4f& guess) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*reference_pc, *input_xyz_ptr);
  RegisterPointCloud(input_xyz_ptr, guess);
};

Eigen::Matrix4f PointCloudRegistrationAlgorithm::GetRelativeTransform() {
  return relative_transform_;
};

Eigen::Matrix4f PointCloudRegistrationAlgorithm::GetAbsoluteTransform() {
  return absolute_transform_;
};

void PointCloudRegistrationAlgorithm::SetSetting(const std::string& setting_name, double value) {
  settings_[setting_name] = value;
}

void PointCloudRegistrationAlgorithm::PrintSettings() {
  std::cout << "Settings: " << std::endl;

  for (auto& [setting_name, value] : settings_) {
    std::cout << "\t" << setting_name << ": " << value << std::endl;
  }
};

}  // namespace registration

}  // namespace sl_sensor