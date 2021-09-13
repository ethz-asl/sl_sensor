#pragma once

#include <memory>
#include "sl_sensor_registration/libpointmatcher_icp.hpp"
#include "sl_sensor_registration/o3d_color_icp.hpp"
#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"

namespace sl_sensor
{
namespace registration
{
class PointCloudRegistrationAlgorithmFactory
{
public:
  static std::unique_ptr<PointCloudRegistrationAlgorithm> GetInstance(const std::string &algo_name)
  {
    std::unique_ptr<PointCloudRegistrationAlgorithm> output_ptr;

    if (algo_name == "lpm_icp")
    {
      output_ptr = std::make_unique<LibpointmatcherICP>();
    }
    else if (algo_name == "o3d_color_icp")
    {
      output_ptr = std::make_unique<O3dColorIcp>();
    }
    else
    {
      std::cout << "Invalid Pc registration algorithm name!" << std::endl;
      throw;
    }

    return std::move(output_ptr);
  };
};
}  // namespace registration

}  // namespace sl_sensor