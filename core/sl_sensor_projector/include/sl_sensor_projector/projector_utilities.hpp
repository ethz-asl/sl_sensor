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

#ifndef SL_SENSOR_PROJECTOR_PROJECTOR_UTILITIES_HPP_
#define SL_SENSOR_PROJECTOR_PROJECTOR_UTILITIES_HPP_

#include <yaml-cpp/yaml.h>
#include <utility>

namespace sl_sensor {
namespace projector {
/**
 * @brief Get the Projector Resolution from a projector YAML file
 *
 * @param yaml_directory
 * @param height
 * @param width
 * @return true
 * @return false
 */
inline bool GetProjectorResolution(const std::string& yaml_directory, unsigned int& height,
                                   unsigned int& width) {
  bool success = true;

  // Get resolution of projector using YAML file
  YAML::Node proj_config = YAML::LoadFile(yaml_directory);

  if (proj_config["properties"]["resolution"] && proj_config["properties"]["diamond_pixel"]) {
    auto resolution = proj_config["properties"]["resolution"].as<std::vector<unsigned int>>();

    auto is_diamond_pixel = proj_config["properties"]["diamond_pixel"].as<bool>();

    if (is_diamond_pixel) {
      width = resolution[0] * 2;
      height = resolution[1];
    } else {
      width = resolution[0];
      height = resolution[1];
    }
  } else {
    success = false;
  }

  return success;
}

/**
 * @brief Determine if projector has a diamond pixel arrangement
 *
 * @param yaml_directory
 * @return true
 * @return false
 */
inline bool GetIsDiamondPixel(const std::string& yaml_directory) {
  bool result = false;

  // Get resolution of projector using YAML file
  YAML::Node proj_config = YAML::LoadFile(yaml_directory);

  if (proj_config["properties"]["diamond_pixel"]) {
    result = proj_config["properties"]["diamond_pixel"].as<bool>();
  }

  return result;
}

}  // namespace projector
}  // namespace sl_sensor

#endif  // SL_SENSOR_PROJECTOR_PROJECTOR_UTILITIES_HPP_