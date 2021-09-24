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

#include "sl_sensor_codec/codec_common.hpp"

#include <sl_sensor_projector/projector_utilities.hpp>

namespace sl_sensor {
namespace codec {
std::tuple<unsigned int, unsigned int, CodecDirection> GetBasicCodecInformationFromYAMLNode(
    const YAML::Node& node) {
  unsigned int screen_rows = 0;
  unsigned int screen_cols = 0;

  std::string direction_str = "";
  std::string projector_yaml_directory = "";

  CodecDirection direction = CodecDirection::kHorizontal;

  projector_yaml_directory = (node["projector_yaml_directory"])
                                 ? node["projector_yaml_directory"].as<std::string>()
                                 : projector_yaml_directory;
  direction_str = (node["direction"]) ? node["direction"].as<std::string>() : direction_str;

  if (direction_str == "horizontal") {
    direction = CodecDirection::kHorizontal;
  } else if (direction_str == "vertical") {
    direction = CodecDirection::kVertical;
  } else if (direction_str == "both") {
    direction = CodecDirection::kBoth;
  } else {
    direction = CodecDirection::kHorizontal;
  }

  sl_sensor::projector::GetProjectorResolution(projector_yaml_directory, screen_rows, screen_cols);

  return std::make_tuple(screen_rows, screen_cols, direction);
}

}  // namespace codec
}  // namespace sl_sensor