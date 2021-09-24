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

// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CODEC_CODEC_COMMON_HPP_
#define SL_SENSOR_CODEC_CODEC_COMMON_HPP_

#include <yaml-cpp/yaml.h>

namespace sl_sensor {
namespace codec {
enum class CodecDirection {
  kNone = 0,
  kHorizontal = 1 << 0,
  kVertical = 1 << 1,
  kBoth = kHorizontal | kVertical
};

std::tuple<unsigned int, unsigned int, CodecDirection> GetBasicCodecInformationFromYAMLNode(
    const YAML::Node& node);

}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_CODEC_COMMON_HPP_