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

#include "sl_sensor_codec/decoder.hpp"

#include <tuple>

namespace sl_sensor {
namespace codec {
Decoder::Decoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir)
    : number_patterns_(0), screen_cols_(screen_cols), screen_rows_(screen_rows), direction_(dir) {}

Decoder::Decoder(const YAML::Node& node) { InitFromYAMLNode(node); }

unsigned int Decoder::GetNumberPatterns() const { return number_patterns_; }

CodecDirection Decoder::GetDirection() const { return direction_; }

void Decoder::SetFrames(std::vector<cv::Mat>& frames) { frames_ = std::move(frames); }

void Decoder::InitFromYAMLNode(const YAML::Node& node) {
  std::tuple<unsigned int, unsigned int, CodecDirection> result =
      GetBasicCodecInformationFromYAMLNode(node);
  screen_rows_ = std::get<0>(result);
  screen_cols_ = std::get<1>(result);
  direction_ = std::get<2>(result);
}

void Decoder::SetFrame(const cv::Mat& frame, unsigned int number) { frames_[number] = frame; }

}  // namespace codec
}  // namespace sl_sensor
