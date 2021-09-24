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

#ifndef SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_
#define SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_

#pragma once

#include <vector>
#include "sl_sensor_codec/decoder.hpp"

namespace sl_sensor {
namespace codec {
/**
 * @brief Decoder for PSP with TPU pattern
 *
 */
class PhaseShiftWithTpuDecoder : public Decoder {
 public:
  PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases_horizontal,
                           unsigned int number_phases_vertical);
  PhaseShiftWithTpuDecoder(const YAML::Node &node);
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) override;

 private:
  unsigned int number_phases_horizontal_ = 1;
  unsigned int number_phases_vertical_ = 1;
  unsigned int shading_threshold_ = 55;
};
}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_