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

#include "sl_sensor_codec/two_plus_one_with_tpu_encoder.hpp"
#include "sl_sensor_codec/phase_shift_utilities.hpp"

#include <math.h>

namespace sl_sensor {
namespace codec {
// Encoder
TwoPlusOneWithTpuEncoder::TwoPlusOneWithTpuEncoder(unsigned int screen_cols,
                                                   unsigned int screen_rows, CodecDirection dir,
                                                   unsigned int number_phases_horizontal,
                                                   unsigned int number_phases_vertical)
    : Encoder(screen_cols, screen_rows, dir),
      number_phases_horizontal_(number_phases_horizontal),
      number_phases_vertical_(number_phases_vertical) {
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 10 : 5;
  GeneratePatterns();
}

TwoPlusOneWithTpuEncoder::TwoPlusOneWithTpuEncoder(const YAML::Node &node) : Encoder(node) {
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 10 : 5;

  number_phases_horizontal_ = (node["number_phases_horizontal"])
                                  ? node["number_phases_horizontal"].as<int>()
                                  : number_phases_horizontal_;
  number_phases_vertical_ = (node["number_phases_vertical"])
                                ? node["number_phases_vertical"].as<int>()
                                : number_phases_vertical_;
  average_intensity_ =
      (node["average_intensity"]) ? node["average_intensity"].as<double>() : average_intensity_;
  modulation_intensity_ = (node["modulation_intensity"]) ? node["modulation_intensity"].as<double>()
                                                         : modulation_intensity_;

  GeneratePatterns();
}

void TwoPlusOneWithTpuEncoder::GeneratePatterns() {
  // Initialise flat pattern
  cv::Mat flat_pattern(5, 5, CV_8UC3);
  flat_pattern.setTo(average_intensity_ * 255.0);

  if (direction_ == CodecDirection::kHorizontal || direction_ == CodecDirection::kBoth) {
    // Horizontally encoding patterns_
    for (unsigned int i = 0; i < 2; i++) {
      float phase = M_PI / 2.0 * i;
      float pitch = (float)screen_cols_ / (float)number_phases_horizontal_;
      cv::Mat pattern(1, 1, CV_8U);
      pattern =
          ComputePhaseVector(screen_cols_, phase, pitch, average_intensity_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Phase cue patterns_
    for (unsigned int i = 0; i < 2; i++) {
      float phase = M_PI / 2.0 * i;
      float pitch = screen_cols_;
      cv::Mat pattern;
      pattern =
          ComputePhaseVector(screen_cols_, phase, pitch, average_intensity_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Insert flat pattern
    patterns_.insert(patterns_.end() - 3, flat_pattern);
  }

  if (direction_ == CodecDirection::kVertical || direction_ == CodecDirection::kBoth) {
    // Precompute vertically encoding patterns_
    for (unsigned int i = 0; i < 2; i++) {
      float phase = M_PI / 2.0 * i;
      float pitch = (float)screen_rows_ / (float)number_phases_vertical_;
      cv::Mat pattern;
      pattern =
          ComputePhaseVector(screen_rows_, phase, pitch, average_intensity_, modulation_intensity_);
      patterns_.push_back(pattern);
    }

    // Precompute vertically phase cue patterns_
    for (unsigned int i = 0; i < 2; i++) {
      float phase = M_PI / 2.0 * i;
      float pitch = screen_rows_;
      cv::Mat pattern;
      pattern =
          ComputePhaseVector(screen_rows_, phase, pitch, average_intensity_, modulation_intensity_);
      patterns_.push_back(pattern);
    }

    patterns_.insert(patterns_.end() - 3, flat_pattern);
  }
}

}  // namespace codec
}  // namespace sl_sensor