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

#include "sl_sensor_codec/two_plus_one_with_tpu_decoder.hpp"
#include "sl_sensor_codec/phase_shift_utilities.hpp"

#include <math.h>

namespace sl_sensor {
namespace codec {
// Decoder
TwoPlusOneWithTpuDecoder::TwoPlusOneWithTpuDecoder(unsigned int screen_cols,
                                                   unsigned int screen_rows, CodecDirection dir,
                                                   unsigned int number_phases_horizontal,
                                                   unsigned int number_phases_vertical)
    : Decoder(screen_cols, screen_rows, dir),
      number_phases_horizontal_(number_phases_horizontal),
      number_phases_vertical_(number_phases_vertical) {
  number_patterns_ = (dir == CodecDirection::kBoth) ? 10 : 5;
  frames_.resize(number_patterns_,
                 cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

TwoPlusOneWithTpuDecoder::TwoPlusOneWithTpuDecoder(const YAML::Node& node) : Decoder(node) {
  number_phases_horizontal_ = (node["number_phases_horizontal"])
                                  ? node["number_phases_horizontal"].as<int>()
                                  : number_phases_horizontal_;
  number_phases_vertical_ = (node["number_phases_vertical"])
                                ? node["number_phases_vertical"].as<int>()
                                : number_phases_vertical_;
  shading_threshold_ =
      (node["shading_threshold"]) ? node["shading_threshold"].as<double>() : shading_threshold_;
  average_intensity_ =
      (node["average_intensity"]) ? node["average_intensity"].as<double>() : average_intensity_;

  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 10 : 5;
  frames_.resize(number_patterns_,
                 cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

void TwoPlusOneWithTpuDecoder::DecodeFrames(cv::Mat& up, cv::Mat& vp, cv::Mat& mask,
                                            cv::Mat& shading) {
  // Set default output formats
  up = cv::Mat(0, 0, CV_32FC1);
  vp = cv::Mat(0, 0, CV_32FC1);
  mask = cv::Mat(0, 0, CV_8UC1);
  shading = cv::Mat(0, 0, CV_8UC1);

  if (direction_ == CodecDirection::kHorizontal) {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]),
                 cv::Mat_<float>(frames_[2]), cv::Mat_<float>(frames_[3]),
                 cv::Mat_<float>(frames_[4]), screen_cols_, number_phases_horizontal_, up);
  } else if (direction_ == CodecDirection::kVertical) {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]),
                 cv::Mat_<float>(frames_[2]), cv::Mat_<float>(frames_[3]),
                 cv::Mat_<float>(frames_[4]), screen_rows_, number_phases_vertical_, vp);
  } else {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]),
                 cv::Mat_<float>(frames_[2]), cv::Mat_<float>(frames_[3]),
                 cv::Mat_<float>(frames_[4]), screen_cols_, number_phases_horizontal_, up);
    ComputePhase(cv::Mat_<float>(frames_[5]), cv::Mat_<float>(frames_[6]),
                 cv::Mat_<float>(frames_[7]), cv::Mat_<float>(frames_[8]),
                 cv::Mat_<float>(frames_[9]), screen_rows_, number_phases_vertical_, vp);
  }

  // Calculate modulation
  shading = (1.0f / average_intensity_) * frames_[1];

  // Generate shading
  mask = shading > shading_threshold_;
}

void TwoPlusOneWithTpuDecoder::ComputePhase(const cv::Mat& i_1, const cv::Mat& i_2,
                                            const cv::Mat& i_3, const cv::Mat& i_4,
                                            const cv::Mat& i_5, unsigned int max_pixel,
                                            unsigned int number_phases, cv::Mat& output_phase) {
  cv::phase(i_1 - i_2, i_3 - i_2, output_phase);
  cv::Mat cue;
  cv::phase(i_4 - i_2, i_5 - i_2, cue);
  output_phase = UnwrapWithCue(output_phase, cue, number_phases);
  output_phase *= max_pixel / (2 * M_PI);
}

}  // namespace codec
}  // namespace sl_sensor