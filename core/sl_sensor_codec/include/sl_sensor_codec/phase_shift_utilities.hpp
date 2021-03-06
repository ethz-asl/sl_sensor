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

#ifndef SL_SENSOR_CODEC_PHASE_SHIFT_UTILITIES_HPP_
#define SL_SENSOR_CODEC_PHASE_SHIFT_UTILITIES_HPP_

#include <opencv2/opencv.hpp>

namespace sl_sensor {
namespace codec {
/**
 * @brief Generates an 8-bit [length x 1] (0-255) matrix using the equation: average_intensity +
 * modulation_intensity * cos(2 * M_PI * i / pitch - phase) where i is the current row indice
 *
 * @param length - length of matrix
 * @param phase - phase offset
 * @param pitch - pitch of cosine wave
 * @param average_intensity - average value (0-1)
 * @param modulation_intensity - modulation vlaue (0-1)
 * @return cv::Mat
 */
cv::Mat ComputePhaseVector(unsigned int length, float phase, float pitch,
                           double average_intensity = 0.5, double modulation_intensity = 0.5);

/**
 * @brief Unwrapped phase computation for N=3 Phase shift profilometry
 *
 * @param I1
 * @param I2
 * @param I3
 * @return cv::Mat
 */
cv::Mat GetPhase(const cv::Mat& I1, const cv::Mat& I2, const cv::Mat& I3);

/**
 * @brief Get the Magnitude (shading) for N=3 Phase shift profilometry
 *
 * @param I1
 * @param I2
 * @param I3
 * @return cv::Mat
 */
cv::Mat GetMagnitude(const cv::Mat& I1, const cv::Mat& I2, const cv::Mat& I3);

/**
 * @brief Perform phase unwrapping
 *
 * @param up - Phase image of high frequency pattern
 * @param up_cue - Cue phase map
 * @param n_phases - Number of periods in high frequency pattern
 * @return cv::Mat - Unwrapped phase map
 */
cv::Mat UnwrapWithCue(const cv::Mat& up, const cv::Mat& up_cue, unsigned int n_phases);

}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_PHASE_SHIFT_UTILITIES_HPP_
