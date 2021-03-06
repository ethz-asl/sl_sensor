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

#ifndef SL_SENSOR_CODEC_ENCODER_HPP_
#define SL_SENSOR_CODEC_ENCODER_HPP_

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "sl_sensor_codec/codec_common.hpp"

namespace sl_sensor {
namespace codec {
/**
 * @brief Base class that generates structured light patterns
 *
 */
class Encoder {
 public:
  /**
   * @brief Construct a new Encoder object
   *
   * @param screen_cols - Number of cols in projector image
   * @param screen_rows - Number of rows in projector image
   * @param direction_ - Direction of pattern
   */
  Encoder(unsigned int screen_cols, unsigned int screen_rows,
          CodecDirection direction_ = CodecDirection::kHorizontal);

  Encoder(const YAML::Node &node);

  /**
   * @brief Get number of patterns in the sequence
   *
   * @return unsigned int
   */
  unsigned int GetNumberPatterns() const;

  /**
   * @brief Get direction of pattern
   *
   * @return CodecDirection
   */
  CodecDirection GetDirection() const;

  /**
   * @brief Get a Encoding Pattern with indice depth
   *
   * @param depth
   * @return cv::Mat
   */
  cv::Mat GetEncodingPattern(size_t depth) const;

  /**
   * @brief Get all patterns in a sequence
   *
   * @return std::vector<cv::Mat>
   */
  std::vector<cv::Mat> GetEncodingPatterns() const;

  /**
   * @brief Downsample a pattern if they have a diamond pixel arrangement
   *
   * @param pattern
   * @return cv::Mat
   */
  static cv::Mat DiamondDownsample(const cv::Mat &pattern);

  /**
   * @brief Forward distortion of points. The inverse of the undistortion in
   * cv::initUndistortRectifyMap()
   * @note Inspired by Pascal Thomet, http://code.opencv.org/issues/1387#note-11 Convention for
   * distortion parameters: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
   * @param instrinsic_matrix
   * @param distortion_coefficients
   * @param size
   * @param map1
   * @param map2
   * @return cv::Mat
   */
  static void InitDistortMap(const cv::Matx33f instrinsic_matrix,
                             const cv::Vec<float, 5> distortion_coefficients, const cv::Size size,
                             cv::Mat &map1, cv::Mat &map2);

  virtual ~Encoder(){};

 protected:
  unsigned int number_patterns_ = 0;
  unsigned int screen_cols_, screen_rows_;
  CodecDirection direction_;
  std::vector<cv::Mat> patterns_;

  void InitFromYAMLNode(const YAML::Node &node);
};

}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_ENCODER_HPP_