// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#include "sl_sensor_codec/encoder.hpp"

#include <tuple>

namespace sl_sensor {
namespace codec {
Encoder::Encoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection direction_)
    : number_patterns_(0),
      screen_cols_(screen_cols),
      screen_rows_(screen_rows),
      direction_(direction_) {}

Encoder::Encoder(const YAML::Node &node)

{
  InitFromYAMLNode(node);
}

unsigned int Encoder::GetNumberPatterns() const { return number_patterns_; }

CodecDirection Encoder::GetDirection() const { return direction_; }

cv::Mat Encoder::GetEncodingPattern(size_t depth) const { return patterns_[depth]; }

std::vector<cv::Mat> Encoder::GetEncodingPatterns() const {
  std::vector<cv::Mat> result = {};

  for (size_t i = 0; i < (size_t)GetNumberPatterns(); i++) {
    result.push_back(GetEncodingPattern(i));
  }

  return result;
}

void Encoder::InitFromYAMLNode(const YAML::Node &node) {
  std::tuple<unsigned int, unsigned int, CodecDirection> result =
      GetBasicCodecInformationFromYAMLNode(node);
  screen_rows_ = std::get<0>(result);
  screen_cols_ = std::get<1>(result);
  direction_ = std::get<2>(result);
}

cv::Mat Encoder::DiamondDownsample(const cv::Mat &pattern) {
  cv::Mat pattern_diamond(pattern.rows, pattern.cols / 2, CV_8UC3);

  for (int col = 0; col < pattern_diamond.cols; col++) {
    for (int row = 0; row < pattern_diamond.rows; row++) {
      pattern_diamond.at<cv::Vec3b>(row, col) = pattern.at<cv::Vec3b>(row, col * 2 + row % 2);
    }
  }

  return pattern_diamond;
}

void Encoder::InitDistortMap(const cv::Matx33f instrinsic_matrix,
                             const cv::Vec<float, 5> distortion_coefficients, const cv::Size size,
                             cv::Mat &map1, cv::Mat &map2) {
  float fx = instrinsic_matrix(0, 0);
  float fy = instrinsic_matrix(1, 1);
  float ux = instrinsic_matrix(0, 2);
  float uy = instrinsic_matrix(1, 2);

  float k1 = distortion_coefficients[0];
  float k2 = distortion_coefficients[1];
  float p1 = distortion_coefficients[2];
  float p2 = distortion_coefficients[3];
  float k3 = distortion_coefficients[4];

  map1.create(size, CV_32F);
  map2.create(size, CV_32F);

  for (int col = 0; col < size.width; col++) {
    for (int row = 0; row < size.height; row++) {
      // move origo to principal point and convert using focal length
      float x = (col - ux) / fx;
      float y = (row - uy) / fy;

      float x_corrected, y_corrected;

      // Step 1 : correct distortion
      float r2 = x * x + y * y;
      // radial
      x_corrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      y_corrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      // tangential
      x_corrected = x_corrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
      y_corrected = y_corrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);

      // convert back to pixel coordinates
      float col_displaced = x_corrected * fx + ux;
      float row_displaced = y_corrected * fy + uy;

      // correct the vector in the opposite direction
      map1.at<float>(row, col) = col + (col - col_displaced);
      map2.at<float>(row, col) = row + (row - row_displaced);
    }
  }
}

}  // namespace codec
}  // namespace sl_sensor
