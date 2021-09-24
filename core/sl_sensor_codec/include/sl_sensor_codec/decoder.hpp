// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CODEC_DECODER_HPP_
#define SL_SENSOR_CODEC_DECODER_HPP_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "sl_sensor_codec/codec_common.hpp"

namespace sl_sensor {
namespace codec {
/**
 * @brief Base class that performed decoding for images with structured light patterns
 *
 */
class Decoder {
 public:
  /**
   * @brief Construct a new Decoder object
   *
   * @param screen_cols - Number of cols in projector image
   * @param screen_rows - Number of rows in projector image
   * @param direction_ - Direction of pattern
   */
  Decoder(unsigned int screen_cols, unsigned int screen_rows,
          CodecDirection dir = CodecDirection::kHorizontal);

  Decoder(const YAML::Node &node);

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
   * @brief Set an image to be decoded
   *
   * @param frame - image to be decoded
   * @param number - image indice
   */
  void SetFrame(const cv::Mat &frame, unsigned int number);

  /**
   * @brief Set images to be decoded
   *
   * @param frames - images to be decoded
   */
  void SetFrames(std::vector<cv::Mat> &frames);

  /**
   * @brief Decode Frames
   *
   * @param up - Matrix of horizontal projector coordinates
   * @param vp  - Matrix of vertical projector coordinates
   * @param mask - Mask
   * @param shading - Shading of each pixel
   */
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) = 0;

  virtual ~Decoder(){};

 protected:
  unsigned int number_patterns_ = 0;
  unsigned int screen_cols_, screen_rows_;
  std::vector<cv::Mat> frames_;
  CodecDirection direction_;

  /**
   * @brief Retrieves basic Decoder information using ROS node handls
   *
   * @param nh
   */
  void InitFromYAMLNode(const YAML::Node &node);
};

}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_DECODER_HPP_