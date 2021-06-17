#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace sl_sensor
{
namespace codec
{
enum class CodecDirection
{
  kNone = 0,
  kHorizontal = 1 << 0,
  kVertical = 1 << 1,
  kBoth = kHorizontal | kVertical
};

/**
 * @brief Base class that generates structured light patterns
 *
 */
class Encoder
{
public:
  /**
   * @brief Construct a new Encoder object
   *
   * @param screen_cols - Number of cols in projector image
   * @param screen_rows - Number of rows in projector image
   * @param direction_ - Direction of pattern
   */
  Encoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection direction_ = CodecDirection::kHorizontal);

  /**
   * @brief Constructor using a ROS node handle
   *
   * @param nh
   */
  Encoder(ros::NodeHandle nh);

  /**
   * @brief Get number of patterns in the sequence
   *
   * @return unsigned int
   */
  unsigned int GetNumberPatterns();

  /**
   * @brief Get direction of pattern
   *
   * @return CodecDirection
   */
  CodecDirection GetDirection();

  /**
   * @brief Get a Encoding Pattern with indice depth
   *
   * @param depth
   * @return cv::Mat
   */
  virtual cv::Mat GetEncodingPattern(unsigned int depth) = 0;

  /**
   * @brief Get all patterns in a sequence
   *
   * @return std::vector<cv::Mat>
   */
  std::vector<cv::Mat> GetEncodingPatterns();

  virtual ~Encoder(){};

protected:
  unsigned int number_patterns_ = 0;
  unsigned int screen_cols_, screen_rows_;
  CodecDirection direction_;

  /**
   * @brief Retrieves basic Encoder information using ROS node handls
   *
   * @param nh
   */
  void InitFromRosNodeHandle(ros::NodeHandle nh);
};

/**
 * @brief Base class that performed decoding for images with structured light patterns
 *
 */
class Decoder
{
public:
  /**
   * @brief Construct a new Decoder object
   *
   * @param screen_cols - Number of cols in projector image
   * @param screen_rows - Number of rows in projector image
   * @param direction_ - Direction of pattern
   */
  Decoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir = CodecDirection::kHorizontal);

  /**
   * @brief Constructor using a ROS node handle
   *
   * @param nh
   */
  Decoder(ros::NodeHandle nh);

  /**
   * @brief Get number of patterns in the sequence
   *
   * @return unsigned int
   */
  unsigned int GetNumberPatterns();

  /**
   * @brief Get direction of pattern
   *
   * @return CodecDirection
   */
  CodecDirection GetDirection();

  /**
   * @brief Set an image to be decoded
   *
   * @param frame - image to be decoded
   * @param number - image indice
   */
  void SetFrame(const cv::Mat &frame, unsigned int number);

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
  void InitFromRosNodeHandle(ros::NodeHandle nh);
};

}  // namespace codec
}  // namespace sl_sensor
