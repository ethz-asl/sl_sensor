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

class Encoder
{
public:
  Encoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection direction_ = CodecDirection::kHorizontal);

  Encoder(ros::NodeHandle nh);

  unsigned int GetNumberPatterns();

  CodecDirection GetDirection();

  virtual cv::Mat GetEncodingPattern(unsigned int depth) = 0;

  std::vector<cv::Mat> GetEncodingPatterns();

  virtual ~Encoder(){};

protected:
  unsigned int number_patterns_ = 0;
  unsigned int screen_cols_, screen_rows_;
  CodecDirection direction_;
  void InitFromRosNodeHandle(ros::NodeHandle nh);
};

class Decoder
{
public:
  Decoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir = CodecDirection::kHorizontal);

  Decoder(ros::NodeHandle nh);

  unsigned int GetNumberPatterns();

  CodecDirection GetPatternDirection();

  void SetFrames(std::vector<cv::Mat> &&frames);

  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) = 0;

  virtual ~Decoder(){};

protected:
  unsigned int number_patterns_ = 0;
  unsigned int screen_cols_, screen_rows_;
  std::vector<cv::Mat> frames_;
  CodecDirection direction_;

  void InitFromRosNodeHandle(ros::NodeHandle nh);
};

}  // namespace codec
}  // namespace sl_sensor
