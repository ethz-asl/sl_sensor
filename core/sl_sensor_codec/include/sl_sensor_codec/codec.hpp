#pragma once

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
  kBoth = CodecDirHorizontal | CodecDirVertical
}

class Encoder
{
public:
  Encoder(unsigned int screen_cols, unsigned int screen_rows,
          CodecDirection direction_ = CodecDirection::CodecDirHorizontal)
    : number_patterns_(0), screen_cols_(screen_cols), screen_rows_(screen_rows), direction_(direction_)
  {
  }
  unsigned int GetNumberPatterns()
  {
    return number_patterns_;
  }
  CodecDirection GetDirection()
  {
    return direction_;
  }
  virtual cv::Mat GetEncodingPattern(unsigned int depth) = 0;
  virtual ~Encoder(){};

protected:
  unsigned int number_patterns_;
  unsigned int screen_cols_, screen_rows_;
  CodecDirection direction_;
};

class Decoder
{
public:
  Decoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir = CodecDirection::CodecDirHorizontal)
    : number_patterns_(0), screen_cols_(screen_cols), screen_rows_(screen_rows), direction_(dir)
  {
  }
  unsigned int GetNumberPatterns()
  {
    return number_patterns_;
  }
  CodecDirection GetPatternDirection()
  {
    return direction_;
  }
  // Decoding
  virtual void SetFrame(unsigned int depth, const cv::Mat frame) = 0;
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) = 0;
  virtual ~Decoder(){};

protected:
  unsigned int number_patterns_;
  unsigned int screen_cols_, screen_rows_;
  CodecDirection direction_;
};

}  // namespace codec

}  // namespace sl_sensor
