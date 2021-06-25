#include "sl_sensor_codec/encoder.hpp"

#include <tuple>

namespace sl_sensor
{
namespace codec
{
Encoder::Encoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection direction_)
  : number_patterns_(0), screen_cols_(screen_cols), screen_rows_(screen_rows), direction_(direction_)
{
}

Encoder::Encoder(ros::NodeHandle nh)
{
  InitFromRosNodeHandle(nh);
}

unsigned int Encoder::GetNumberPatterns()
{
  return number_patterns_;
}

CodecDirection Encoder::GetDirection()
{
  return direction_;
}

std::vector<cv::Mat> Encoder::GetEncodingPatterns()
{
  std::vector<cv::Mat> result = {};

  for (size_t i = 0; i < (size_t)GetNumberPatterns(); i++)
  {
    result.push_back(GetEncodingPattern(i));
  }

  return result;
}

void Encoder::InitFromRosNodeHandle(ros::NodeHandle nh)
{
  std::tuple<int, int, CodecDirection> result = GetBasicCodecInformationFromNodeHandle(nh);
  screen_rows_ = std::get<0>(result);
  screen_cols_ = std::get<1>(result);
  direction_ = std::get<2>(result);
}

}  // namespace codec
}  // namespace sl_sensor
