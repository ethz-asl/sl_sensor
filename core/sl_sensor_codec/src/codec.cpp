#include "sl_sensor_codec/codec.hpp"

#include <tuple>

namespace sl_sensor
{
namespace codec
{
std::tuple<int, int, CodecDirection> GetBasicCodecInformationFromNodeHandle(ros::NodeHandle nh);

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

  for (int i = 0; i < (int)GetNumberPatterns(); i++)
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

Decoder::Decoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir)
  : number_patterns_(0), screen_cols_(screen_cols), screen_rows_(screen_rows), direction_(dir)
{
}

Decoder::Decoder(ros::NodeHandle nh)
{
  InitFromRosNodeHandle(nh);
}

unsigned int Decoder::GetNumberPatterns()
{
  return number_patterns_;
}

CodecDirection Decoder::GetPatternDirection()
{
  return direction_;
}

void Decoder::SetFrames(std::vector<cv::Mat> &&frames)
{
  frames_ = frames;
}

void Decoder::InitFromRosNodeHandle(ros::NodeHandle nh)
{
  std::tuple<int, int, CodecDirection> result = GetBasicCodecInformationFromNodeHandle(nh);
  screen_rows_ = std::get<0>(result);
  screen_cols_ = std::get<1>(result);
  direction_ = std::get<2>(result);
}

std::tuple<int, int, CodecDirection> GetBasicCodecInformationFromNodeHandle(ros::NodeHandle nh)
{
  int screen_rows = 0;
  int screen_cols = 0;
  std::string direction_str = "";
  CodecDirection direction = CodecDirection::kHorizontal;

  nh.param<int>("screen_rows", screen_rows, 0);
  nh.param<int>("screen_cols", screen_cols, 0);
  nh.param<std::string>("direction", direction_str, "horizontal");

  if (direction_str == "horizontal")
  {
    direction = CodecDirection::kHorizontal;
  }
  else if (direction_str == "vertical")
  {
    direction = CodecDirection::kVertical;
  }
  else if (direction_str == "both")
  {
    direction = CodecDirection::kBoth;
  }
  else
  {
    ROS_INFO("[Codec] Error parsing error direction, setting to horizontal");
    direction = CodecDirection::kHorizontal;
  }

  return std::make_tuple(screen_rows, screen_cols, direction);
}

}  // namespace codec
}  // namespace sl_sensor
