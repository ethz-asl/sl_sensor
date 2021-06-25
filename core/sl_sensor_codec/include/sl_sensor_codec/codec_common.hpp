#pragma once

#include <ros/ros.h>

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

std::tuple<int, int, CodecDirection> GetBasicCodecInformationFromNodeHandle(ros::NodeHandle nh);

}  // namespace codec
}  // namespace sl_sensor