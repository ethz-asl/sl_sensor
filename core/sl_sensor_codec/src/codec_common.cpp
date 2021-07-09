
#include "sl_sensor_codec/codec_common.hpp"

namespace sl_sensor
{
namespace codec
{
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