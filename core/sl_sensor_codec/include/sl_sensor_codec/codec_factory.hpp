#pramga once

#include <ros/ros.h>
#include <memory>

#include "sl_sensor_codec/phase_shift_with_tpu_codec.hpp"

namespace sl_sensor
{
namespace codec
{
class CodecFactory
{
public:
  static std::unique_ptr<Encoder> GetInstanceEncoder(const std::string& encoder_name, ros::NodeHandle nh)
  {
    std::unique_ptr<Encoder> output_ptr;

    if (encoder_name == "PhaseShiftWithTpu")
    {
      output_ptr = std::make_unique<PhaseShiftWithTpuEncoder>(nh);
    }
    else
    {
      ROS_INFO("[CodecFactory] Invalid Encoder name, returning empty pointer");
    }

    return std::move(output_ptr);
  };

  static std::unique_ptr<Decoder> GetInstanceEncoder(const std::string& decoder_name, ros::NodeHandle nh)
  {
    std::unique_ptr<Decoder> output_ptr;

    if (decoder_name == "PhaseShiftWithTpu")
    {
      output_ptr = std::make_unique<PhaseShiftWithTpuDecoder>(nh);
    }
    else
    {
      ROS_INFO("[CodecFactory] Invalid Decoder name, returning empty pointer");
    }

    return std::move(output_ptr);
  };
};

}  // namespace codec
}  // namespace sl_sensor