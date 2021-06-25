#pragma once

#include <ros/ros.h>
#include <memory>

#include "sl_sensor_codec/decoder.hpp"
#include "sl_sensor_codec/encoder.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_decoder.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_encoder.hpp"

namespace sl_sensor
{
namespace codec
{
/**
 * @brief Factory class that generates Encoders and Decoders
 *
 */
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

  static std::unique_ptr<Decoder> GetInstanceDecoder(const std::string& decoder_name, ros::NodeHandle nh)
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