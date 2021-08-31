#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sl_sensor_codec/decoder.hpp"
#include "sl_sensor_codec/encoder.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_decoder.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_encoder.hpp"
#include "sl_sensor_codec/two_plus_one_with_tpu_decoder.hpp"
#include "sl_sensor_codec/two_plus_one_with_tpu_encoder.hpp"

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
  static std::unique_ptr<Encoder> GetInstanceEncoder(const std::string& encoder_name, const YAML::Node& node)
  {
    std::unique_ptr<Encoder> output_ptr;

    if (encoder_name == codec_names_[0])
    {
      // Note the pattern used for calibration is the Phase Shift with TPU pattern so we initialise it when decoder_name
      // = calibration
      output_ptr = std::make_unique<PhaseShiftWithTpuEncoder>(node);
    }
    else if (encoder_name == codec_names_[1])
    {
      output_ptr = std::make_unique<PhaseShiftWithTpuEncoder>(node);
    }
    else if (encoder_name == codec_names_[2])
    {
      output_ptr = std::make_unique<TwoPlusOneWithTpuEncoder>(node);
    }
    else
    {
      std::cout << "[CodecFactory] Invalid Encoder name, returning empty pointer" << std::endl;
    }

    return std::move(output_ptr);
  };

  static std::unique_ptr<Decoder> GetInstanceDecoder(const std::string& decoder_name, const YAML::Node& node)
  {
    std::unique_ptr<Decoder> output_ptr;

    if (decoder_name == codec_names_[0])
    {
      // Note the pattern used for calibration is the Phase Shift with TPU pattern so we initialise it when decoder_name
      // = calibration
      output_ptr = std::make_unique<PhaseShiftWithTpuDecoder>(node);
    }
    else if (decoder_name == codec_names_[1])
    {
      output_ptr = std::make_unique<PhaseShiftWithTpuDecoder>(node);
    }
    else if (decoder_name == codec_names_[2])
    {
      output_ptr = std::make_unique<TwoPlusOneWithTpuDecoder>(node);
    }
    else
    {
      std::cout << "[CodecFactory] Invalid Decoder name, returning empty pointer" << std::endl;
    }

    return std::move(output_ptr);
  };

  static std::vector<std::string> GetAllCodecNames()
  {
    return codec_names_;
  }

private:
  static const std::vector<std::string> codec_names_;
};

const std::vector<std::string> CodecFactory::codec_names_ = { "calibration", "phase_shift_with_tpu",
                                                              "two_plus_one_with_tpu" };

}  // namespace codec
}  // namespace sl_sensor