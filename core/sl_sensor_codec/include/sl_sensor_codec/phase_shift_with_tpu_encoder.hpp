// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_ENCODER_HPP_
#define SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_ENCODER_HPP_

#include <vector>
#include "sl_sensor_codec/encoder.hpp"

namespace sl_sensor {
namespace codec {
/**
 * @brief Encoder for PSP with TPU pattern
 *
 */
class PhaseShiftWithTpuEncoder : public Encoder {
 public:
  PhaseShiftWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases_horizontal,
                           unsigned int number_phases_vertical);
  PhaseShiftWithTpuEncoder(const YAML::Node &node);

 private:
  unsigned int number_phases_horizontal_ = 1;
  unsigned int number_phases_vertical_ = 1;

  double average_intensity_ = 0.5;
  double modulation_intensity_ = 0.5;
  void GeneratePatterns();
};
}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_ENCODER_HPP_