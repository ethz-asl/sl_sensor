#pragma once

#include <vector>
#include "sl_sensor_codec/encoder.hpp"

namespace sl_sensor
{
namespace codec
{
/**
 * @brief Encoder for PSP with TPU pattern
 *
 */
class PhaseShiftWithTpuEncoder : public Encoder
{
public:
  PhaseShiftWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases);
  PhaseShiftWithTpuEncoder(const YAML::Node &node);

private:
  int number_phases_ = 1;
  double average_intensity_ = 0.5;
  double modulation_intensity_ = 0.5;
  void GeneratePatterns();
};
}  // namespace codec
}  // namespace sl_sensor