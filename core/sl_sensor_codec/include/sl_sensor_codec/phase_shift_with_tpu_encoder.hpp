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
  PhaseShiftWithTpuEncoder(ros::NodeHandle nh);
  virtual cv::Mat GetEncodingPattern(size_t depth) override;

private:
  std::vector<cv::Mat> patterns_;
  int number_phases_ = 1;
  void GeneratePatterns();
};
}  // namespace codec
}  // namespace sl_sensor