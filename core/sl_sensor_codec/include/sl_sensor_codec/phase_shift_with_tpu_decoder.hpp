#pragma once

#include <vector>
#include "sl_sensor_codec/decoder.hpp"

namespace sl_sensor
{
namespace codec
{
/**
 * @brief Decoder for PSP with TPU pattern
 *
 */
class PhaseShiftWithTpuDecoder : public Decoder
{
public:
  PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases);
  PhaseShiftWithTpuDecoder(const YAML::Node &node);
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) override;

private:
  int number_phases_ = 1;
  int shading_threshold_ = 55;
};
}  // namespace codec
}  // namespace sl_sensor