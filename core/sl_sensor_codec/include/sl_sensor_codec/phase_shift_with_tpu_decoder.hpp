// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_
#define SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_

#pragma once

#include <vector>
#include "sl_sensor_codec/decoder.hpp"

namespace sl_sensor {
namespace codec {
/**
 * @brief Decoder for PSP with TPU pattern
 *
 */
class PhaseShiftWithTpuDecoder : public Decoder {
 public:
  PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases_horizontal,
                           unsigned int number_phases_vertical);
  PhaseShiftWithTpuDecoder(const YAML::Node &node);
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) override;

 private:
  unsigned int number_phases_horizontal_ = 1;
  unsigned int number_phases_vertical_ = 1;
  unsigned int shading_threshold_ = 55;
};
}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_PHASE_SHIFT_WITH_TPU_DECODER_HPP_