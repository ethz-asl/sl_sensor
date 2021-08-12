#pragma once

#include <vector>
#include "sl_sensor_codec/decoder.hpp"

namespace sl_sensor
{
namespace codec
{
/**
 * @brief Decoder for Two plus one with tpu
 *
 */
class TwoPlusOneWithTpuDecoder : public Decoder
{
public:
  TwoPlusOneWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases);
  TwoPlusOneWithTpuDecoder(const YAML::Node& node);
  virtual void DecodeFrames(cv::Mat& up, cv::Mat& vp, cv::Mat& mask, cv::Mat& shading) override;

private:
  void ComputePhase(const cv::Mat& i_1, const cv::Mat& i_2, const cv::Mat& i_3, const cv::Mat& i_4, const cv::Mat& i_5,
                    int max_pixel, int number_phases, cv::Mat& output_phase);

  int number_phases_ = 1;
  int shading_threshold_ = 55;
  int average_intensity_ = 0.6;
};
}  // namespace codec
}  // namespace sl_sensor