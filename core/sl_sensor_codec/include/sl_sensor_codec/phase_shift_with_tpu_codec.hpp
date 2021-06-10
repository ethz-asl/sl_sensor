#pragma once

#include <vector>
#include "sl_sensor_codec/codec.hpp"

namespace sl_sensor
{
namespace codec
{
class PhaseShiftWithTpuEncoder : public Encoder
{
public:
  PhaseShiftWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases);
  PhaseShiftWithTpuEncoder(ros::NodeHandle nh);
  virtual cv::Mat GetEncodingPattern(unsigned int depth) override;

private:
  std::vector<cv::Mat> patterns_;
  int number_phases_ = 1;
  void GeneratePatterns();
};

class PhaseShiftWithTpuDecoder : public Decoder
{
public:
  PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDirection dir,
                           unsigned int number_phases);
  PhaseShiftWithTpuDecoder(ros::NodeHandle nh);
  virtual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) override;

private:
  int number_phases_ = 1;
  int shading_threshold_ = 55;
};
}  // namespace codec
}  // namespace sl_sensor