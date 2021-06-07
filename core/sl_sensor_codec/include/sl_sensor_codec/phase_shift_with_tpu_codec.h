#pragma once

#include <vector>
#include "sl_sensor_codec/codec.h"

namespace sl_sensor
{
namespace codec
{
class PhaseShiftWithTPUEncoder : public Encoder
{
public:
  PhaseShiftWithTPUEncoder(unsigned int screen_cols, unsigned int screen_rows, CodecDir dir);
  virtual cv::Mat GetEncodingPattern(unsigned int depth) override;

private:
  std::vector<cv::Mat> patterns_;
};

class PhaseShiftWithTPUDecoder : public Decoder
{
public:
  PhaseShiftWithTPUDecoder(unsigned int screen_cols, unsigned int screen_rows, CodecDir dir);
  virtual void SetFrame(unsigned int depth, cv::Mat frame) override;
  vritual void DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);

private:
  std::vector<cv::Mat> frames_;
};