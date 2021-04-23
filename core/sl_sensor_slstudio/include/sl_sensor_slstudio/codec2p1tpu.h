#pragma once

#include "sl_sensor_slstudio/codec.h"

namespace sl_sensor
{
namespace slstudio
{
class EncoderPhaseShift2p1Tpu : public Encoder
{
public:
  EncoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth);

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift2p1Tpu : public Decoder
{
public:
  DecoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
  ~DecoderPhaseShift2p1Tpu();

  void decode_keypoints(const std::vector<cv::Mat> &image_sequence, std::vector<cv::KeyPoint> &keypoint_vec,
                        std::vector<float> &phase_vector);

private:
  std::vector<cv::Mat> frames;
  std::vector<cv::Point2d> shiftHistory;
  cv::Mat_<float> *lastShading;
};

}  // namespace slstudio
}  // namespace sl_sensor