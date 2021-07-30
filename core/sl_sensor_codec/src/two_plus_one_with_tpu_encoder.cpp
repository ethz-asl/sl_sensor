#include "sl_sensor_codec/phase_shift_utilities.hpp"
#include "sl_sensor_codec/two_plus_one_with_tpu_encoder.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
// Encoder
TwoPlusOneWithTpuEncoder::TwoPlusOneWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Encoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 9 : 5;
  GeneratePatterns();
}

TwoPlusOneWithTpuEncoder::TwoPlusOneWithTpuEncoder(ros::NodeHandle nh) : Encoder(nh)
{
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 9 : 5;

  nh.param<int>("number_phases", number_phases_, number_phases_);
  nh.param<double>("average_value", average_value_, average_value_);
  nh.param<double>("modulation_intensity", modulation_intensity_, modulation_intensity_);

  GeneratePatterns();
}

void TwoPlusOneWithTpuEncoder::GeneratePatterns()
{
  // Initialise flat pattern
  cv::Mat flat_pattern(5, 5, CV_8UC3);
  flat_pattern.setTo(0.6 * 255.0);

  if (direction_ == CodecDirection::kHorizontal || direction_ == CodecDirection::kBoth)
  {
    // Horizontally encoding patterns_
    for (unsigned int i = 0; i < 2; i++)
    {
      float phase = M_PI / 2.0 * i;
      float pitch = (float)screen_cols_ / (float)number_phases_;
      cv::Mat pattern(1, 1, CV_8U);
      pattern = ComputePhaseVector(screen_cols_, phase, pitch, average_value_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Phase cue patterns_
    for (unsigned int i = 0; i < 2; i++)
    {
      float phase = M_PI / 2.0 * i;
      float pitch = screen_cols_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_cols_, phase, pitch, average_value_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Insert flat pattern
    patterns_.insert(patterns_.end() - 3, flat_pattern);
  }

  if (direction_ == CodecDirection::kVertical || direction_ == CodecDirection::kBoth)
  {
    // Precompute vertically encoding patterns_
    for (unsigned int i = 0; i < 2; i++)
    {
      float phase = M_PI / 2.0 * i;
      float pitch = (float)screen_rows_ / (float)number_phases_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_rows_, phase, pitch, average_value_, modulation_intensity_);
      patterns_.push_back(pattern);
    }

    // Precompute vertically phase cue patterns_
    for (unsigned int i = 0; i < 2; i++)
    {
      float phase = M_PI / 2.0 * i;
      float pitch = screen_rows_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_rows_, phase, pitch, average_value_, modulation_intensity_);
      patterns_.push_back(pattern);
    }

    // Insert flat pattern only if pattern is vertical. If both directions, flat pattern has already been added when
    // processing horizontal pattern
    if (direction_ == CodecDirection::kVertical)
    {
      patterns_.insert(patterns_.end() - 3, flat_pattern);
    }
  }
}

cv::Mat TwoPlusOneWithTpuEncoder::GetEncodingPattern(size_t depth)
{
  return patterns_[depth];
}

}  // namespace codec
}  // namespace sl_sensor