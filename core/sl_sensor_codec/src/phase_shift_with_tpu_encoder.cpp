#include "sl_sensor_codec/phase_shift_utilities.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_encoder.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
// Encoder
PhaseShiftWithTpuEncoder::PhaseShiftWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Encoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 12 : 6;
  GeneratePatterns();
}

PhaseShiftWithTpuEncoder::PhaseShiftWithTpuEncoder(const YAML::Node &node) : Encoder(node)
{
  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 12 : 6;

  number_phases_ = (node["number_phases"]) ? node["number_phases"].as<int>() : number_phases_;
  average_intensity_ = (node["average_intensity"]) ? node["average_intensity"].as<double>() : average_intensity_;
  modulation_intensity_ =
      (node["modulation_intensity"]) ? node["modulation_intensity"].as<double>() : modulation_intensity_;

  GeneratePatterns();
}

void PhaseShiftWithTpuEncoder::GeneratePatterns()
{
  if (direction_ == CodecDirection::kHorizontal || direction_ == CodecDirection::kBoth)
  {
    // Horizontally encoding patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = (float)screen_cols_ / (float)number_phases_;
      cv::Mat pattern(1, 1, CV_8U);
      pattern = ComputePhaseVector(screen_cols_, phase, pitch, average_intensity_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Phase cue patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = screen_cols_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_cols_, phase, pitch, average_intensity_, modulation_intensity_);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }
  }

  if (direction_ == CodecDirection::kVertical || direction_ == CodecDirection::kBoth)
  {
    // Precompute vertically encoding patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = (float)screen_rows_ / (float)number_phases_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_rows_, phase, pitch, average_intensity_, modulation_intensity_);
      patterns_.push_back(pattern);
    }

    // Precompute vertically phase cue patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = screen_rows_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_rows_, phase, pitch, average_intensity_, modulation_intensity_);
      patterns_.push_back(pattern);
    }
  }
}

}  // namespace codec
}  // namespace sl_sensor