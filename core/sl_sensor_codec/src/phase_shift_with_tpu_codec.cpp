#include "sl_sensor_codec/phase_shift_utilities.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_codec.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
unsigned int GetNumberPatternsPhaseShiftWithTpu(CodecDirection dir)
{
  return (dir == CodecDirection::kBoth) ? 12 : 6;
}

// Encoder
PhaseShiftWithTpuEncoder::PhaseShiftWithTpuEncoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Encoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = GetNumberPatternsPhaseShiftWithTpu(direction_);
  GeneratePatterns();
}

PhaseShiftWithTpuEncoder::PhaseShiftWithTpuEncoder(ros::NodeHandle nh) : Encoder(nh)
{
  number_patterns_ = GetNumberPatternsPhaseShiftWithTpu(direction_);

  nh.param<int>("number_phases", number_phases_, number_phases_);

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
      pattern = ComputePhaseVector(screen_cols_, phase, pitch);
      pattern = pattern.t();
      patterns_.push_back(pattern);
    }

    // Phase cue patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = screen_cols_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_cols_, phase, pitch);
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
      pattern = ComputePhaseVector(screen_rows_, phase, pitch);
      patterns_.push_back(pattern);
    }

    // Precompute vertically phase cue patterns_
    for (unsigned int i = 0; i < 3; i++)
    {
      float phase = 2.0 * M_PI / 3.0 * i;
      float pitch = screen_rows_;
      cv::Mat pattern;
      pattern = ComputePhaseVector(screen_rows_, phase, pitch);
      patterns_.push_back(pattern);
    }
  }
}

cv::Mat PhaseShiftWithTpuEncoder::GetEncodingPattern(unsigned int depth)
{
  return patterns_[depth];
}

// Decoder
PhaseShiftWithTpuDecoder::PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Decoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = GetNumberPatternsPhaseShiftWithTpu(dir);
  frames_.resize(number_patterns_, cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

PhaseShiftWithTpuDecoder::PhaseShiftWithTpuDecoder(ros::NodeHandle nh) : Decoder(nh)
{
  nh.param<int>("number_phases", number_phases_, number_phases_);
  nh.param<int>("shading_threshold", shading_threshold_, shading_threshold_);

  number_patterns_ = GetNumberPatternsPhaseShiftWithTpu(direction_);
  frames_.resize(number_patterns_, cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

void PhaseShiftWithTpuDecoder::DecodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading)
{
  // Set default output formats
  up = cv::Mat(0, 0, CV_32FC1);
  vp = cv::Mat(0, 0, CV_32FC1);
  mask = cv::Mat(0, 0, CV_8UC1);
  shading = cv::Mat(0, 0, CV_8UC1);

  if (direction_ == CodecDirection::kHorizontal || direction_ == CodecDirection::kBoth)
  {
    // Horizontal decoding
    up = GetPhase(frames_[0], frames_[1], frames_[2]);

    cv::Mat up_cue = GetPhase(frames_[3], frames_[4], frames_[5]);

    up = UnwrapWithCue(up, up_cue, number_phases_);

    up *= screen_cols_ / (2 * M_PI);
  }
  if (direction_ == CodecDirection::kVertical || direction_ == CodecDirection::kBoth)
  {
    // Vertical decoding
    std::vector<cv::Mat> frames_vertical(frames_.end() - 6, frames_.end());
    vp = GetPhase(frames_vertical[0], frames_vertical[1], frames_vertical[2]);
    cv::Mat vp_cue = GetPhase(frames_vertical[3], frames_vertical[4], frames_vertical[5]);
    vp = UnwrapWithCue(vp, vp_cue, number_phases_);
    vp *= screen_rows_ / (2 * M_PI);
  }

  // Calculate modulation
  shading = GetMagnitude(frames_[0], frames_[1], frames_[2]);

  // Generate shading
  mask = shading > shading_threshold_;
}

}  // namespace codec
}  // namespace sl_sensor