#include "sl_sensor_codec/phase_shift_utilities.hpp"
#include "sl_sensor_codec/phase_shift_with_tpu_decoder.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
// Decoder
PhaseShiftWithTpuDecoder::PhaseShiftWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Decoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = (dir == CodecDirection::kBoth) ? 12 : 6;
  frames_.resize(number_patterns_, cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

PhaseShiftWithTpuDecoder::PhaseShiftWithTpuDecoder(ros::NodeHandle nh) : Decoder(nh)
{
  nh.param<int>("number_phases", number_phases_, number_phases_);
  nh.param<int>("shading_threshold", shading_threshold_, shading_threshold_);

  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 12 : 6;
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