#include "sl_sensor_codec/phase_shift_utilities.hpp"
#include "sl_sensor_codec/two_plus_one_with_tpu_decoder.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
// Decoder
TwoPlusOneWithTpuDecoder::TwoPlusOneWithTpuDecoder(unsigned int screen_cols, unsigned int screen_rows,
                                                   CodecDirection dir, unsigned int number_phases)
  : Decoder(screen_cols, screen_rows, dir), number_phases_(number_phases)
{
  number_patterns_ = (dir == CodecDirection::kBoth) ? 10 : 5;
  frames_.resize(number_patterns_, cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

TwoPlusOneWithTpuDecoder::TwoPlusOneWithTpuDecoder(ros::NodeHandle nh) : Decoder(nh)
{
  nh.param<int>("number_phases", number_phases_, number_phases_);
  nh.param<int>("shading_threshold", shading_threshold_, shading_threshold_);

  number_patterns_ = (direction_ == CodecDirection::kBoth) ? 10 : 5;
  frames_.resize(number_patterns_, cv::Mat(0, 0, CV_8UC1));  // Set number of elements in frames_ vector
}

void TwoPlusOneWithTpuDecoder::DecodeFrames(cv::Mat& up, cv::Mat& vp, cv::Mat& mask, cv::Mat& shading)
{
  // Set default output formats
  up = cv::Mat(0, 0, CV_32FC1);
  vp = cv::Mat(0, 0, CV_32FC1);
  mask = cv::Mat(0, 0, CV_8UC1);
  shading = cv::Mat(0, 0, CV_8UC1);

  if (direction_ == CodecDirection::kHorizontal)
  {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]), cv::Mat_<float>(frames_[2]),
                 cv::Mat_<float>(frames_[3]), cv::Mat_<float>(frames_[4]), screen_cols_, number_phases_, up);
  }
  else if (direction_ == CodecDirection::kVertical)
  {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]), cv::Mat_<float>(frames_[2]),
                 cv::Mat_<float>(frames_[3]), cv::Mat_<float>(frames_[4]), screen_rows_, number_phases_, vp);
  }
  else
  {
    ComputePhase(cv::Mat_<float>(frames_[0]), cv::Mat_<float>(frames_[1]), cv::Mat_<float>(frames_[2]),
                 cv::Mat_<float>(frames_[3]), cv::Mat_<float>(frames_[4]), screen_cols_, number_phases_, up);
    ComputePhase(cv::Mat_<float>(frames_[5]), cv::Mat_<float>(frames_[6]), cv::Mat_<float>(frames_[7]),
                 cv::Mat_<float>(frames_[8]), cv::Mat_<float>(frames_[9]), screen_rows_, number_phases_, vp);
  }

  // Calculate modulation
  shading = 2.0f * frames_[1];

  // Generate shading
  mask = shading > shading_threshold_;
}

void TwoPlusOneWithTpuDecoder::ComputePhase(const cv::Mat& i_1, const cv::Mat& i_2, const cv::Mat& i_3,
                                            const cv::Mat& i_4, const cv::Mat& i_5, int max_pixel, int number_phases,
                                            cv::Mat& output_phase)
{
  cv::phase(i_1 - i_2, i_3 - i_2, output_phase);
  cv::Mat cue;
  cv::phase(i_4 - i_2, i_5 - i_2, cue);
  output_phase = UnwrapWithCue(output_phase, cue, number_phases);
  output_phase *= max_pixel / (2 * M_PI);
}

}  // namespace codec
}  // namespace sl_sensor