#include "sl_sensor_codec/phase_shift_utilities.hpp"

#include <math.h>

namespace sl_sensor
{
namespace codec
{
// Cosine function vector (3-channel)
cv::Mat ComputePhaseVector(unsigned int length, float phase, float pitch, double average_value,
                           double modulation_intensity)
{
  cv::Mat phase_vector(length, 1, CV_8UC3);

  // Loop through vector
  for (int i = 0; i < phase_vector.rows; i++)
  {
    float amp = average_value + modulation_intensity * cos(2 * M_PI * i / pitch - phase);
    phase_vector.at<cv::Vec3b>(i, 0) = cv::Vec3b(255.0 * amp, 255.0 * amp, 255.0 * amp);
  }

  return phase_vector;
}

// Absolute phase from 3 frames
cv::Mat GetPhase(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3)
{
  cv::Mat_<float> I1_(I1);
  cv::Mat_<float> I2_(I2);
  cv::Mat_<float> I3_(I3);

  cv::Mat phase;

  // One call approach
  cv::phase(2.0 * I1_ - I3_ - I2_, sqrt(3.0) * (I2_ - I3_), phase);
  return phase;
}

// Absolute magnitude from 3 frames
cv::Mat GetMagnitude(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3)
{
  cv::Mat_<float> I1_(I1);
  cv::Mat_<float> I2_(I2);
  cv::Mat_<float> I3_(I3);

  cv::Mat magnitude;

  // One call approach
  cv::magnitude(2.0 * I1_ - I2_ - I3_, sqrt(3.0) * (I2_ - I3_), magnitude);
  magnitude.convertTo(magnitude, CV_8U);

  return magnitude;
}

// Phase unwrapping by means of a phase cue
cv::Mat UnwrapWithCue(const cv::Mat up, const cv::Mat up_cue, unsigned int n_phases)
{
  // Determine number of jumps
  cv::Mat P = (up_cue * n_phases - up) / (2 * M_PI);

  // Round to integers
  P.convertTo(P, CV_8U);
  P.convertTo(P, CV_32F);

  // Add to phase
  cv::Mat up_unwrapped = up + P * 2 * M_PI;

  // Scale to range [0; 2pi]
  up_unwrapped *= 1.0 / n_phases;

  return up_unwrapped;
}

}  // namespace codec

}  // namespace sl_sensor