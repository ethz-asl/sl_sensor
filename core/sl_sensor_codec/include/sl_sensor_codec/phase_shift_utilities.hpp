#pragma once

#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace codec
{
cv::Mat ComputePhaseVector(unsigned int length, float phase, float pitch, double average_value = 0.5,
                           double modulation_intensity = 0.5);

cv::Mat ComputePhaseVectorDithered(unsigned int length, float phase, float pitch);

cv::Mat GetPhase(const cv::Mat& I1, const cv::Mat& I2, const cv::Mat& I3);

cv::Mat GetMagnitude(const cv::Mat& I1, const cv::Mat& I2, const cv::Mat& I3);

cv::Mat UnwrapWithCue(const cv::Mat& up, const cv::Mat& up_cue, unsigned int n_phases);

}  // namespace codec
}  // namespace sl_sensor
