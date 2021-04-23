#pragma once
// Tools for phase shifting profilometry

#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace slstudio
{
namespace pstools
{
cv::Mat computePhaseVector(unsigned int length, float phase, float pitch);
cv::Mat computePhaseVectorDithered(unsigned int length, float phase, float pitch);
cv::Mat getPhase(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3);
cv::Mat getMagnitude(const cv::Mat I1, const cv::Mat I2, const cv::Mat I3);
std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat> frames);
cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue, unsigned int nPhases);
}  // namespace pstools

}  // namespace slstudio

}  // namespace sl_sensor
