#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace sl_sensor
{
namespace motion_compensation
{
enum class ShiftingOption
{
  kHorizontalShiftingOnly,
  kVerticalShiftingOnly,
  kBothDirectionsShifting
};

void PhaseCorrelateAlignImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                      std::vector<cv::Mat> &output_image_sequence, size_t reference_indice,
                                      double subsample_factor = 1.0f,
                                      ShiftingOption shifting_option = ShiftingOption::kBothDirectionsShifting);

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, size_t reference_indice,
                                         std::vector<cv::Point2d> &shifts);

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, size_t reference_indice,
                                         std::vector<cv::Point2d> &shifts, double subsample_factor);

void ApplyShiftsToImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                std::vector<cv::Mat> &output_image_sequence, const std::vector<cv::Point2d> &shifts,
                                ShiftingOption shifting_option = ShiftingOption::kBothDirectionsShifting);

void SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                            std::vector<cv::Mat> &image_sequence_output, double subsample_factor);

}  // namespace motion_compensation

}  // namespace sl_sensor