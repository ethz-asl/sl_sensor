#ifndef SL_SENSOR_MOTION_COMPENSATION_PHASE_CORRELATION_UTILITIES_HPP_
#define SL_SENSOR_MOTION_COMPENSATION_PHASE_CORRELATION_UTILITIES_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace sl_sensor {
namespace motion_compensation {
enum class ShiftingOption {
  kHorizontalShiftingOnly,
  kVerticalShiftingOnly,
  kBothDirectionsShifting
};

/**
 * @brief Applies phase correlation image registration to a sequence of images
 *
 * @param input_image_sequence - Input images, must be either CV_8UC1 or CV_32FC1 format
 * @param output_image_sequence - Aligned images, same format as input images
 * @param reference_index - Index of the image that all other images will be aligned to
 * @param subsample_factor - Float larger than 0 and smaller than 1 in which images will be
 * subsampled to before phase correlation is performed to speed up computations. Set to 1,0 to
 * disable subsampling. If number out of the aforemention range is supplied, no subsampling is done
 * by default
 * @param shifting_option - Apply only the horizontal / vertical / both components of the computed
 * shift based on the provided ShiftingOption
 */
void PhaseCorrelateAlignImageSequence(
    const std::vector<cv::Mat> &input_image_sequence, std::vector<cv::Mat> &output_image_sequence,
    size_t reference_index, double subsample_factor = 1.0f,
    ShiftingOption shifting_option = ShiftingOption::kBothDirectionsShifting);

/**
 * @brief Compute the shifts to align the image sequence using phase correlation image registration.
 *
 *
 * @param image_sequence - Input images, must be either CV_8UC1 or CV_32FC1 format
 * @param reference_index - Index of the image that all other images will be aligned to
 * @param shifts - Output computed shifts
 * @param subsample_factor - Float larger than 0 and smaller than 1 in which images will be
 * subsampled to before phase correlation is performed to speed up computations. Set to 1,0 to
 * disable subsampling. If number out of the aforemention range is supplied, no subsampling is done
 * by default
 */
void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                         size_t reference_index, std::vector<cv::Point2d> &shifts,
                                         double subsample_factor);

/**
 * @brief Compute the shifts to align the image sequence using phase correlation image registration.
 * No subsampling is performed
 *
 * @param image_sequence- Input images, must be either CV_8UC1 or CV_32FC1 format
 * @param reference_index - Index of the image that all other images will be aligned to
 * @param shifts - Output computed shifts
 */
void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                         size_t reference_index, std::vector<cv::Point2d> &shifts);

/**
 * @brief Translate input_image_sequence based on the provided shifts
 *
 * @param input_image_sequence - Input images, must be either CV_8UC1 or CV_32FC1 format
 * @param output_image_sequence - Aligned images, same format as input images
 * @param shifts - Shifts to be applied
 * @param shifting_option - Apply only the horizontal / vertical / both components of the computed
 * shift based on the provided ShiftingOption
 */
void ApplyShiftsToImageSequence(
    const std::vector<cv::Mat> &input_image_sequence, std::vector<cv::Mat> &output_image_sequence,
    const std::vector<cv::Point2d> &shifts,
    ShiftingOption shifting_option = ShiftingOption::kBothDirectionsShifting);

/**
 * @brief Subsample an image sequence
 *
 * @param image_sequence_input - Input images, must be either CV_8UC1 or CV_32FC1 format
 * @param image_sequence_output - Aligned images, same format as input images, but size will be
 * difference
 * @param subsample_factor - Float larger than 0 and smaller than 1 in which images will be
 * subsampled to before phase correlation is performed to speed up computations. Set to 1,0 to
 * disable subsampling. If number out of the aforemention range is supplied, no subsampling is done
 * by default
 */
void SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                            std::vector<cv::Mat> &image_sequence_output, double subsample_factor);

}  // namespace motion_compensation
}  // namespace sl_sensor

#endif  // SL_SENSOR_MOTION_COMPENSATION_PHASE_CORRELATION_UTILITIES_HPP_