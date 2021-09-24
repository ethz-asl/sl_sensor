#include "sl_sensor_motion_compensation/phase_correlation_utilities.hpp"

namespace sl_sensor {
namespace motion_compensation {
void PhaseCorrelateAlignImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                      std::vector<cv::Mat> &output_image_sequence,
                                      size_t reference_index, double subsample_factor,
                                      ShiftingOption shifting_option) {
  std::vector<cv::Point2d> shifts;
  PhaseCorrelateRegisterImageSequence(input_image_sequence, reference_index, shifts,
                                      subsample_factor);
  ApplyShiftsToImageSequence(input_image_sequence, output_image_sequence, shifts, shifting_option);
}

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                         size_t reference_index, std::vector<cv::Point2d> &shifts) {
  shifts.clear();

  std::vector<cv::Mat> image_seq_to_register;

  // If input is an 8 bit image, we convert it to the required CV_32FC1 format
  if (image_sequence[0].type() == CV_8UC1) {
    for (size_t i = 0; i < image_sequence.size(); i++) {
      image_seq_to_register.push_back(cv::Mat(image_sequence[i].size(), CV_32F));
      image_sequence[i].convertTo(image_seq_to_register.back(), CV_32FC1, 1.0f / 255.0f);
    }
  } else {
    image_seq_to_register = image_sequence;
  }

  for (size_t i = 0; i < image_seq_to_register.size(); i++) {
    if (i != reference_index) {
      shifts.push_back(
          cv::phaseCorrelate(image_seq_to_register[reference_index], image_seq_to_register[i]));
    } else {
      shifts.push_back(cv::Point2d(0.0f, 0.0f));
    }
  }
}

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence,
                                         size_t reference_index, std::vector<cv::Point2d> &shifts,
                                         double subsample_factor) {
  // We do not subsample if subsample factor is negative or larger than /equal to 1.0
  if (subsample_factor <= 0.0f || subsample_factor > 0.99999f) {
    PhaseCorrelateRegisterImageSequence(image_sequence, reference_index, shifts);
  } else {
    std::vector<cv::Mat> subsampled_image_sequence = {};
    SubsampleImageSequence(image_sequence, subsampled_image_sequence, subsample_factor);
    PhaseCorrelateRegisterImageSequence(subsampled_image_sequence, reference_index, shifts);

    // Rescale computed shifts
    for (auto &shift : shifts) {
      shift /= subsample_factor;
    }
  }
}

void ApplyShiftsToImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                std::vector<cv::Mat> &output_image_sequence,
                                const std::vector<cv::Point2d> &shifts,
                                ShiftingOption shifting_option) {
  // Within this amount of pixel shift, we consider image to be stationary in that direction
  const float stationary_tol = 0.1f;

  output_image_sequence.clear();
  output_image_sequence.resize(input_image_sequence.size(), cv::Mat());

  for (int i = 0; i < (int)shifts.size(); i++) {
    float x = (shifting_option == ShiftingOption::kVerticalShiftingOnly) ? 0.0f : shifts[i].x;
    float y = (shifting_option == ShiftingOption::kHorizontalShiftingOnly) ? 0.0f : shifts[i].y;

    bool non_zero_shift = !(x < stationary_tol && x > -1.0 * stationary_tol && y < stationary_tol &&
                            y > -1.0 * stationary_tol);

    if (non_zero_shift) {
      cv::Mat M = (cv::Mat_<float>(2, 3) << 1.0f, 0.0f, x, 0.0f, 1.0f, y);
      cv::warpAffine(input_image_sequence[i], output_image_sequence[i], M,
                     input_image_sequence[i].size(), cv::WARP_INVERSE_MAP);
    } else {
      input_image_sequence[i].copyTo(output_image_sequence[i]);
    }
  }
}

void SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                            std::vector<cv::Mat> &image_sequence_output, double subsample_factor) {
  image_sequence_output.clear();

  for (const auto &input_image : image_sequence_input) {
    image_sequence_output.push_back(cv::Mat{input_image.size(), input_image.type()});
    cv::resize(input_image, image_sequence_output.back(), cv::Size(), subsample_factor,
               subsample_factor, cv::InterpolationFlags::INTER_NEAREST);
  }
}

}  // namespace motion_compensation
}  // namespace sl_sensor