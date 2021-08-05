#include "sl_sensor_motion_compensation/phase_correlation_utilities.hpp"

namespace sl_sensor
{
namespace motion_compensation
{
void PhaseCorrelateAlignImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                      std::vector<cv::Mat> &output_image_sequence, int reference_indice,
                                      double subsample_factor, ShiftingOption shifting_option)
{
  std::vector<cv::Point2d> shifts;
  PhaseCorrelateRegisterImageSequence(input_image_sequence, reference_indice, shifts, subsample_factor);
  ApplyShiftsToImageSequence(input_image_sequence, output_image_sequence, shifts, shifting_option);
}

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, int reference_indice,
                                         std::vector<cv::Point2d> &shifts)
{
  shifts.clear();

  std::vector<cv::Mat> image_seq_to_register;

  // If input is an 8 bit image, we convert it to the required CV_32FC1 format
  if (image_sequence[0].type() == CV_8UC1)
  {
    for (int i = 0; i < (int)image_sequence.size(); i++)
    {
      image_seq_to_register.push_back(cv::Mat(image_sequence[i].size(), CV_32F));
      image_sequence[i].convertTo(image_seq_to_register.back(), CV_32FC1, 1.0f / 255.0f);
    }
  }
  else
  {
    image_seq_to_register = image_sequence;
  }

  for (int i = 0; i < (int)image_seq_to_register.size(); i++)
  {
    if (i != reference_indice)
    {
      shifts.push_back(cv::phaseCorrelate(image_seq_to_register[reference_indice], image_seq_to_register[i]));
    }
    else
    {
      shifts.push_back(cv::Point2d(0.0f, 0.0f));
    }
  }

  /**
  for (const auto &shift : shifts)
  {
    std::cout << shift << " ";
  }
  std::cout << std::endl;
  **/
}

void PhaseCorrelateRegisterImageSequence(const std::vector<cv::Mat> &image_sequence, int reference_indice,
                                         std::vector<cv::Point2d> &shifts, double subsample_factor)
{
  if (subsample_factor <= 0.0f)
  {
    PhaseCorrelateRegisterImageSequence(image_sequence, reference_indice, shifts);
  }
  else
  {
    std::vector<cv::Mat> subsampled_image_sequence = {};
    SubsampleImageSequence(image_sequence, subsampled_image_sequence, subsample_factor);
    PhaseCorrelateRegisterImageSequence(subsampled_image_sequence, reference_indice, shifts);

    for (auto &shift : shifts)
    {
      shift /= subsample_factor;
    }
  }
}

void ApplyShiftsToImageSequence(const std::vector<cv::Mat> &input_image_sequence,
                                std::vector<cv::Mat> &output_image_sequence, const std::vector<cv::Point2d> &shifts,
                                ShiftingOption shifting_option)
{
  const float stationary_tol =
      0.1f;  // Within this amount of pixel shift, we consider image to be stationary in that direction

  output_image_sequence.clear();
  output_image_sequence.resize(input_image_sequence.size(), cv::Mat());

  for (int i = 0; i < (int)shifts.size(); i++)
  {
    float x = (shifting_option == ShiftingOption::kVerticalShiftingOnly) ? 0.0f : shifts[i].x;
    float y = (shifting_option == ShiftingOption::kHorizontalShiftingOnly) ? 0.0f : shifts[i].y;

    bool non_zero_shift =
        !(x < stationary_tol && x > -1.0 * stationary_tol && y < stationary_tol && y > -1.0 * stationary_tol);

    // std::cout << "Shifts: " << x << " | " << y << std::endl;

    if (non_zero_shift)
    {
      cv::Mat M = (cv::Mat_<float>(2, 3) << 1.0f, 0.0f, x, 0.0f, 1.0f, y);
      // std::cout << M << std::endl;
      cv::warpAffine(input_image_sequence[i], output_image_sequence[i], M, input_image_sequence[i].size(),
                     cv::WARP_INVERSE_MAP);
    }
    else
    {
      input_image_sequence[i].copyTo(output_image_sequence[i]);
    }
  }
}

void SubsampleImageSequence(const std::vector<cv::Mat> &image_sequence_input,
                            std::vector<cv::Mat> &image_sequence_output, double subsample_factor)
{
  image_sequence_output.clear();

  for (const auto &input_image : image_sequence_input)
  {
    image_sequence_output.push_back(cv::Mat{ input_image.size(), input_image.type() });
    cv::resize(input_image, image_sequence_output.back(), cv::Size(), subsample_factor, subsample_factor,
               cv::InterpolationFlags::INTER_NEAREST);
  }
}

}  // namespace motion_compensation
}  // namespace sl_sensor