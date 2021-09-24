// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef INTRINSIC_PARAMETERS_HPP_
#define INTRINSIC_PARAMETERS_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object to store camera intrinsic parameters
 *
 */
class IntrinsicParameters {
 public:
  IntrinsicParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                      double calibration_error, int resolution_x, int resolution_y);

  IntrinsicParameters(const std::string& filename);

  IntrinsicParameters(){};

  virtual bool Save(const std::string& filename);

  virtual bool Load(const std::string& filename);

  const cv::Matx33f& intrinsic_mat() const;
  const cv::Vec<float, 5>& lens_distortion() const;
  const double& calibration_error() const;
  const int& resolution_x() const;
  const int& resolution_y() const;

  static std::string GetFileExtension(const std::string& filename);

 protected:
  cv::Matx33f intrinsic_mat_ = cv::Matx33f::eye();
  cv::Vec<float, 5> lens_distortion_ = cv::Vec<float, 5>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  double calibration_error_ = 0.0f;
  int resolution_x_ = 0;
  int resolution_y_ = 0;
};

std::ostream& operator<<(std::ostream& os, const IntrinsicParameters& dt);

}  // namespace calibration
}  // namespace sl_sensor

#endif  // INTRINSIC_PARAMETERS_HPP_