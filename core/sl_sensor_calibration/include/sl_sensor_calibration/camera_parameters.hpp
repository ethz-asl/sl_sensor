// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_
#define SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_

#include "sl_sensor_calibration/intrinsic_parameters.hpp"

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object to store Camera calibration information
 * It contains:
 * 1) Camera Intrinsic paramters
 * 2) Transformation between projector frame to camera frame
 *
 */
class CameraParameters {
 public:
  CameraParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                   double calibration_error, int resolution_x, int resolution_y,
                   const cv::Matx33f& extrinsic_rot, const cv::Vec3f& extrinsic_trans,
                   double stereo_error);

  CameraParameters(const std::string& filename);

  CameraParameters();

  bool Save(const std::string& filename);

  bool Load(const std::string& filename);

  const cv::Matx33f& extrinsic_rot() const;

  const cv::Vec3f& extrinsic_trans() const;

  const double& stereo_error() const;

  cv::Mat GetTransformationMatrix() const;

  cv::Mat GetInverseTransformationMatrix() const;

  cv::Mat GetProjectionMatrix() const;

  const cv::Matx33f& intrinsic_mat() const;
  const cv::Vec<float, 5>& lens_distortion() const;
  const double& calibration_error() const;
  const int& resolution_x() const;
  const int& resolution_y() const;

  const IntrinsicParameters& intrinsic_parameters() const;

 private:
  IntrinsicParameters intrinsic_parameters_;
  cv::Matx33f extrinsic_rot_ = cv::Matx33f::eye();
  cv::Vec3f extrinsic_trans_ = cv::Vec3f(0, 0, 0);
  double stereo_error_;

  bool SaveExtrinsic(const std::string& filename);
  bool LoadExtrinsic(const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const CameraParameters& dt);

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_