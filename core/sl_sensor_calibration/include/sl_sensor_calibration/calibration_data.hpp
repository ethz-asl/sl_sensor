#pragma once

#include <opencv2/core/core.hpp>
#include <string>

namespace sl_sensor
{
namespace calibration
{
class CalibrationData
{
public:
  CalibrationData();
  CalibrationData(cv::Matx33f Kc, cv::Vec<float, 5> kc, double cam_error, cv::Matx33f Kp, cv::Vec<float, 5> kp,
                  double proj_error, cv::Matx33f Rp, cv::Vec3f Tp, double stereo_error);
  bool Load(const std::string& filename);
  bool Save(const std::string& filename);
  bool LoadXML(const std::string& filename);
  bool SaveXML(const std::string& filename);
  bool SaveMatlab(const std::string& filename);

  cv::Matx33f Kc_;        // Intrinsic camera matrix
  cv::Vec<float, 5> kc_;  // Camera distortion coefficients
  double cam_error_;

  cv::Matx33f Kp_;        // Intrinsic projector matrix
  cv::Vec<float, 5> kp_;  // Projector distortion coefficients
  double proj_error_;

  cv::Matx33f Rp_;  // Extrinsic camera rotation matrix
  cv::Vec3f Tp_;    // Extrinsic camera rotation matrix

  double stereo_error_;

  int frame_width_, frame_height_;
  int screen_res_x_, screen_res_y_;
};

std::ostream& operator<<(std::ostream& os, const CalibrationData& dt);

}  // namespace calibration
}  // namespace sl_sensor
