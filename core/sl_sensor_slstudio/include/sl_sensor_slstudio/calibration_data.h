#pragma once

#include <opencv2/core/core.hpp>
#include <string>

namespace sl_sensor
{
namespace slstudio
{
class CalibrationData
{
public:
  CalibrationData();
  CalibrationData(cv::Matx33f _Kc, cv::Vec<float, 5> _kc, double _cam_error, cv::Matx33f _Kp, cv::Vec<float, 5> _kp,
                  double _proj_error, cv::Matx33f _Rp, cv::Vec3f _Tp, double _stereo_error);

  bool load(const std::string& filename);
  bool save(const std::string& filename);
  bool loadXML(const std::string& filename);
  bool saveXML(const std::string& filename);
  bool saveMatlab(const std::string& filename);
  bool saveSLCALIB(const std::string& filename);
  void print(std::ostream& stream);

  cv::Matx33f Kc;        // Intrinsic camera matrix
  cv::Vec<float, 5> kc;  // Camera distortion coefficients
  double cam_error;

  cv::Matx33f Kp;        // Intrinsic projector matrix
  cv::Vec<float, 5> kp;  // Projector distortion coefficients
  double proj_error;

  cv::Matx33f Rp;  // Extrinsic camera rotation matrix
  cv::Vec3f Tp;    // Extrinsic camera rotation matrix

  double stereo_error;

  int frameWidth, frameHeight;
  int screenResX, screenResY;
  std::string calibrationDateTime;

private:
  std::string get_file_extension(const std::string& filename);
  bool file_exists(const std::string& filename);
};
}  // namespace slstudio

}  // namespace sl_sensor
