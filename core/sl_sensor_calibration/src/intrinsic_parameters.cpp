#include "sl_sensor_calibration/intrinsic_parameters.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace sl_sensor
{
namespace calibration
{
std::string IntrinsicParameters::GetFileExtension(const std::string& filename)
{
  return (filename.find_last_of(".") != std::string::npos) ? filename.substr(filename.find_last_of(".") + 1) : "";
}

IntrinsicParameters::IntrinsicParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                                         double calibration_error, int resolution_x, int resolution_y)
  : intrinsic_mat_(intrinsic_mat)
  , lens_distortion_(lens_distortion)
  , calibration_error_(calibration_error)
  , resolution_x_(resolution_x)
  , resolution_y_(resolution_y)
{
}

IntrinsicParameters::IntrinsicParameters(const std::string& filename)
{
  Load(filename);
}

bool IntrinsicParameters::Save(const std::string& filename)
{
  std::string file_extension = this->GetFileExtension(filename);

  if (file_extension == "xml")
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (fs.isOpened())
    {
      fs << "intrinsic_mat" << cv::Mat(intrinsic_mat_) << "lens_distortion" << cv::Mat(lens_distortion_)
         << "calibration_error" << calibration_error_ << "resolution_x" << resolution_x_ << "resolution_y"
         << resolution_y_;
      fs.release();
      return true;
    }
  }
  else
  {
    std::cerr << "[IntrinsicParameters] Error: File extension not .xml, got " << file_extension << " instead."
              << std::endl;
  }

  return false;
}

bool IntrinsicParameters::Load(const std::string& filename)
{
  std::cout << filename << std::endl;

  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "[IntrinsicParameters] Error: could not open file " << filename << std::endl;
    return false;
  }

  cv::Mat temp;
  fs["intrinsic_mat"] >> temp;
  intrinsic_mat_ = temp;
  fs["lens_distortion"] >> temp;
  lens_distortion_ = temp;

  fs["calibration_error"] >> calibration_error_;
  fs["resolution_x"] >> resolution_x_;
  fs["resolution_y"] >> resolution_y_;

  fs.release();

  return true;
}

const cv::Matx33f& IntrinsicParameters::intrinsic_mat() const
{
  return intrinsic_mat_;
}

const cv::Vec<float, 5>& IntrinsicParameters::lens_distortion() const
{
  return lens_distortion_;
}

const double& IntrinsicParameters::calibration_error() const
{
  return calibration_error_;
}

const int& IntrinsicParameters::resolution_x() const
{
  return resolution_x_;
}

const int& IntrinsicParameters::resolution_y() const
{
  return resolution_y_;
}

std::ostream& operator<<(std::ostream& os, const IntrinsicParameters& data)
{
  os << std::setw(5) << std::setprecision(4) << "=== Intrinsic Parameters ===\n"
     << "Intrinsic Matrix: \n"
     << data.intrinsic_mat() << "\n"
     << "Lens Distortion: \n"
     << data.lens_distortion() << "\n"
     << "Calibration Error: \n"
     << data.calibration_error() << "\n"
     << "Resolution x: \n"
     << data.resolution_x() << "\n"
     << "Resolution y: \n"
     << data.resolution_y() << "\n"
     << std::endl;
  return os;
}

}  // namespace calibration

}  // namespace sl_sensor