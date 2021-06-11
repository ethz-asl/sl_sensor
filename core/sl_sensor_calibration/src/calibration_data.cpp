#include "sl_sensor_calibration/calibration_data.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>

namespace sl_sensor
{
namespace calibration
{
std::string GetFileExtension(const std::string& filename)
{
  return (filename.find_last_of(".") != std::string::npos) ? filename.substr(filename.find_last_of(".") + 1) : "";
}

CalibrationData::CalibrationData()
  : Kc_(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
  , kc_(0.0)
  , cam_error_(0.0)
  , Kp_(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
  , kp_(0.0)
  , proj_error_(0.0)
  , Rp_(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
  , Tp_(0.0)
  , stereo_error_(0.0)
  , frame_width_(640)
  , frame_height_(512)
  , screen_res_x_(0)
  , screen_res_y_(0)
{
}

CalibrationData::CalibrationData(cv::Matx33f Kc, cv::Vec<float, 5> kc, double cam_error, cv::Matx33f Kp,
                                 cv::Vec<float, 5> kp, double proj_error, cv::Matx33f Rp, cv::Vec3f Tp,
                                 double stereo_error)
  : Kc_(Kc)
  , kc_(kc)
  , cam_error_(cam_error)
  , Kp_(Kp)
  , kp_(kp)
  , proj_error_(proj_error)
  , Rp_(Rp)
  , Tp_(Tp)
  , stereo_error_(stereo_error)
{
}

bool CalibrationData::Load(const std::string& filename)
{
  std::ifstream file(filename);

  if (file.good() && GetFileExtension(filename) == "xml")
  {
    return LoadXML(filename);
  }
  else
  {
    std::cerr << "[CalibrationData] Error: No such .xml file: " << filename << std::endl;
    return false;
  }
}

bool CalibrationData::Save(const std::string& filename)
{
  std::string file_extension = GetFileExtension(filename);

  if (file_extension == "xml")
  {
    return SaveXML(filename);
  }
  else if (file_extension == "m")
  {
    return SaveMatlab(filename);
  }
  else
  {
    std::cerr << "[CalibrationData] Error: Unknown file extension: " << file_extension << std::endl;
    return false;
  }

  return false;
}

bool CalibrationData::LoadXML(const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "CalibrationData error: could not open file " << filename << std::endl;
    return false;
  }

  cv::Mat temp;
  fs["Kc"] >> temp;
  Kc_ = temp;
  fs["kc"] >> temp;
  kc_ = temp;
  fs["Kp"] >> temp;
  Kp_ = temp;
  fs["kp"] >> temp;
  kp_ = temp;
  fs["Rp"] >> temp;
  Rp_ = temp;
  fs["Tp"] >> temp;
  Tp_ = temp;

  fs["cam_error"] >> cam_error_;
  fs["proj_error"] >> proj_error_;
  fs["stereo_error"] >> stereo_error_;

  fs["frame_width"] >> frame_width_;
  fs["frame_height"] >> frame_height_;
  fs["screen_res_x"] >> screen_res_x_;
  fs["screen_res_y"] >> screen_res_y_;

  fs.release();

  return true;
}

bool CalibrationData::SaveXML(const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    return false;
  }

  fs << "Kc" << cv::Mat(Kc_) << "kc" << cv::Mat(kc_) << "Kp" << cv::Mat(Kp_) << "kp" << cv::Mat(kp_) << "Rp"
     << cv::Mat(Rp_) << "Tp" << cv::Mat(Tp_) << "cam_error" << cam_error_ << "proj_error" << proj_error_
     << "stereo_error" << stereo_error_ << "frame_width" << frame_width_ << "frame_height" << frame_height_
     << "screen_res_x" << screen_res_x_ << "screen_res_y" << screen_res_y_;
  fs.release();

  return true;
}

bool CalibrationData::SaveMatlab(const std::string& filename)
{
  std::ofstream file(filename);
  if (!file)
  {
    return false;
  }

  file << "%%SLStudio calibration" << std::endl;
  file << "Kc = " << Kc_ << ";" << std::endl;
  file << "kc = " << kc_ << ";" << std::endl;
  file << "Kp = " << Kp_ << ";" << std::endl;
  file << "kp = " << kp_ << ";" << std::endl;
  file << "Rp = " << Rp_ << ";" << std::endl;
  file << "Tp = " << Tp_ << ";" << std::endl;

  file.close();

  return true;
}

std::ostream& operator<<(std::ostream& os, const CalibrationData& data)
{
  os << std::setw(5) << std::setprecision(4) << "========================================\n"
     << "Camera Calibration: \n"
     << "- cam_error:\n"
     << data.cam_error_ << "\n"
     << "- Kc:\n"
     << data.Kc_ << "\n"
     << "- kc:\n"
     << data.kc_ << "\n"
     << "Projector Calibration: "
     << "\n"
     << "- proj_error: \n"
     << data.proj_error_ << "\n"
     << "- Kp: \n"
     << data.Kp_ << "\n"
     << "- kp: \n"
     << data.kp_ << "\n"
     << "Stereo Calibration: \n"
     << "- stereo_error:\n"
     << data.stereo_error_ << "\n"
     << "- Rp:\n"
     << data.Rp_ << "\n"
     << "- Tp:\n"
     << data.Tp_ << std::endl;

  return os;
}

}  // namespace calibration
}  // namespace sl_sensor