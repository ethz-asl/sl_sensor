#include "sl_sensor_calibration/camera_parameters.hpp"

#include "sl_sensor_calibration/calibration_utils.hpp"

namespace sl_sensor
{
namespace calibration
{
CameraParameters::CameraParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                                   double calibration_error, int resolution_x, int resolution_y,
                                   const cv::Matx33f& extrinsic_rot, const cv::Vec3f& extrinsic_trans,
                                   double stereo_error)
  : IntrinsicParameters(intrinsic_mat, lens_distortion, calibration_error, resolution_x, resolution_y)
  , extrinsic_rot_(extrinsic_rot)
  , extrinsic_trans_(extrinsic_trans)
  , stereo_error_(stereo_error)

{
}

CameraParameters::CameraParameters(const std::string& filename) : IntrinsicParameters(filename)
{
  LoadExtrinsic(filename);
}

CameraParameters::CameraParameters(){};

const cv::Matx33f& CameraParameters::extrinsic_rot() const
{
  return extrinsic_rot_;
}

const cv::Vec3f& CameraParameters::extrinsic_trans() const
{
  return extrinsic_trans_;
}

const double& CameraParameters::stereo_error() const
{
  return stereo_error_;
}

bool CameraParameters::SaveExtrinsic(const std::string& filename)
{
  std::string file_extension = this->GetFileExtension(filename);

  if (file_extension == "xml")
  {
    // We append to original parameter file with intrinsics
    cv::FileStorage fs(filename, cv::FileStorage::APPEND);

    if (fs.isOpened())
    {
      fs << "extrinsic_rot" << cv::Mat(extrinsic_rot_) << "extrinsic_trans" << cv::Mat(extrinsic_trans_)
         << "stereo_error" << stereo_error_;
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

bool CameraParameters::LoadExtrinsic(const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "[IntrinsicParameters] Error: could not open file " << filename << std::endl;
    return false;
  }

  cv::Mat temp;
  fs["extrinsic_rot"] >> temp;
  extrinsic_rot_ = temp;
  fs["extrinsic_trans"] >> temp;
  extrinsic_trans_ = temp;

  fs["stereo_error"] >> stereo_error_;

  fs.release();

  return true;
}

std::ostream& operator<<(std::ostream& os, const CameraParameters& data)
{
  os << (const IntrinsicParameters&)data << std::endl;

  os << std::setw(5) << std::setprecision(4) << "=== Extrinsic Parameters ===\n"
     << "Extrinsic Rotation Matrix: \n"
     << data.extrinsic_rot() << "\n"
     << "Extrinsic Translation Vector: \n"
     << data.extrinsic_trans() << "\n"
     << "Stereo Error: \n"
     << data.stereo_error() << std::endl;

  return os;
}

bool CameraParameters::Save(const std::string& filename)
{
  bool success = false;

  success = IntrinsicParameters::Save(filename);
  success = SaveExtrinsic(filename) & success;

  return success;
}

bool CameraParameters::Load(const std::string& filename)
{
  bool success = false;

  success = IntrinsicParameters::Load(filename);
  success = LoadExtrinsic(filename) & success;

  return success;
}

cv::Mat CameraParameters::GetTransformationMatrix() const
{
  cv::Mat transformation_matrix = cv::Mat::eye(4, 4, CV_32F);
  GetExtrinsicMatrix().copyTo(transformation_matrix(cv::Range(0, 3), cv::Range(0, 4)));
  return transformation_matrix;
}

cv::Mat CameraParameters::GetInverseTransformationMatrix() const
{
  cv::Mat inverted_transformation_matrix = cv::Mat::eye(4, 4, CV_32F);
  SwapFramesCVMat(GetTransformationMatrix(), inverted_transformation_matrix);
  return inverted_transformation_matrix;
}

cv::Mat CameraParameters::GetExtrinsicMatrix() const
{
  cv::Mat extrinsic_matrix = cv::Mat::zeros(3, 4, CV_32F);
  cv::Mat(this->extrinsic_rot()).copyTo(extrinsic_matrix(cv::Range(0, 3), cv::Range(0, 3)));
  cv::Mat(this->extrinsic_trans()).copyTo(extrinsic_matrix(cv::Range(0, 3), cv::Range(3, 4)));
  return extrinsic_matrix;
}

}  // namespace calibration
}  // namespace sl_sensor