#include <math.h>
#include <iostream>
#include "sl_sensor_reconstruction/triangulator.hpp"

using namespace sl_sensor::calibration;

namespace sl_sensor
{
namespace reconstruction
{
CalibrationData Triangulator::GetCalibrationData()
{
  return calibration_data_;
}

Triangulator::Triangulator(CalibrationData calibration) : calibration_data_(calibration)
{
  // Precompute uc_, vc_ maps
  uc_.create(calibration_data_.frame_height_, calibration_data_.frame_width_, CV_32F);
  vc_.create(calibration_data_.frame_height_, calibration_data_.frame_width_, CV_32F);

  for (unsigned int row = 0; row < (unsigned int)calibration_data_.frame_height_; row++)
  {
    for (unsigned int col = 0; col < (unsigned int)calibration_data_.frame_width_; col++)
    {
      uc_.at<float>(row, col) = col;
      vc_.at<float>(row, col) = row;
    }
  }

  // Precompute determinant tensor
  Pc_ = cv::Mat(3, 4, CV_32F, cv::Scalar(0.0));
  cv::Mat(calibration_data_.Kc_).copyTo(Pc_(cv::Range(0, 3), cv::Range(0, 3)));

  Pp_ = cv::Mat(3, 4, CV_32F);
  cv::Mat temp(3, 4, CV_32F);
  cv::Mat(calibration_data_.Rp_).copyTo(temp(cv::Range(0, 3), cv::Range(0, 3)));
  cv::Mat(calibration_data_.Tp_).copyTo(temp(cv::Range(0, 3), cv::Range(3, 4)));
  Pp_ = cv::Mat(calibration_data_.Kp_) * temp;

  cv::Mat e = cv::Mat::eye(4, 4, CV_32F);

  int sz[] = { 4, 3, 3, 3 };
  cv::Mat C(4, sz, CV_32F, cv::Scalar::all(0));
  for (int k = 0; k < 4; k++)
  {
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        for (int l = 0; l < 3; l++)
        {
          cv::Mat op(4, 4, CV_32F);
          Pc_.row(i).copyTo(op.row(0));
          Pc_.row(j).copyTo(op.row(1));
          Pp_.row(l).copyTo(op.row(2));
          e.row(k).copyTo(op.row(3));
          C.at<float>(cv::Vec4i(k, i, j, l)) = cv::determinant(op.t());
        }
      }
    }
  }
  determinant_tensor_ = C;

  // Precompute lens correction maps
  cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
  cv::initUndistortRectifyMap(calibration_data_.Kc_, calibration_data_.kc_, eye, calibration_data_.Kc_,
                              cv::Size(calibration_data_.frame_width_, calibration_data_.frame_height_), CV_16SC2,
                              lens_map_1_, lens_map_2_);

  // Precompute parts of xyzw
  xyzw_precompute_offset_.resize(4);
  xyzw_precompute_factor_.resize(4);
  for (unsigned int i = 0; i < 4; i++)
  {
    xyzw_precompute_offset_[i] = C.at<float>(cv::Vec4i(i, 0, 1, 0)) - C.at<float>(cv::Vec4i(i, 2, 1, 0)) * uc_ -
                                 C.at<float>(cv::Vec4i(i, 0, 2, 0)) * vc_;
    xyzw_precompute_factor_[i] = -C.at<float>(cv::Vec4i(i, 0, 1, 2)) + C.at<float>(cv::Vec4i(i, 2, 1, 2)) * uc_ +
                                 C.at<float>(cv::Vec4i(i, 0, 2, 2)) * vc_;
  }

  // Precompute camera coordinates matrix in UpVpTriangulate
  int number_pixels_ = calibration_data_.frame_height_ * calibration_data_.frame_width_;
  proj_points_cam_ = cv::Mat(2, number_pixels_, CV_32F);

  uc_.reshape(0, 1).copyTo(proj_points_cam_.row(0));
  vc_.reshape(0, 1).copyTo(proj_points_cam_.row(1));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Triangulator::Triangulate(const cv::Mat &up, const cv::Mat &vp,
                                                                 const cv::Mat &mask, const cv::Mat &shading)
{
  cv::Mat up_undistorted;
  cv::Mat vp_undistorted;
  cv::Mat mask_undistorted;
  cv::Mat shading_undistorted;

  // Undistort up, mask and shading
  if (!up.empty())
  {
    cv::remap(up, up_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }
  if (!vp.empty())
  {
    cv::remap(vp, vp_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }

  cv::remap(mask, mask_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  cv::remap(shading, shading_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);

  // Triangulate
  cv::Mat xyz;
  if (!up.empty() && vp.empty())
  {
    TriangulateFromUp(up_undistorted, xyz);
  }
  else if (!vp.empty() && up.empty())
  {
    TriangulateFromVp(vp_undistorted, xyz);
  }
  else if (!up.empty() && !vp.empty())
  {
    TriangulateFromUpVp(up_undistorted, vp_undistorted, xyz);
  }

  // Apply Mask
  cv::Mat masked_xyz(uc_.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
  xyz.copyTo(masked_xyz, mask_undistorted);

  // Convert to pcl dense point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl_pc_ptr->width = masked_xyz.cols;
  pcl_pc_ptr->height = masked_xyz.rows;
  pcl_pc_ptr->is_dense = true;

  pcl_pc_ptr->points.resize(masked_xyz.rows * masked_xyz.cols);

  for (int row = 0; row < masked_xyz.rows; row++)
  {
    int offset = row * pcl_pc_ptr->width;
    for (int col = 0; col < masked_xyz.cols; col++)
    {
      const cv::Vec3f point_coords = masked_xyz.at<cv::Vec3f>(row, col);
      unsigned char shade = shading_undistorted.at<unsigned char>(row, col);
      pcl::PointXYZRGB point;
      point.x = point_coords[0];
      point.y = point_coords[1];
      point.z = point_coords[2];
      point.r = shade;
      point.g = shade;
      point.b = shade;
      pcl_pc_ptr->points[offset + col] = point;
    }
  }

  return pcl_pc_ptr;
}

void Triangulator::TriangulateFromUp(const cv::Mat &up, cv::Mat &xyz)
{
  // Solve for xyzw using determinant tensor
  cv::Mat C = determinant_tensor_;
  std::vector<cv::Mat> xyzw(4);
  for (unsigned int i = 0; i < 4; i++)
  {
    xyzw[i] = xyzw_precompute_offset_[i] + xyzw_precompute_factor_[i].mul(up);
  }

  // Convert to non homogenous coordinates
  cv::Mat winv;
  cv::divide(1.0, xyzw[3], winv);
  for (unsigned int i = 0; i < 3; i++)
  {
    xyzw[i] = xyzw[i].mul(winv);
  }

  // Merge
  cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin() + 3), xyz);
}

void Triangulator::TriangulateFromVp(const cv::Mat &vp, cv::Mat &xyz)
{
  // Solve for xyzw using determinant tensor
  cv::Mat C = determinant_tensor_;
  std::vector<cv::Mat> xyzw(4);
  for (unsigned int i = 0; i < 4; i++)
  {
    xyzw[i] = C.at<float>(cv::Vec4i(i, 0, 1, 1)) - C.at<float>(cv::Vec4i(i, 2, 1, 1)) * uc_ -
              C.at<float>(cv::Vec4i(i, 0, 2, 1)) * vc_ - C.at<float>(cv::Vec4i(i, 0, 1, 2)) * vp +
              C.at<float>(cv::Vec4i(i, 2, 1, 2)) * vp.mul(uc_) + C.at<float>(cv::Vec4i(i, 0, 2, 2)) * vp.mul(vc_);
  }

  // Convert to non homogenous coordinates
  cv::Mat winv;
  cv::divide(1.0, xyzw[3], winv);
  for (unsigned int i = 0; i < 3; i++)
    xyzw[i] = xyzw[i].mul(winv);

  // Merge
  cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin() + 3), xyz);
}

void Triangulator::TriangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp, cv::Mat &xyz)
{
  // TODO: Assign proj_points_proj without cloning (most likely with for loop, which is how copyTo
  // does it)
  cv::Mat proj_points_proj(2, number_pixels_, CV_32F);
  up.clone().reshape(0, 1).copyTo(proj_points_proj.row(0));
  vp.clone().reshape(0, 1).copyTo(proj_points_proj.row(1));

  cv::Mat xyzw;
  cv::triangulatePoints(Pc_, Pp_, proj_points_cam_, proj_points_proj, xyzw);

  xyz.create(3, number_pixels_, CV_32F);
  for (int i = 0; i < number_pixels_; i++)
  {
    xyz.at<float>(0, i) = xyzw.at<float>(0, i) / xyzw.at<float>(3, i);
    xyz.at<float>(1, i) = xyzw.at<float>(1, i) / xyzw.at<float>(3, i);
    xyz.at<float>(2, i) = xyzw.at<float>(2, i) / xyzw.at<float>(3, i);
  }

  xyz = xyz.t();
  xyz = xyz.reshape(3, up.rows);
}

}  // namespace reconstruction
}  // namespace sl_sensor
