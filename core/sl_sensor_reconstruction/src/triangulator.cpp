// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#include "sl_sensor_reconstruction/triangulator.hpp"
#include <math.h>
#include <omp.h>
#include <iostream>

using namespace sl_sensor::calibration;

namespace sl_sensor {
namespace reconstruction {
std::pair<ProjectorParameters, CameraParameters> Triangulator::GetCalibrationParams() {
  return std::make_pair(projector_parameters_, triangulation_camera_parameters_);
}

Triangulator::Triangulator(const calibration::ProjectorParameters &projector_parameters,
                           const calibration::CameraParameters &triangulation_camera_parameters,
                           const calibration::CameraParameters &secondary_camera_parameters)
    : projector_parameters_(projector_parameters),
      triangulation_camera_parameters_(triangulation_camera_parameters),
      colour_camera_parameters_(secondary_camera_parameters) {
  InitTriangulationParameters();
  InitColourShadingInfo();
}

Triangulator::Triangulator(const calibration::ProjectorParameters &projector_parameters,
                           const calibration::CameraParameters &triangulation_camera_parameters)
    : projector_parameters_(projector_parameters),
      triangulation_camera_parameters_(triangulation_camera_parameters) {
  InitTriangulationParameters();
}

void Triangulator::InitColourShadingInfo() {
  colour_shading_enabled_ = true;

  cv::Mat transform_p_c1 = triangulation_camera_parameters_.GetTransformationMatrix();
  cv::Mat transform_c2_p = colour_camera_parameters_.GetInverseTransformationMatrix();
  cv::Mat transform_c2_c1 = transform_c2_p * transform_p_c1;

  cv::Mat rot = cv::Mat(3, 3, CV_32F);
  cv::Mat trans = cv::Mat(3, 1, CV_32F);

  transform_c2_c1(cv::Range(0, 3), cv::Range(0, 3)).copyTo(rot);
  transform_c2_c1(cv::Range(0, 3), cv::Range(3, 4)).copyTo(trans);

  cv::Rodrigues(rot, colour_shading_info_.rvec);
  colour_shading_info_.tvec = trans;
  colour_shading_info_.lens_distortion = colour_camera_parameters_.lens_distortion();
  colour_shading_info_.intrinsic_mat = colour_camera_parameters_.intrinsic_mat();
}

void Triangulator::InitTriangulationParameters() {
  // Precompute uc_, vc_ maps
  uc_.create(triangulation_camera_parameters_.resolution_y(),
             triangulation_camera_parameters_.resolution_x(), CV_32F);
  vc_.create(triangulation_camera_parameters_.resolution_y(),
             triangulation_camera_parameters_.resolution_x(), CV_32F);

  for (unsigned int row = 0; row < (unsigned int)triangulation_camera_parameters_.resolution_y();
       row++) {
    for (unsigned int col = 0; col < (unsigned int)triangulation_camera_parameters_.resolution_x();
         col++) {
      uc_.at<float>(row, col) = col;
      vc_.at<float>(row, col) = row;
    }
  }

  // Compute camera matrix from calibration data
  projection_matrix_camera_ = cv::Mat(3, 4, CV_32F, cv::Scalar(0.0));
  cv::Mat(triangulation_camera_parameters_.intrinsic_mat())
      .copyTo(projection_matrix_camera_(cv::Range(0, 3), cv::Range(0, 3)));

  projection_matrix_projector_ = cv::Mat(projector_parameters_.intrinsic_mat()) *
                                 triangulation_camera_parameters_.GetProjectionMatrix();

  // Precompute determinant tensor
  int determinant_tensor_size[] = {4, 3, 3, 3};  // Dimensions of determinant tensor
  cv::Mat basis_vectors = cv::Mat::eye(4, 4, CV_32F);
  determinant_tensor_ = cv::Mat(4, determinant_tensor_size, CV_32F, cv::Scalar::all(0));
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int l = 0; l < 3; l++) {
          cv::Mat op(4, 4, CV_32F);
          projection_matrix_camera_.row(i).copyTo(op.row(0));
          projection_matrix_camera_.row(j).copyTo(op.row(1));
          projection_matrix_projector_.row(l).copyTo(op.row(2));
          basis_vectors.row(k).copyTo(op.row(3));
          determinant_tensor_.at<float>(cv::Vec4i(k, i, j, l)) = cv::determinant(op.t());
        }
      }
    }
  }

  // Precompute lens correction maps
  cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
  cv::initUndistortRectifyMap(triangulation_camera_parameters_.intrinsic_mat(),
                              triangulation_camera_parameters_.lens_distortion(), eye,
                              triangulation_camera_parameters_.intrinsic_mat(),
                              cv::Size(triangulation_camera_parameters_.resolution_x(),
                                       triangulation_camera_parameters_.resolution_y()),
                              CV_16SC2, lens_map_1_, lens_map_2_);

  // Precompute parts of xyzw
  cv::Mat &dt = determinant_tensor_;
  xyzw_precompute_offset_.resize(4);
  xyzw_precompute_factor_.resize(4);
  for (unsigned int i = 0; i < 4; i++) {
    xyzw_precompute_offset_[i] = dt.at<float>(cv::Vec4i(i, 0, 1, 0)) -
                                 dt.at<float>(cv::Vec4i(i, 2, 1, 0)) * uc_ -
                                 dt.at<float>(cv::Vec4i(i, 0, 2, 0)) * vc_;
    xyzw_precompute_factor_[i] = -dt.at<float>(cv::Vec4i(i, 0, 1, 2)) +
                                 dt.at<float>(cv::Vec4i(i, 2, 1, 2)) * uc_ +
                                 dt.at<float>(cv::Vec4i(i, 0, 2, 2)) * vc_;
  }

  // Precompute camera coordinates matrix in UpVpTriangulate
  number_pixels_ = triangulation_camera_parameters_.resolution_y() *
                   triangulation_camera_parameters_.resolution_x();
  proj_points_cam_ = cv::Mat(2, number_pixels_, CV_32F);

  uc_.reshape(0, 1).copyTo(proj_points_cam_.row(0));
  vc_.reshape(0, 1).copyTo(proj_points_cam_.row(1));
}

void Triangulator::UndistortImages(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                                   const cv::Mat &shading, cv::Mat &up_undistorted,
                                   cv::Mat &vp_undistorted, cv::Mat &mask_undistorted,
                                   cv::Mat &shading_undistorted) {
  // Undistort up, vp, mask and shading
  if (!up.empty()) {
    cv::remap(up, up_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }
  if (!vp.empty()) {
    cv::remap(vp, vp_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }

  if (!mask.empty()) {
    cv::remap(mask, mask_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }

  if (!shading.empty()) {
    cv::remap(shading, shading_undistorted, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
  }
}

void Triangulator::Triangulate(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                               cv::Mat &xyz)

{
  // Triangulate
  if (!up.empty() && vp.empty()) {
    TriangulateFromUp(up, xyz);
  } else if (!vp.empty() && up.empty()) {
    TriangulateFromVp(vp, xyz);
  } else if (!up.empty() && !vp.empty()) {
    TriangulateFromUpVp(up, vp, xyz);
  }

  // Apply Mask
  cv::Mat masked_xyz(uc_.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
  xyz.copyTo(masked_xyz, mask);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Triangulator::TriangulateMonochrome(const cv::Mat &up,
                                                                         const cv::Mat &vp,
                                                                         const cv::Mat &mask,
                                                                         const cv::Mat &shading) {
  // Undistort input
  cv::Mat up_undistorted;
  cv::Mat vp_undistorted;
  cv::Mat mask_undistorted;
  cv::Mat shading_undistorted;
  UndistortImages(up, vp, mask, shading, up_undistorted, vp_undistorted, mask_undistorted,
                  shading_undistorted);

  // Perform triangulation
  cv::Mat xyz;
  Triangulate(up_undistorted, vp_undistorted, mask_undistorted, xyz);

  // Convert coordinates to point cloud, with shading as intensity values
  return ConvertToMonochomePCLPointCLoud(xyz, mask_undistorted, shading_undistorted);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Triangulator::TriangulateColour(
    const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask, const cv::Mat &colour_shading) {
  if (!colour_shading_enabled_) {
    std::cerr << "[Triangulator] Error: TriangulateColour called when Triangulator was not "
                 "configured to generate "
                 "coloured point cloud. Returning empty pointer!"
              << std::endl;
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
  }

  // Undistort input
  cv::Mat up_undistorted;
  cv::Mat vp_undistorted;
  cv::Mat mask_undistorted;
  cv::Mat shading_undistorted;
  UndistortImages(up, vp, mask, cv::Mat(), up_undistorted, vp_undistorted, mask_undistorted,
                  shading_undistorted);

  // Perform triangulation
  cv::Mat xyz;
  Triangulate(up_undistorted, vp_undistorted, mask_undistorted, xyz);

  // Convert coordinates to point cloud, with shading as intensity values
  return ConvertToColourPCLPointCloud(xyz, mask_undistorted, colour_shading);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Triangulator::ConvertToColourPCLPointCloud(
    const cv::Mat &xyz, const cv::Mat &mask, const cv::Mat &colour_shading) {
  pcl::PointXYZRGB default_point;
  default_point.x = NAN;
  default_point.y = NAN;
  default_point.z = NAN;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>(xyz.cols, xyz.rows, default_point));

  pcl_pc_ptr->points.resize(xyz.rows * xyz.cols);

#pragma omp parallel for
  for (int row = 0; row < xyz.rows; row++) {
    int offset = row * pcl_pc_ptr->width;
    for (int col = 0; col < xyz.cols; col++) {
      if (mask.at<bool>(row, col) == true) {
        std::vector<cv::Point2f> output_point_vec;

        // We project 3D point to secondary camera's image to get pixel coordinate
        cv::projectPoints(std::vector<cv::Vec3f>{xyz.at<cv::Vec3f>(row, col)},
                          colour_shading_info_.rvec, colour_shading_info_.tvec,
                          colour_shading_info_.intrinsic_mat, colour_shading_info_.lens_distortion,
                          output_point_vec);

        cv::Point2f &output_point = output_point_vec.at(0);

        // If pixel coordinate is witin the coloured image, we populate the point cloud with the
        // cloud 3D point
        if (output_point.x > 0 && output_point.y > 0 && output_point.x < xyz.cols &&
            output_point.y < xyz.rows) {
          const cv::Vec3f point_coords = xyz.at<cv::Vec3f>(row, col);

          const int coord_x = std::floor(output_point.x);
          const int coord_y = std::floor(output_point.y);
          pcl::PointXYZRGB &point = pcl_pc_ptr->points[offset + col];
          point.x = point_coords[0];
          point.y = point_coords[1];
          point.z = point_coords[2];
          point.r = colour_shading.at<cv::Vec3b>(coord_y, coord_x)[2];
          point.g = colour_shading.at<cv::Vec3b>(coord_y, coord_x)[1];
          point.b = colour_shading.at<cv::Vec3b>(coord_y, coord_x)[0];
        }
      }
    }
  }

  return pcl_pc_ptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Triangulator::ConvertToMonochomePCLPointCLoud(
    const cv::Mat &xyz, const cv::Mat &mask, const cv::Mat &shading) {
  pcl::PointXYZI default_point;
  default_point.x = NAN;
  default_point.y = NAN;
  default_point.z = NAN;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>(xyz.cols, xyz.rows, default_point));

  pcl_pc_ptr->points.resize(xyz.rows * xyz.cols);

#pragma omp parallel for
  for (int row = 0; row < xyz.rows; row++) {
    int offset = row * pcl_pc_ptr->width;
    for (int col = 0; col < xyz.cols; col++) {
      if (mask.at<bool>(row, col) == true) {
        const cv::Vec3f point_coords = xyz.at<cv::Vec3f>(row, col);
        pcl::PointXYZI point;
        point.x = point_coords[0];
        point.y = point_coords[1];
        point.z = point_coords[2];
        point.intensity = shading.at<unsigned char>(row, col);
        pcl_pc_ptr->points[offset + col] = point;
      }
    }
  }

  return pcl_pc_ptr;
}

void Triangulator::TriangulateFromUp(const cv::Mat &up, cv::Mat &xyz) {
  // Solve for xyzw
  std::vector<cv::Mat> xyzw(4);
  for (unsigned int i = 0; i < 4; i++) {
    xyzw[i] = xyzw_precompute_offset_[i] + xyzw_precompute_factor_[i].mul(up);
  }

  // Convert to non homogenous coordinates
  cv::Mat winv;
  cv::divide(1.0, xyzw[3], winv);
  for (unsigned int i = 0; i < 3; i++) {
    xyzw[i] = xyzw[i].mul(winv);
  }

  // Merge
  cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin() + 3), xyz);
}

void Triangulator::TriangulateFromVp(const cv::Mat &vp, cv::Mat &xyz) {
  // Solve for xyzw using determinant tensor
  cv::Mat &dt = determinant_tensor_;
  std::vector<cv::Mat> xyzw(4);
  for (unsigned int i = 0; i < 4; i++) {
    xyzw[i] = dt.at<float>(cv::Vec4i(i, 0, 1, 1)) - dt.at<float>(cv::Vec4i(i, 2, 1, 1)) * uc_ -
              dt.at<float>(cv::Vec4i(i, 0, 2, 1)) * vc_ - dt.at<float>(cv::Vec4i(i, 0, 1, 2)) * vp +
              dt.at<float>(cv::Vec4i(i, 2, 1, 2)) * vp.mul(uc_) +
              dt.at<float>(cv::Vec4i(i, 0, 2, 2)) * vp.mul(vc_);
  }

  // Convert to non homogenous coordinates
  cv::Mat winv;
  cv::divide(1.0, xyzw[3], winv);
  for (unsigned int i = 0; i < 3; i++) {
    xyzw[i] = xyzw[i].mul(winv);
  }

  // Merge
  cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin() + 3), xyz);
}

void Triangulator::TriangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp, cv::Mat &xyz) {
  // TODO: Assign proj_points_proj without cloning (most likely with for loop, which is how copyTo
  // does it)
  cv::Mat proj_points_proj(2, number_pixels_, CV_32F);
  up.clone().reshape(0, 1).copyTo(proj_points_proj.row(0));
  vp.clone().reshape(0, 1).copyTo(proj_points_proj.row(1));

  cv::Mat xyzw;
  cv::triangulatePoints(projection_matrix_camera_, projection_matrix_projector_, proj_points_cam_,
                        proj_points_proj, xyzw);

  xyz.create(3, number_pixels_, CV_32F);

#pragma omp parallel for
  for (int i = 0; i < number_pixels_; i++) {
    xyz.at<float>(0, i) = xyzw.at<float>(0, i) / xyzw.at<float>(3, i);
    xyz.at<float>(1, i) = xyzw.at<float>(1, i) / xyzw.at<float>(3, i);
    xyz.at<float>(2, i) = xyzw.at<float>(2, i) / xyzw.at<float>(3, i);
  }

  xyz = xyz.t();
  xyz = xyz.reshape(3, up.rows);
}

}  // namespace reconstruction
}  // namespace sl_sensor
