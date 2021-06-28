#include <ros/ros.h>

#include <experimental/filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include "sl_sensor_calibration/calibration_data.hpp"
#include "sl_sensor_calibration/calibration_option.hpp"
#include "sl_sensor_calibration/calibrator.hpp"

using namespace sl_sensor::calibration;

std::vector<std::string> SplitString(const std::string& s, const std::string& delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "calibrator");
  ros::NodeHandle nh_public;
  ros::NodeHandle nh_private("~");

  // Load params from private ros node
  std::string camera_folder_name;
  std::string projector_folder_name;
  std::string directories;
  std::vector<std::string> directories_vec;
  std::vector<std::unordered_set<std::strings>> files_to_ignore_vec;

  CalibrationOption camera_calibration_option;
  std::string camera_intrinsics_str = "1 0 0 0 1 0 0 0 1";
  std::string camera_distortion_str = "0 0 0 0 0";
  CalibrationOption projector_calibration_option;
  std::string projector_intrinsics_str = "1 0 0 0 1 0 0 0 1";
  std::string projector_distortion_str = "0 0 0 0 0";

  int checkerboard_rows = 0;
  int checkerboard_cols = 0;
  int checkerboard_size_mm = 10;

  int window_radius = 10;
  int minimum_valid_pixels = 50;
  double homography_ransac_threshold = 3.0;

  private_nh.param<std::string>("camera_folder_name", camera_folder_name, camera_folder_name);
  private_nh.param<std::string>("projector_folder_name", projector_folder_name, projector_folder_name);
  private_nh.param<std::string>("directories", directories);
  private_nh.param<std::string>("projector_yaml_directory");

  private_nh.param<bool>("camera_fix_principle_point", camera_calibration_option.fix_prinicple_point,
                         camera_calibration_option.fix_prinicple_point);
  private_nh.param<bool>("camera_fix_aspect_ratio", camera_calibration_option.fix_aspect_ratio,
                         camera_calibration_option.fix_aspect_ratio);
  private_nh.param<bool>("camera_fix_k2", camera_calibration_option.fix_k2, camera_calibration_option.fix_k2);
  private_nh.param<bool>("camera_fix_k3", camera_calibration_option.fix_k3, camera_calibration_option.fix_k3);
  private_nh.param<bool>("camera_zero_tangential_distortion", camera_calibration_option.zero_tangential_distortion,
                         camera_calibration_option.zero_tangential_distortion);
  private_nh.param<bool>("camera_fix_values", camera_calibration_option.fix_values,
                         camera_calibration_option.fix_values);
  private_nh.param<bool>("camera_use_initial_guess", camera_calibration_option.use_initial_guess,
                         camera_calibration_option.use_initial_guess);
  private_nh.param<std::string>("camera_intrinsics_init", camera_intrinsics_str, camera_intrinsics_str);
  private_nh.param<std::string>("camera_lens_distortion_init", camera_distortion_str, camera_distortion_str);

  private_nh.param<bool>("projector_fix_principle_point", projector_calibration_option.fix_prinicple_point,
                         projector_calibration_option.fix_prinicple_point);
  private_nh.param<bool>("projector_fix_aspect_ratio", projector_calibration_option.fix_aspect_ratio,
                         projector_calibration_option.fix_aspect_ratio);
  private_nh.param<bool>("projector_fix_k2", projector_calibration_option.fix_k2, projector_calibration_option.fix_k2);
  private_nh.param<bool>("projector_fix_k3", projector_calibration_option.fix_k3, projector_calibration_option.fix_k3);
  private_nh.param<bool>("projector_zero_tangential_distortion",
                         projector_calibration_option.zero_tangential_distortion,
                         projector_calibration_option.zero_tangential_distortion);
  private_nh.param<bool>("projector_fix_values", projector_calibration_option.fix_values,
                         projector_calibration_option.fix_values);
  private_nh.param<bool>("projector_use_initial_guess", projector_calibration_option.use_initial_guess,
                         projector_calibration_option.use_initial_guess);
  private_nh.param<std::string>("projector_intrinsics_init", projector_intrinsics_str, projector_intrinsics_str);
  private_nh.param<std::string>("projector_lens_distortion_init", projector_distortion_str, projector_distortion_str);

  private_nh.param<int>("checkerboard_rows", checkerboard_rows, checkerboard_rows);
  private_nh.param<int>("checkerboard_cols", checkerboard_cols, checkerboard_cols);
  private_nh.param<double>("checkerboard_size_mm", checkerboard_size_mm, checkerboard_size_mm);

  private_nh.param<int>("window_radius", window_radius, window_radius);
  private_nh.param<int>("minimum_valid_pixels", minimum_valid_pixels, minimum_valid_pixels);
  private_nh.param<double>("homography_ransac_threshold", homography_ransac_threshold, homography_ransac_threshold);

  for ()
  {
  }

  // Add calibration sequences to calibrator

  // Perform calibration and save to calibration file

  return 0;
}