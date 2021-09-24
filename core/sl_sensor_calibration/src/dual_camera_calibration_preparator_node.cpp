#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include <sl_sensor_projector/projector_utilities.hpp>

#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/dual_camera_calibration_preparator.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

using namespace sl_sensor::calibration;

bool DirectoryExists(const char* path) {
  struct stat info;

  if (stat(path, &info) != 0)
    return false;
  else if (info.st_mode & S_IFDIR)
    return true;
  else
    return false;
};

std::vector<std::string> SplitString(const std::string& s, const std::string& delimiter) {
  std::vector<std::string> res;
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));

  return res;
};

bool GetCvMatFromXml(const std::string& filename, const std::string& param_name, cv::Mat& mat) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    return false;
  }

  fs[param_name] >> mat;

  return mat.empty() ? false : true;
};

int main(int argc, char** argv) {
  // Init ros node
  ros::init(argc, argv, "dual_camera_calibration_preparator");
  ros::NodeHandle nh_public;
  ros::NodeHandle private_nh("~");

  // Load params from private ros node
  std::string pri_camera_folder_name;
  std::string sec_camera_folder_name;
  std::string pri_projector_folder_name;
  std::string sec_projector_folder_name;
  std::string directories;
  std::vector<std::string> directories_vec;
  std::vector<std::unordered_set<std::string>> files_to_ignore_vec;
  std::string pri_camera_parameters_filename = "";
  std::string sec_camera_parameters_filename = "";
  std::string projector_parameters_filename = "";
  std::string bundle_adjustment_problem_output_filename = "";

  int checkerboard_rows = 0;
  int checkerboard_cols = 0;
  double checkerboard_size = 10;

  int window_radius = 10;
  int minimum_valid_pixels = 50;

  double projector_acceptance_tol = 0.25;

  private_nh.param<std::string>("pri_camera_folder_name", pri_camera_folder_name,
                                pri_camera_folder_name);
  private_nh.param<std::string>("sec_camera_folder_name", sec_camera_folder_name,
                                sec_camera_folder_name);
  private_nh.param<std::string>("pri_projector_folder_name", pri_projector_folder_name,
                                pri_projector_folder_name);
  private_nh.param<std::string>("sec_projector_folder_name", sec_projector_folder_name,
                                sec_projector_folder_name);
  private_nh.param<std::string>("directories", directories, directories);

  private_nh.param<std::string>("bundle_adjustment_problem_output_filename",
                                bundle_adjustment_problem_output_filename,
                                bundle_adjustment_problem_output_filename);

  private_nh.param<std::string>("pri_camera_parameters_filename", pri_camera_parameters_filename,
                                pri_camera_parameters_filename);
  private_nh.param<std::string>("sec_camera_parameters_filename", sec_camera_parameters_filename,
                                sec_camera_parameters_filename);
  private_nh.param<std::string>("projector_parameters_filename", projector_parameters_filename,
                                projector_parameters_filename);

  private_nh.param<int>("checkerboard_num_rows", checkerboard_rows, checkerboard_rows);
  private_nh.param<int>("checkerboard_num_cols", checkerboard_cols, checkerboard_cols);
  private_nh.param<double>("checkerboard_size", checkerboard_size, checkerboard_size);

  private_nh.param<int>("window_radius", window_radius, window_radius);
  private_nh.param<int>("minimum_valid_pixels", minimum_valid_pixels, minimum_valid_pixels);

  private_nh.param<double>("projector_acceptance_tol", projector_acceptance_tol,
                           projector_acceptance_tol);

  CameraParameters pri_cam_params(pri_camera_parameters_filename);
  CameraParameters sec_cam_params(sec_camera_parameters_filename);
  ProjectorParameters proj_params(projector_parameters_filename);

  std::string delimiter = " ";
  directories_vec = SplitString(directories, delimiter);

  for (unsigned int i = 1; i <= directories_vec.size(); i++) {
    std::string ignore_param_name = "ignore_" + std::to_string(i);
    std::string temp_string = "";
    private_nh.param<std::string>(ignore_param_name, temp_string, temp_string);

    std::vector<std::string> ignore_filenames_vec = SplitString(temp_string, delimiter);

    std::unordered_set<std::string> ignore_filenames;

    for (const auto& filenames : ignore_filenames_vec) {
      ignore_filenames.insert(filenames);
    }

    files_to_ignore_vec.push_back(ignore_filenames);
  }

  // Initialise preparator
  DualCameraCalibrationPreparator preparator(proj_params, pri_cam_params, sec_cam_params,
                                             checkerboard_cols, checkerboard_rows,
                                             checkerboard_size, projector_acceptance_tol);
  preparator.SetLocalHomographySettings(window_radius, minimum_valid_pixels);

  // Add calibration sequences to calibrator
  int counter = 0;
  for (const auto& directory : directories_vec) {
    // Check if the required folders exist
    std::string pri_cam_shading_directory = directory + pri_camera_folder_name + "/shading";
    std::string pri_cam_mask_directory = directory + pri_camera_folder_name + "/mask";
    std::string sec_cam_shading_directory = directory + sec_camera_folder_name + "/shading";
    std::string sec_cam_mask_directory = directory + sec_camera_folder_name + "/mask";
    std::string pri_projector_up_directory = directory + pri_projector_folder_name + "/up";
    std::string pri_projector_vp_directory = directory + pri_projector_folder_name + "/vp";
    std::string sec_projector_up_directory = directory + sec_projector_folder_name + "/up";
    std::string sec_projector_vp_directory = directory + sec_projector_folder_name + "/vp";

    if (!DirectoryExists(pri_cam_shading_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Shading Directory " +
                                   pri_cam_shading_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(pri_cam_mask_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Mask Directory " +
                                   pri_cam_mask_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(sec_cam_shading_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Shading Directory " +
                                   sec_cam_shading_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(sec_cam_mask_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Mask Directory " +
                                   sec_cam_mask_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(pri_projector_up_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Up Directory " +
                                   pri_projector_up_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(pri_projector_vp_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Vp Directory " +
                                   pri_projector_vp_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(sec_projector_up_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Up Directory " +
                                   sec_projector_up_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(sec_projector_vp_directory.c_str())) {
      std::string status_message = "[DualCameraCalibrationPreparatorNode] Vp Directory " +
                                   sec_projector_vp_directory +
                                   " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    for (const auto& entry :
         std::experimental::filesystem::directory_iterator(pri_cam_shading_directory)) {
      std::string stem = entry.path().stem().u8string();

      // If stem is in the list of filenames to ignore, skip to next entry
      if (files_to_ignore_vec[counter].find(stem) != files_to_ignore_vec[counter].end()) {
        std::cout << "Ignoring files named " << stem << std::endl;
        continue;
      }

      // Load cv::Mat from files
      cv::Mat pri_shading;
      cv::Mat pri_mask;
      cv::Mat sec_shading;
      cv::Mat sec_mask;
      cv::Mat pri_up;
      cv::Mat pri_vp;
      cv::Mat sec_up;
      cv::Mat sec_vp;

      pri_shading = cv::imread(pri_cam_shading_directory + "/" + stem + ".bmp", CV_8UC1);
      pri_mask = cv::imread(pri_cam_mask_directory + "/" + stem + ".bmp", CV_8UC1);
      sec_shading = cv::imread(sec_cam_shading_directory + "/" + stem + ".bmp", CV_8UC1);
      sec_mask = cv::imread(sec_cam_mask_directory + "/" + stem + ".bmp", CV_8UC1);

      bool pri_shading_success = !pri_shading.empty();
      bool sec_shading_success = !sec_shading.empty();
      bool pri_mask_success = !pri_mask.empty();
      bool sec_mask_success = !sec_mask.empty();
      bool pri_up_success =
          GetCvMatFromXml(pri_projector_up_directory + "/" + stem + ".xml", "up", pri_up);
      bool pri_vp_success =
          GetCvMatFromXml(pri_projector_vp_directory + "/" + stem + ".xml", "vp", pri_vp);
      bool sec_up_success =
          GetCvMatFromXml(sec_projector_up_directory + "/" + stem + ".xml", "up", sec_up);
      bool sec_vp_success =
          GetCvMatFromXml(sec_projector_vp_directory + "/" + stem + ".xml", "vp", sec_vp);

      // If all read successfully, add to calibrator
      if (pri_shading_success && sec_shading_success && pri_mask_success && sec_mask_success &&
          pri_up_success && pri_vp_success && sec_up_success && sec_vp_success) {
        std::string label = directory + "|" + stem;
        bool load_success = preparator.AddSingleCalibrationSequence(
            pri_shading, pri_mask, pri_up, pri_vp, sec_shading, sec_mask, sec_up, sec_vp, label);
        std::cout << "Loading files from " << label << "..."
                  << (load_success ? " success " : " fail ") << std::endl;
      }
    }

    counter++;
  }

  // Export Bundle Adjustment Problem
  std::cout << "Exporting" << std::endl;

  preparator.ExportFile(bundle_adjustment_problem_output_filename);
  std::string final_message =
      "Bundle Adjustment Problem Exported to " + bundle_adjustment_problem_output_filename;
  ROS_INFO("[DualCameraCalibrationPreparatorNode] %s", final_message.c_str());

  return 0;
}