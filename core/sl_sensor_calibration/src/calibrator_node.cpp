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

#include <sl_sensor_projector/projector_utils.hpp>

#include "sl_sensor_calibration/calibration_option.hpp"
#include "sl_sensor_calibration/calibration_utils.hpp"
#include "sl_sensor_calibration/calibrator.hpp"
#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

using namespace sl_sensor::calibration;

bool DirectoryExists(const char* path)
{
  struct stat info;

  if (stat(path, &info) != 0)
    return false;
  else if (info.st_mode & S_IFDIR)
    return true;
  else
    return false;
};

std::vector<std::string> SplitString(const std::string& s, const std::string& delimiter)
{
  std::vector<std::string> res;
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));

  return res;
};

bool GetCvMatFromXml(const std::string& filename, const std::string& param_name, cv::Mat& mat)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    return false;
  }

  fs[param_name] >> mat;

  return mat.empty() ? false : true;
};

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "calibrator");
  ros::NodeHandle nh_public;
  ros::NodeHandle private_nh("~");

  // Load params from private ros node
  std::string camera_folder_name;
  std::string projector_folder_name;
  std::string directories;
  std::vector<std::string> directories_vec;
  std::vector<std::unordered_set<std::string>> files_to_ignore_vec;
  std::string projector_yaml_directory = "";
  std::string output_camera_parameters_filename = "";
  std::string output_projector_parameters_filename = "";
  std::string residuals_save_folder = "";
  std::string residuals_file_label = "";

  unsigned int projector_cols = 0;
  unsigned int projector_rows = 0;

  CalibrationOption camera_calibration_option;
  std::string camera_calibration_init_yaml = "";
  CalibrationOption projector_calibration_option;
  std::string projector_calibration_init_yaml = "";

  int checkerboard_rows = 0;
  int checkerboard_cols = 0;
  double checkerboard_size = 10;

  int window_radius = 10;
  int minimum_valid_pixels = 50;

  private_nh.param<std::string>("camera_folder_name", camera_folder_name, camera_folder_name);
  private_nh.param<std::string>("projector_folder_name", projector_folder_name, projector_folder_name);
  private_nh.param<std::string>("directories", directories, directories);
  private_nh.param<std::string>("projector_yaml_directory", projector_yaml_directory, projector_yaml_directory);

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
  private_nh.param<std::string>("camera_calibration_init_yaml", camera_calibration_init_yaml,
                                camera_calibration_init_yaml);

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
  private_nh.param<std::string>("projector_calibration_init_yaml", projector_calibration_init_yaml,
                                projector_calibration_init_yaml);

  private_nh.param<int>("checkerboard_rows", checkerboard_rows, checkerboard_rows);
  private_nh.param<int>("checkerboard_cols", checkerboard_cols, checkerboard_cols);
  private_nh.param<double>("checkerboard_size", checkerboard_size, checkerboard_size);

  private_nh.param<int>("window_radius", window_radius, window_radius);
  private_nh.param<int>("minimum_valid_pixels", minimum_valid_pixels, minimum_valid_pixels);

  private_nh.param<std::string>("output_camera_parameters_filename", output_camera_parameters_filename,
                                output_camera_parameters_filename);
  private_nh.param<std::string>("output_projector_parameters_filename", output_projector_parameters_filename,
                                output_projector_parameters_filename);

  private_nh.param<std::string>("residuals_save_folder", residuals_save_folder, residuals_save_folder);
  private_nh.param<std::string>("residuals_file_label", residuals_file_label, residuals_file_label);

  CameraParameters camera_calibration_data_init;

  if ((camera_calibration_option.use_initial_guess || camera_calibration_option.fix_values) &&
      camera_calibration_data_init.Load(camera_calibration_init_yaml))
  {
    camera_calibration_option.intrinsics_init = camera_calibration_data_init.intrinsic_mat();
    camera_calibration_option.lens_distortion_init = camera_calibration_data_init.lens_distortion();
  }

  ProjectorParameters projector_calibration_data_init;

  if ((projector_calibration_option.use_initial_guess || projector_calibration_option.fix_values) &&
      projector_calibration_data_init.Load(projector_calibration_init_yaml))
  {
    projector_calibration_option.intrinsics_init = projector_calibration_data_init.intrinsic_mat();
    projector_calibration_option.lens_distortion_init = projector_calibration_data_init.lens_distortion();
  }

  std::string delimiter = " ";
  directories_vec = SplitString(directories, delimiter);

  for (unsigned int i = 1; i <= directories_vec.size(); i++)
  {
    std::string ignore_param_name = "ignore_" + std::to_string(i);
    std::string temp_string = "";
    private_nh.param<std::string>(ignore_param_name, temp_string, temp_string);

    std::vector<std::string> ignore_filenames_vec = SplitString(temp_string, delimiter);

    std::unordered_set<std::string> ignore_filenames;

    for (const auto& filenames : ignore_filenames_vec)
    {
      ignore_filenames.insert(filenames);
    }

    files_to_ignore_vec.push_back(ignore_filenames);
  }

  // Get resolution of projector using YAML file
  sl_sensor::projector::GetProjectorResolution(projector_yaml_directory, projector_rows, projector_cols);

  // Initialise calibrator
  Calibrator calibrator;
  calibrator.SetProjectorResolution(projector_cols, projector_rows);
  calibrator.SetCheckerboardInformation(checkerboard_cols, checkerboard_rows, checkerboard_size);
  calibrator.SetCameraCalibrationOption(camera_calibration_option);
  calibrator.SetProjectorCalibrationOption(projector_calibration_option);
  calibrator.SetLocalHomographySettings(window_radius, minimum_valid_pixels);

  // Add calibration sequences to calibrator
  int counter = 0;
  for (const auto& directory : directories_vec)
  {
    // Check if the required folders exist
    std::string shading_directory = directory + camera_folder_name + "/shading";
    std::string mask_directory = directory + camera_folder_name + "/mask";
    std::string up_directory = directory + projector_folder_name + "/up";
    std::string vp_directory = directory + projector_folder_name + "/vp";

    if (!DirectoryExists(shading_directory.c_str()))
    {
      std::string status_message =
          "[CalibratorNode] Shading Directory " + shading_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(mask_directory.c_str()))
    {
      std::string status_message =
          "[CalibratorNode] Mask Directory " + mask_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(up_directory.c_str()))
    {
      std::string status_message =
          "[CalibratorNode] Up Directory " + up_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    if (!DirectoryExists(vp_directory.c_str()))
    {
      std::string status_message =
          "[CalibratorNode] Vp Directory " + vp_directory + " does not exist, skipping this folder";
      ROS_INFO("%s", status_message.c_str());
      continue;
    }

    for (const auto& entry : std::experimental::filesystem::directory_iterator(shading_directory))
    {
      std::string stem = entry.path().stem().u8string();

      // If stem is in the list of filenames to ignore, skip to next entry
      if (files_to_ignore_vec[counter].find(stem) != files_to_ignore_vec[counter].end())
      {
        std::cout << "Ignoring files named " << stem << std::endl;
        continue;
      }

      // Load cv::Mat from files
      cv::Mat shading;
      cv::Mat mask;
      cv::Mat up;
      cv::Mat vp;

      shading = cv::imread(entry.path().u8string(), CV_8UC1);
      mask = cv::imread(mask_directory + "/" + stem + ".bmp", CV_8UC1);

      bool shading_success = !shading.empty();
      bool mask_success = !mask.empty();
      bool up_success = GetCvMatFromXml(up_directory + "/" + stem + ".xml", "up", up);
      bool vp_success = GetCvMatFromXml(vp_directory + "/" + stem + ".xml", "vp", vp);

      // If all read successfully, add to calibrator
      if (shading_success && mask_success && up_success && vp_success)
      {
        std::string label = directory + "|" + stem;
        bool load_success = calibrator.AddSingleCalibrationSequence(shading, mask, up, vp, label);
        std::cout << "Loading files from " << label << "..." << (load_success ? " success " : " fail ") << std::endl;
      }
    }

    counter++;
  }

  // Perform calibration and save to calibration file

  ProjectorParameters projector_parameters;
  CameraParameters camera_parameters;
  std::vector<double> cam_residuals;
  std::vector<double> proj_residuals;

  bool calibration_success =
      calibrator.Calibrate(projector_parameters, camera_parameters, proj_residuals, cam_residuals);

  bool save_residuals = (!residuals_save_folder.empty() && !residuals_file_label.empty());

  if (calibration_success)
  {
    ROS_INFO("[CalibratorNode] Calibration was successful");

    if (!projector_calibration_option.fix_values)
    {
      if (projector_parameters.Save(output_projector_parameters_filename))
      {
        ROS_INFO("[CalibratorNode] Projector parameters saved");
      }
      else
      {
        ROS_INFO("[CalibratorNode] Failed to save projector parameter file");
      }
    }

    if (!camera_calibration_option.fix_values)
    {
      if (camera_parameters.Save(output_camera_parameters_filename))
      {
        ROS_INFO("[CalibratorNode] Camera parameters saved");
      }
      else
      {
        ROS_INFO("[CalibratorNode] Failed to save camera parameter file");
      }
    }

    if (save_residuals)
    {
      WriteResidualTextFile(residuals_save_folder, "cam_residuals_" + residuals_file_label + ".txt", cam_residuals);
      WriteResidualTextFile(residuals_save_folder, "proj_residuals_" + residuals_file_label + ".txt", proj_residuals);
    }
  }
  else
  {
    ROS_INFO("[CalibratorNode] Calibration was unsuccessful");
  }

  return 0;
}