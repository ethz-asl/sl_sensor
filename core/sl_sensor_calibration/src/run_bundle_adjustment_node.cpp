/***************************************************************************************************
 * Multi-Camera Calibration Suite was built to help with intrinsic and extrinsic multi-camera
 * calibration. It also specifically contains a bundle adjustment module to help with the joint
 * calibration of the cameras.
 *
 * Copyright (c) 2017 Idiap Research Institute, http://www.idiap.ch/
 * Written by Salim Kayal <salim.kayal@idiap.ch>,
 *
 * This file is part of Multi-Camera Calibration Suite.
 *
 * Multi-Camera Calibration Suite is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Multi-Camera Calibration Suite is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Multi-Camera Calibration Suite. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#include <ros/ros.h>

#include "multicamera-calibration/BAProblem.hpp"
#include "multicamera-calibration/SnavelyReprojectionError.hpp"

#include "sl_sensor_calibration/calibration_utils.hpp"
#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <memory>

#include <omp.h>

using namespace sl_sensor::calibration;

bool SetOrdering(BAProblem& problem, ceres::Solver::Options& options, bool fix_intrinsics = false)
{
  const int num_points = problem.num_points();
  const int point_block_size = problem.point_block_size();
  double* points = problem.mutable_points();
  // const int num_fixed_points = problem.num_fixed_points();
  // double* fixed_points = problem.fixed_points();
  const int num_cameras = problem.num_cameras();
  const int camera_block_size = problem.camera_block_size();
  double* cameras = problem.mutable_cameras();
  ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
  ceres::ParameterBlockOrdering* inner_ordering = new ceres::ParameterBlockOrdering();
  // The points come before the cameras.
  // First argument is the pointer to the value to be optimised, Second argument is the group number
  for (int i = 0; i < num_points; ++i)
  {
    ordering->AddElementToGroup(points + point_block_size * i, 0);
    inner_ordering->AddElementToGroup(points + point_block_size * i, 0);
  }
  for (int i = 0; i < num_cameras; ++i)
  {
    ordering->AddElementToGroup(cameras + camera_block_size * i, 1);
    inner_ordering->AddElementToGroup(cameras + camera_block_size * i, 1);
    if (fix_intrinsics == false)
    {
      ordering->AddElementToGroup(cameras + 6 + camera_block_size * i, 2);
      inner_ordering->AddElementToGroup(cameras + 6 + camera_block_size * i, 2);
    }
  }
  options.linear_solver_ordering.reset(ordering);
  if (options.use_inner_iterations)
  {
    options.inner_iteration_ordering.reset(inner_ordering);
  }
  else
  {
    delete inner_ordering;
  }
  return true;
}

bool BuildCeresProblem(BAProblem& ba_problem, ceres::Problem& problem, bool fix_intrinsics = false)
{
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  const double* observations = ba_problem.observations();
  const int num_observations = ba_problem.num_observations();
  const double* fixed_observations = ba_problem.fixed_observations();
  const int num_fixed_observations = ba_problem.num_fixed_observations();
  for (int i = 0; i < ba_problem.num_observations(); ++i)
  {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1);
    double* mutable_camera = ba_problem.mutable_camera_for_observation(i);
    double* mutable_point = ba_problem.mutable_point_for_observation(i);
    problem.AddResidualBlock(cost_function, loss_function, mutable_camera, mutable_camera + 6, mutable_point);
  }
  for (int i = 0; i < ba_problem.num_fixed_observations(); ++i)
  {
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(fixed_observations[2 * i + 0], fixed_observations[2 * i + 1]);
    ceres::LossFunction* loss_function = new ceres::ScaledLoss(
        new ceres::HuberLoss(1), (double)num_observations / num_fixed_observations, ceres::TAKE_OWNERSHIP);
    double* mutable_camera = ba_problem.mutable_camera_for_fixed_observation(i);
    double* fixed_point = ba_problem.fixed_point_for_observation(i);
    problem.AddResidualBlock(cost_function, loss_function, mutable_camera, mutable_camera + 6, fixed_point);
    problem.SetParameterBlockConstant(fixed_point);
  }
  const int num_cameras = ba_problem.num_cameras();
  const int camera_block_size = ba_problem.camera_block_size();
  double* cameras = ba_problem.mutable_cameras();

  // We set extrinsics for first camera to be fixed
  problem.SetParameterBlockConstant(cameras);

  for (int i = 0; i < num_cameras; ++i)
  {
    if (fix_intrinsics)
    {
      problem.SetParameterBlockConstant(cameras + camera_block_size * i + 6);
    }
    else
    {
      problem.SetParameterBlockVariable(cameras + camera_block_size * i + 6);
    }
  }
  return true;
}

bool BuildCeresOptions(BAProblem& ba_problem, ceres::Solver::Options& options, bool fix_intrinsics = false)
{
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.use_inner_iterations = true;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 200000;
  options.num_threads = omp_get_max_threads();
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;
  options.max_num_consecutive_invalid_steps = 20;
  SetOrdering(ba_problem, options, fix_intrinsics);
  return true;
}

void ExtractCameraParametersFromBAProblem(BAProblem& ba_problem, int camera_index, cv::Matx33f& intrinsic_mat,
                                          cv::Vec<float, 5> lens_distortion, cv::Matx33f& rot_mat, cv::Vec3f& trans_vec)

{
  double* params = ba_problem.mutable_camera_for_observation(camera_index);

  cv::Mat rvec = cv::Mat(std::vector<float>{ (float)params[0], (float)params[1], (float)params[2] }, true);
  cv::Rodrigues(rvec, rot_mat);

  trans_vec = cv::Vec3f((float)params[3], (float)params[4], (float)params[5]);

  intrinsic_mat =
      cv::Matx33f((float)params[6], 0.0, (float)params[8], 0.0, (float)params[7], (float)params[9], 0.0, 0.0, 1.0);

  lens_distortion =
      cv::Vec<float, 5>((float)params[10], (float)params[11], (float)params[13], (float)params[14], (float)params[12]);
}

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "dual_camera_calibration_preparator");
  ros::NodeHandle nh_public;
  ros::NodeHandle private_nh("~");

  std::string input_ba_problem_file;
  std::string output_ba_problem_file;
  std::string intrinsic_adjustment;
  std::string residuals_save_folder;
  std::string pri_cam_parameters_file;
  std::string sec_cam_parameters_file;
  std::string proj_parameters_file;

  // Process parameters
  private_nh.param<std::string>("input_ba_problem_file", input_ba_problem_file, input_ba_problem_file);
  private_nh.param<std::string>("output_ba_problem_file", output_ba_problem_file, output_ba_problem_file);
  private_nh.param<std::string>("intrinsic_adjustment", intrinsic_adjustment,
                                intrinsic_adjustment);  // fixed, unconstrained or two_pass
  private_nh.param<std::string>("residuals_save_folder", residuals_save_folder, residuals_save_folder);
  private_nh.param<std::string>("pri_cam_parameters_file", pri_cam_parameters_file, pri_cam_parameters_file);
  private_nh.param<std::string>("sec_cam_parameters_file", sec_cam_parameters_file, sec_cam_parameters_file);
  private_nh.param<std::string>("proj_parameters_file", proj_parameters_file, proj_parameters_file);

  bool constrain = !(intrinsic_adjustment == "unconstrained");
  bool two_pass = (intrinsic_adjustment == "two_pass");

  // Create BAProblem
  std::cout << "open file " << input_ba_problem_file << "\n";
  BAProblem ba_problem(input_ba_problem_file);
  ceres::Problem* problem = new ceres::Problem;

  // Build ceres problem
  BuildCeresProblem(ba_problem, *problem, constrain);
  ceres::Solver::Options options;
  BuildCeresOptions(ba_problem, options, constrain);
  ceres::Solver::Summary summary;

  // Compute residuals before solving
  std::vector<double> initial_residuals;
  std::vector<int> camera_indices;

  for (size_t i = 0; i < (size_t)(ba_problem.num_observations() + ba_problem.num_fixed_observations()); i++)
  {
    camera_indices.push_back(ba_problem.GetCameraIndex(i));
  }

  double initial_cost;
  problem->Evaluate(ceres::Problem::EvaluateOptions(), &initial_cost, &initial_residuals, nullptr, nullptr);

  if (!std::string(residuals_save_folder).empty())
  {
    WriteResidualTextFiles(
        std::string(residuals_save_folder),
        { "pri_cam_residuals_initial.txt", "proj_residuals_initial.txt", "sec_cam_residuals_initial.txt" },
        initial_residuals, camera_indices);
  }

  // Solve BA problem
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // If second pass is desired, run BA problem again, but this time optimising for intrinsics as well
  if (two_pass)
  {
    delete problem;
    problem = new ceres::Problem;
    BuildCeresProblem(ba_problem, *problem);
    BuildCeresOptions(ba_problem, options);
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << "\n";
  }

  // Compute residuals after solving
  std::vector<double> final_residuals;
  double final_cost;
  problem->Evaluate(ceres::Problem::EvaluateOptions(), &final_cost, &final_residuals, nullptr, nullptr);
  if (!std::string(residuals_save_folder).empty())
  {
    WriteResidualTextFiles(std::string(residuals_save_folder),
                           { "pri_cam_residuals_final.txt", "proj_residuals_final.txt", "sec_cam_residuals_final.txt" },
                           final_residuals, camera_indices);
  }

  // Write camera and projector parameter files

  // Primary Camera
  cv::Matx33f pri_cam_rot_mat;
  cv::Vec3f pri_cam_trans;
  cv::Matx33f pri_cam_intrinsics;
  cv::Vec<float, 5> pri_cam_lens_distortion;
  ExtractCameraParametersFromBAProblem(ba_problem, 0, pri_cam_intrinsics, pri_cam_lens_distortion, pri_cam_rot_mat,
                                       pri_cam_trans);

  // Projector
  cv::Matx33f proj_rot_mat;
  cv::Vec3f proj_trans;
  cv::Matx33f proj_intrinsics;
  cv::Vec<float, 5> proj_lens_distortion;
  ExtractCameraParametersFromBAProblem(ba_problem, 1, proj_intrinsics, proj_lens_distortion, proj_rot_mat, proj_trans);

  // Secondary Camera
  cv::Matx33f rot_mat_sc_to_pc;
  cv::Vec3f trans_sc_to_pc;
  cv::Matx33f sec_cam_intrinsics;
  cv::Vec<float, 5> sec_cam_lens_distortion;
  ExtractCameraParametersFromBAProblem(ba_problem, 2, sec_cam_intrinsics, sec_cam_lens_distortion, rot_mat_sc_to_pc,
                                       trans_sc_to_pc);

  CameraParameters pri_cam_params(pri_cam_intrinsics, pri_cam_lens_distortion, 0, 0, 0, proj_rot_mat, proj_trans, 0);
  pri_cam_params.Save(pri_cam_parameters_file);

  ProjectorParameters proj_params(proj_intrinsics, proj_lens_distortion, 0, 0, 0);
  proj_params.Save(proj_parameters_file);

  cv::Matx33f sec_cam_rot_mat;
  cv::Vec3f sec_cam_trans;
  cv::Mat transformation_matrix_sec_cam_to_pri_cam = cv::Mat(3, 4, CV_32F, cv::Scalar(0.0));
  cv::Mat(rot_mat_sc_to_pc).copyTo(transformation_matrix_sec_cam_to_pri_cam(cv::Range(0, 3), cv::Range(0, 3)));
  cv::Mat(trans_sc_to_pc).copyTo(transformation_matrix_sec_cam_to_pri_cam(cv::Range(0, 3), cv::Range(3, 4)));
  cv::Mat transformation_matrix_sec_cam_to_projector =
      transformation_matrix_sec_cam_to_pri_cam * pri_cam_params.GetInverseTransformationMatrix();
  transformation_matrix_sec_cam_to_projector(cv::Range(0, 3), cv::Range(0, 3)).copyTo(sec_cam_rot_mat);
  transformation_matrix_sec_cam_to_projector(cv::Range(0, 3), cv::Range(3, 4)).copyTo(sec_cam_trans);

  CameraParameters sec_cam_params(sec_cam_intrinsics, sec_cam_lens_distortion, 0, 0, 0, sec_cam_rot_mat, sec_cam_trans,
                                  0);
  sec_cam_params.Save(sec_cam_parameters_file);

  // Clean up
  delete problem;

  if (!ba_problem.WriteFile(output_ba_problem_file))
  {
    std::cerr << "ERROR: unable to open file " << output_ba_problem_file << "\n";
    return 1;
  }

  return 0;
}
