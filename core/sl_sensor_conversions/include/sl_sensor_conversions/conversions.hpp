#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace sl_sensor
{
namespace conversions
{
/**
 * @brief Converts Eigen 4x4 transformation matrix to geometry_msgs::Pose message
 *
 * @param matrix - Transformation matrix
 * @return geometry_msgs::Pose
 */
geometry_msgs::Pose eigen_to_pose(const Eigen::Matrix4f &matrix);

/**
 * @brief Converts Eigen 4x4 transformation matrix to geometry_msgs::PoseStamped message
 *
 * @param matrix - Transformation matrix
 * @param time - Time stamp
 * @param frame_id - Name of frame
 * @return geometry_msgs::PoseStamped
 */
geometry_msgs::PoseStamped eigen_to_pose_stamped(const Eigen::Matrix4f &matrix, const ros::Time &time,
                                                 const std::string &frame_id);

/**
 * @brief Takes a Eigen 4x4 transformation matrix from A to B and results transformation matrix from B to A
 *
 * @param mat - transformation matrix from A to B
 * @return Eigen::Matrix4f - transformation matrix from B to A
 */
Eigen::Matrix4f swap_frames_matrix(const Eigen::Matrix4f &mat);

/**
 * @brief Converts Transformation matrix from Eigen::Matrix4f to Eigen::Affine3d
 *
 * @param input
 * @return Eigen::Affine3d
 */
Eigen::Affine3d matrix4f_to_affine3d(const Eigen::Matrix4f &input);

// TODO Either use swap_frame_matrix or invert_transformation_matrix, they serve the same purpose
/**
 * @brief Inverts transformatiom matrix
 *
 * @param mat
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f invert_transformation_matrix(const Eigen::Matrix4f &mat);

/**
 * @brief Converts rotation matrix in input transformatiom matrix to YPR, sets P and R to be 0 and then converts it back
 * to a rotation matrix
 *
 * @param mat
 * @return Eigen::Matrix4f - Transformation matrix where rotation matrix has roll and pitch angles set to 0
 */
Eigen::Matrix4f remove_pitch_and_roll(const Eigen::Matrix4f &mat);

}  // namespace conversions
}  // namespace sl_sensor