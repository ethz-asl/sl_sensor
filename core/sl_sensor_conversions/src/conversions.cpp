#include "sl_sensor_conversions/conversions.hpp"

namespace sl_sensor
{
namespace conversions
{
geometry_msgs::Pose eigen_to_pose(const Eigen::Matrix4f &matrix)
{
  geometry_msgs::Pose pose_output;

  auto md = matrix.cast<double>();

  tf::Matrix3x3 tf3d;
  tf3d.setValue(md(0, 0), md(0, 1), md(0, 2), md(1, 0), md(1, 1), md(1, 2), md(2, 0), md(2, 1), md(2, 2));

  tf::Quaternion quat;
  tf3d.getRotation(quat);

  pose_output.position.x = md(0, 3);
  pose_output.position.y = md(1, 3);
  pose_output.position.z = md(2, 3);
  pose_output.orientation.x = quat.getX();
  pose_output.orientation.y = quat.getY();
  pose_output.orientation.z = quat.getZ();
  pose_output.orientation.w = quat.getW();

  return pose_output;
};

geometry_msgs::PoseStamped eigen_to_pose_stamped(const Eigen::Matrix4f &matrix, const ros::Time &time,
                                                 const std::string &frame_id)
{
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = time;
  pose_stamped.pose = eigen_to_pose(matrix);

  return pose_stamped;
};

Eigen::Matrix4f swap_frames_matrix(const Eigen::Matrix4f &mat)
{
  Eigen::Matrix3f RotSclInv =
      (mat.block<3, 3>(0, 0).array().rowwise() / mat.block<3, 3>(0, 0).colwise().squaredNorm().array()  // scaling
       )
          .transpose();                                                           // rotation
  return (Eigen::Matrix4f(4, 4) << RotSclInv, -RotSclInv * mat.block<3, 1>(0, 3)  // translation
          ,
          0, 0, 0, 1)
      .finished();
};

Eigen::Affine3d matrix4f_to_affine3d(const Eigen::Matrix4f &input)
{
  Eigen::Matrix4d md(input.cast<double>());
  Eigen::Affine3d affine(md);
  return affine;
};

Eigen::Matrix4f invert_transformation_matrix(const Eigen::Matrix4f &mat)
{
  Eigen::Matrix3f RotSclInv = (mat.block<3, 3>(0, 0)).transpose();  // Rotation
  return (Eigen::Matrix4f(4, 4) << RotSclInv, -RotSclInv * mat.block<3, 1>(0, 3), 0, 0, 0, 1).finished();
};

Eigen::Matrix4f remove_pitch_and_roll(const Eigen::Matrix4f &mat)
{
  Eigen::Matrix4f output_matrix = Eigen::Matrix4f::Identity();

  output_matrix(0, 3) = mat(0, 3);
  output_matrix(1, 3) = mat(1, 3);
  output_matrix(2, 3) = mat(2, 3);

  tf::Matrix3x3 rotation_matrix_tf;
  rotation_matrix_tf.setValue(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2), mat(2, 0), mat(2, 1),
                              mat(2, 2));

  double yaw = 0.0f;
  double pitch = 0.0f;
  double roll = 0.0f;

  rotation_matrix_tf.getEulerYPR(yaw, pitch, roll);
  rotation_matrix_tf.setEulerYPR(yaw, 0.0f, 0.0f);

  Eigen::Matrix3d rotation_matrix_eigen;
  tf::matrixTFToEigen(rotation_matrix_tf, rotation_matrix_eigen);

  output_matrix.block<3, 3>(0, 0) = rotation_matrix_eigen.cast<float>();

  return output_matrix;
}
}  // namespace conversions
}  // namespace sl_sensor