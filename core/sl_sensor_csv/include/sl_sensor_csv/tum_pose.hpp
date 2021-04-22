#pragma once
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <cstdint>

namespace sl_sensor
{
namespace csv
{
/**
 * @brief Struct used to encapsulate a pose in TUM format
 *
 */
struct TumPose
{
  uint64_t timestamp = 0;
  double x = 0.0f;
  double y = 0.0f;
  double z = 0.0f;
  double qx = 0.0f;
  double qy = 0.0f;
  double qz = 0.0f;
  double qw = 1.0f;

  /**
   * @brief Construct a new Tum Pose object
   *
   * @param _timestamp
   * @param _x
   * @param _y
   * @param _z
   * @param _qx
   * @param _qy
   * @param _qz
   * @param _qw
   */
  TumPose(uint64_t _timestamp = 0, double _x = 0.0f, double _y = 0.0f, double _z = 0.0f, double _qx = 0.0f,
          double _qy = 0.0f, double _qz = 0.0f, double _qw = 1.0f)
    : timestamp(_timestamp), x(_x), y(_y), z(_z), qx(_qx), qy(_qy), qz(_qz), qw(_qw)
  {
  }

  /**
   * @brief Construct a new Tum Pose object using a 4x4 eigen transformation matrix
   *
   * @param ts
   * @param mat
   */
  TumPose(uint64_t ts, const Eigen::Matrix4f& mat)
  {
    set(ts, mat);
  }

  /**
   * @brief Convert tum pose to a 4x4 eigen transformation matrix
   *
   * @param mat
   */
  void GetTransformationMatrix(Eigen::Matrix4f& mat)
  {
    tf::Matrix3x3 R;
    tf::Quaternion q(qx, qy, qz, qw);
    R.setRotation(q);

    mat = Eigen::Matrix4f::Identity();

    auto row_1 = R.getRow(0);
    auto row_2 = R.getRow(1);
    auto row_3 = R.getRow(2);

    mat(0, 0) = row_1.getX();
    mat(0, 1) = row_1.getY();
    mat(0, 2) = row_1.getZ();

    mat(1, 0) = row_2.getX();
    mat(1, 1) = row_2.getY();
    mat(1, 2) = row_2.getZ();

    mat(2, 0) = row_3.getX();
    mat(2, 1) = row_3.getY();
    mat(2, 2) = row_3.getZ();

    mat(0, 3) = x;
    mat(1, 3) = y;
    mat(2, 3) = z;
  }

  /**
   * @brief Set TumPose based on a 4x4 eigen transformation matrix
   *
   * @param ts - timestamp
   * @param mat
   */
  void set(uint64_t ts, const Eigen::Matrix4f& mat)
  {
    timestamp = ts;

    tf::Matrix3x3 tf3d;
    tf3d.setValue(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));

    tf::Quaternion quat;
    tf3d.getRotation(quat);

    x = mat(0, 3);
    y = mat(1, 3);
    z = mat(2, 3);
    qx = quat.getX();
    qy = quat.getY();
    qz = quat.getZ();
    qw = quat.getW();
  }

  void ScaleTransform(double scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
  }
};

}  // namespace csv
}  // namespace sl_sensor