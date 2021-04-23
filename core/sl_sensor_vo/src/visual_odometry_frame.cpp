#include "sl_sensor_vo/visual_odometry_frame.hpp"

namespace sl_sensor
{
namespace vo
{
void VisualOdometryFrame::RemoverUnreliableDepthKeypoints(double x_min, double x_max, double y_min, double y_max,
                                                          double z_min, double z_max)
{
  int num_kps_3d = kps_pc_ptr->size();

  size_t j = 0;
  for (size_t i = 0; i < num_kps_3d; ++i)
  {
    double x = kps_pc_ptr->points[i].x;
    double y = kps_pc_ptr->points[i].y;
    double z = kps_pc_ptr->points[i].z;

    bool has_nan_entries = !isfinite(x) || !isfinite(y) || !isfinite(z);

    bool is_outside_of_bounding_box =
        !(x_min <= x && x <= x_max) || !(y_min <= y && y <= y_max) || !(z_min <= z && z <= z_max);

    if (has_nan_entries || is_outside_of_bounding_box)
      continue;
    kps_pc_ptr->points[j] = kps_pc_ptr->points[i];
    kps[j] = kps[i];
    j++;
  }

  if (j != num_kps_3d)
  {
    // Resize to the correct size
    kps_pc_ptr->points.resize(j);
    kps.resize(j);
  }

  kps_pc_ptr->height = 1;
  kps_pc_ptr->width = static_cast<uint32_t>(j);
  kps_pc_ptr->is_dense = true;
}

void VisualOdometryFrame::Get3DPointsCV(std::vector<cv::Point3d>& output_vec)
{
  output_vec.clear();

  for (int i = 0; i < kps_pc_ptr->size(); i++)
  {
    output_vec.push_back(cv::Point3d{ kps_pc_ptr->points[i].x, kps_pc_ptr->points[i].y, kps_pc_ptr->points[i].z });
  }
}

}  // namespace vo

}  // namespace sl_sensor
