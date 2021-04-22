#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sl_sensor
{
namespace registration
{
/**
 * @brief Voxel subsample point cloud
 *
 * @tparam PointT - Type of point in point cloud
 * @param input_cloud_ptr - Smart ptr to point cloud to be subsampled
 * @param leafsize - Leafsize of voxels
 * @return pcl::PointCloud<PointT>::Ptr - Smart pointer to subsampled point cloud
 */
template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr
GetSubsampledPointCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr, double leafsize = 0.8f)
{
  typedef typename pcl::PointCloud<PointT>::Ptr point_t_pc_ptr;
  typedef typename pcl::PointCloud<PointT> point_t_pc;
  typedef typename pcl::VoxelGrid<PointT> point_t_voxel_grid;

  point_t_pc_ptr output_cloud_ptr(new point_t_pc);

  if (leafsize != 0.0)
  {
    point_t_voxel_grid voxel_filter;
    voxel_filter.setInputCloud(input_cloud_ptr);
    voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
    voxel_filter.filter(*output_cloud_ptr);
  }
  else
  {
    *output_cloud_ptr = *input_cloud_ptr;
  }

  return output_cloud_ptr;
};

}  // namespace registration
}  // namespace sl_sensor