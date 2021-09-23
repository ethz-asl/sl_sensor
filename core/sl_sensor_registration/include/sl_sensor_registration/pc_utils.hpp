#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sl_sensor {
namespace registration {
/**
 * @brief Voxel subsample point cloud
 *
 * @tparam PointT - Type of point in point cloud
 * @param input_cloud_ptr - Smart ptr to point cloud to be subsampled
 * @param leafsize - Leafsize of voxels
 * @return pcl::PointCloud<PointT>::Ptr - Smart pointer to subsampled point cloud
 */
template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr GetSubsampledPointCloud(
    typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr, double leafsize = 0.8f) {
  typedef typename pcl::PointCloud<PointT>::Ptr point_t_pc_ptr;
  typedef typename pcl::PointCloud<PointT> point_t_pc;
  typedef typename pcl::VoxelGrid<PointT> point_t_voxel_grid;

  point_t_pc_ptr output_cloud_ptr(new point_t_pc);

  if (leafsize != 0.0) {
    point_t_voxel_grid voxel_filter;
    voxel_filter.setInputCloud(input_cloud_ptr);
    voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
    voxel_filter.filter(*output_cloud_ptr);
  } else {
    *output_cloud_ptr = *input_cloud_ptr;
  }

  return output_cloud_ptr;
};

/**
 * @brief Scale a point cloud
 *
 * @tparam PointT - Type of pcl point
 * @param cloud_in - input cloud
 * @param cloud_out - output cloud
 * @param scale -scaling factor
 */
template <typename PointT>
inline void ScalePointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, double scale) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 0) = scale;
  transform(1, 1) = scale;
  transform(2, 2) = scale;
  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

/**
 * @brief Scale a point cloud, input is a smart pointer of original point cloud
 *
 * @tparam PointT - Type of pcl point
 * @param cloud_in_ptr - smart pointer of original point cloud
 * @param scale -scaling factor
 * @return pcl::PointCloud<PointT>::Ptr - Smart pointer of output point cloud
 */
template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr ScalePointCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud_in_ptr, double scale) {
  typedef typename pcl::PointCloud<PointT>::Ptr point_t_pc_ptr;
  typedef typename pcl::PointCloud<PointT> point_t_pc;

  point_t_pc_ptr transformed_ptr(new point_t_pc);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 0) = scale;
  transform(1, 1) = scale;
  transform(2, 2) = scale;
  pcl::transformPointCloud(*cloud_in_ptr, *transformed_ptr, transform);

  return transformed_ptr;
}

}  // namespace registration
}  // namespace sl_sensor