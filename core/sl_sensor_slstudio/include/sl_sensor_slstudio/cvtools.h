#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sl_sensor
{
namespace slstudio
{
namespace cvtools
{
cv::Point2d phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, cv::Point2d &shift);
cv::Mat logPolar(const cv::Mat &image, float scale);
void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size,
                    cv::Mat &map1, cv::Mat &map2);
cv::Mat diamondDownsample(cv::Mat &pattern);
void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
void imagesc(const char *windowName, cv::Mat im);
cv::Mat histimage(cv::Mat histogram);
void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
void writeMat(cv::Mat const &mat, const char *filename, const char *varName = "A", bool bgr2rgb = true);

std::vector<cv::Mat> read_image_sequence(const std::string &folder_directory, const std::string &file_title,
                                         const std::string &file_format, int frame_no, int pat_no_min, int pat_no_max);

char showImages(std::string title, std::vector<cv::Mat> &imgs, cv::Size cellSize);

void phase_correlate_register_image_sequence(const std::vector<cv::Mat> &image_sequence, int reference_index,
                                             std::vector<cv::Point2d> &shifts);

void phase_correlate_register_image_sequence(const std::vector<cv::Mat> &image_sequence, int reference_index,
                                             std::vector<cv::Point2d> &shifts, double subsample_factor);

void apply_shifts_to_image_sequence(std::vector<cv::Mat> &input_image_sequence,
                                    std::vector<cv::Mat> &output_image_sequence,
                                    const std::vector<cv::Point2d> &shifts);

void convert_mser_rects_to_keypoints(const std::vector<cv::Rect> &rect_vector, std::vector<cv::KeyPoint> &keypoint_vec);

void detect_mser_keypoints(cv::Ptr<cv::MSER> mser_detector_ptr, const cv::Mat &img,
                           std::vector<cv::KeyPoint> &keypoint_vec);

void get_descriptor_matches(cv::Ptr<cv::DescriptorMatcher> matcher_ptr, const cv::Mat &descriptors_static,
                            const cv::Mat &descriptors_moving, std::vector<cv::DMatch> &good_matches,
                            float ratio_thresh);

void cv_dmatches_to_pcl_correspondences(const std::vector<cv::DMatch> &dmatches, pcl::Correspondences &correspondences);

void pcl_correspondences_to_cv_dmatches(const pcl::Correspondences &correspondences, std::vector<cv::DMatch> &dmatches);

void update_correspondences_by_deviation_from_median(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                     const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                     pcl::Correspondences &correspondences);

void remove_nan_correspondences(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                const pcl::PointCloud<pcl::PointXYZ> &moving_pc, pcl::Correspondences &correspondences);

void subsample_image_sequence(const std::vector<cv::Mat> &image_sequence_input,
                              std::vector<cv::Mat> &image_sequence_output, double subsample_factor);

void keep_only_symmetric_matches(const std::vector<cv::DMatch> &matches12, const std::vector<cv::DMatch> &matches21,
                                 std::vector<cv::DMatch> &symmetric_matches);

template <typename T>
T get_median(const std::vector<T> &values)
{
  auto values_sorted = values;
  int N = values.size();
  std::sort(values_sorted.begin(), values_sorted.end());
  return (N % 2 == 0) ? ((values_sorted.at(N / 2 - 1) + values_sorted.at(N / 2))) / 2.0f : (values_sorted.at(N / 2));
}

void filter_correspondences_by_median_vertical_pixel_shift(const std::vector<cv::KeyPoint> &static_kps,
                                                           const std::vector<cv::KeyPoint> &moving_kps,
                                                           std::vector<cv::DMatch> &kp_matches, double threshold);

void filter_correspondences_by_median_3d_distances(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                   const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                   pcl::Correspondences &correspondences, double y_threshold,
                                                   double z_threshold, double l2_threshold);

}  // namespace cvtools

}  // namespace slstudio
}  // namespace sl_sensor