#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "sl_sensor_slstudio/cvtools.h"

#ifdef _WIN32
#include <cstdint>
#endif

#include <stdio.h>
#include <algorithm>
#include <cmath>

namespace sl_sensor
{
namespace slstudio
{
namespace cvtools
{
void phase_correlate_register_image_sequence(const std::vector<cv::Mat> &image_sequence, int reference_indice,
                                             std::vector<cv::Point2d> &shifts)

{
  shifts.clear();

  for (int i = 0; i < image_sequence.size(); i++)
  {
    if (i != reference_indice)
    {
      shifts.push_back(cv::phaseCorrelate(image_sequence[reference_indice], image_sequence[i]));
    }
    else
    {
      shifts.push_back(cv::Point2d(0.0f, 0.0f));
    }
  }
}

void phase_correlate_register_image_sequence(const std::vector<cv::Mat> &image_sequence, int reference_indice,
                                             std::vector<cv::Point2d> &shifts, double subsample_factor)

{
  if (subsample_factor <= 0.0f)
  {
    phase_correlate_register_image_sequence(image_sequence, reference_indice, shifts);
  }
  else
  {
    std::vector<cv::Mat> subsampled_image_sequence = {};
    subsample_image_sequence(image_sequence, subsampled_image_sequence, subsample_factor);
    phase_correlate_register_image_sequence(subsampled_image_sequence, reference_indice, shifts);

    for (auto &shift : shifts)
    {
      shift /= subsample_factor;
    }
  }
}

void apply_shifts_to_image_sequence(std::vector<cv::Mat> &input_image_sequence,
                                    std::vector<cv::Mat> &output_image_sequence, const std::vector<cv::Point2d> &shifts)
{
  output_image_sequence.clear();

  for (int i = 0; i < shifts.size(); i++)
  {
    output_image_sequence.push_back(cv::Mat::zeros(input_image_sequence[i].size(), input_image_sequence[i].depth()));

    float x = shifts[i].x;
    float y = shifts[i].y;
    bool non_zero_shift = !(x == 0.0f && y == 0.0f);

    // std::cout << "Shifts: " << x << " | " << y << std::endl;

    if (non_zero_shift)
    {
      cv::Mat M = (cv::Mat_<float>(2, 3) << 1.0f, 0.0f, x, 0.0f, 1.0f, y);
      // std::cout << M << std::endl;
      cv::warpAffine(input_image_sequence[i], output_image_sequence[i], M, input_image_sequence[i].size(),
                     cv::WARP_INVERSE_MAP);
    }
    else
    {
      input_image_sequence[i].copyTo(output_image_sequence[i]);
    }
  }
}

void convert_mser_rects_to_keypoints(const std::vector<cv::Rect> &rect_vec, std::vector<cv::KeyPoint> &keypoint_vec)
{
  keypoint_vec.clear();

  for (const auto &rect : rect_vec)
  {
    keypoint_vec.push_back(
        cv::KeyPoint(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f, std::max(rect.width, rect.height)));
  }
}

void detect_mser_keypoints(cv::Ptr<cv::MSER> mser_detector_ptr, const cv::Mat &img,
                           std::vector<cv::KeyPoint> &keypoint_vec)
{
  // cv::Mat img_8uc;
  // img.convertTo(img_8uc, CV_8UC1, 255.0f);
  std::vector<std::vector<cv::Point>> regions;
  std::vector<cv::Rect> mser_rects;
  mser_detector_ptr->detectRegions(img, regions, mser_rects);
  convert_mser_rects_to_keypoints(mser_rects, keypoint_vec);
}

void get_descriptor_matches(cv::Ptr<cv::DescriptorMatcher> matcher_ptr, const cv::Mat &descriptors_static,
                            const cv::Mat &descriptors_moving, std::vector<cv::DMatch> &good_matches,
                            float ratio_thresh)
{
  // float max_match_distance = 0.25f;
  // bool cross_check = false;
  std::vector<std::vector<cv::DMatch>> knn_matches;
  // cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, cross_check);
  // cv::Ptr<cv::DescriptorMatcher> matcher_ptr = cv::FlannBasedMatcher::create();

  // matcher->radiusMatch(descriptors_static, descriptors_moving, knn_matches, max_match_distance);
  matcher_ptr->knnMatch(descriptors_static, descriptors_moving, knn_matches, 2);

  //-- Filter matches using the Lowe's ratio test
  for (size_t i = 0; i < knn_matches.size(); i++)
  {
    if (knn_matches[i].size() == 2)
    {
      // std::cout << knn_matches[i][0].distance << " " << knn_matches[i][1].distance << std::endl;

      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
      {
        good_matches.push_back(knn_matches[i][0]);
      }
    }
  }
}

void keep_only_symmetric_matches(const std::vector<cv::DMatch> &matches12, const std::vector<cv::DMatch> &matches21,
                                 std::vector<cv::DMatch> &symmetric_matches)
{
  symmetric_matches.clear();

  for (const auto &match12 : matches12)
  {
    for (const auto &match21 : matches21)
    {
      // Match symmetry test
      if ((match12.queryIdx == match21.trainIdx) && (match12.queryIdx == match21.trainIdx))
      {
        // add symmetrical match
        symmetric_matches.push_back(cv::DMatch(match12.queryIdx, match12.trainIdx, match12.distance));
        break;
      }
    }
  }
}

void cv_dmatches_to_pcl_correspondences(const std::vector<cv::DMatch> &dmatches, pcl::Correspondences &correspondences)
{
  correspondences.clear();

  for (const auto &dmatch : dmatches)
  {
    correspondences.push_back(pcl::Correspondence(dmatch.queryIdx, dmatch.trainIdx, dmatch.distance));
  }
}

void pcl_correspondences_to_cv_dmatches(const pcl::Correspondences &correspondences, std::vector<cv::DMatch> &dmatches)
{
  dmatches.clear();

  for (const auto &correspondence : correspondences)
  {
    dmatches.push_back(cv::DMatch(correspondence.index_query, correspondence.index_match, correspondence.distance));
  }
}

void filter_correspondences_by_median_3d_distances(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                   const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                   pcl::Correspondences &correspondences, double y_threshold,
                                                   double z_threshold, double l2_threshold)
{
  // We do not process if correspondences is empty (get_median function will complain)
  if (correspondences.empty())
  {
    return;
  }

  std::vector<double> y_shifts = {};
  std::vector<double> z_shifts = {};
  std::vector<double> l2_shifts = {};

  // std::cout << "Computing median" << std::endl;

  // Get vector of euclidian distances
  for (const auto &correspondence : correspondences)
  {
    pcl::PointXYZ source_pt = static_pc.at(correspondence.index_query);
    pcl::PointXYZ target_pt = moving_pc.at(correspondence.index_match);
    float l2_dist = std::sqrt(pow(source_pt.x - target_pt.x, 2) + pow(source_pt.y - target_pt.y, 2) +
                              pow(source_pt.z - target_pt.z, 2));
    l2_shifts.push_back(l2_dist);
    y_shifts.push_back(source_pt.y - target_pt.y);
    z_shifts.push_back(source_pt.z - target_pt.z);
  }

  // std::cout << "Get Median" << std::endl;

  double median_l2 = get_median(l2_shifts);
  double median_y = get_median(y_shifts);
  double median_z = get_median(z_shifts);

  // std::cout << "Filtering" << std::endl;

  size_t j = 0;
  for (size_t i = 0; i < y_shifts.size(); ++i)
  {
    bool l2_outside_threshold = l2_shifts[i] > median_l2 + l2_threshold || l2_shifts[i] < median_l2 - l2_threshold;
    bool y_shift_outside_threshold = y_shifts[i] > median_y + y_threshold || y_shifts[i] < median_y - y_threshold;
    bool z_shift_outside_threshold = z_shifts[i] > median_z + z_threshold || z_shifts[i] < median_z - z_threshold;

    if (l2_outside_threshold || y_shift_outside_threshold || z_shift_outside_threshold)
      continue;

    correspondences[j] = correspondences[i];
    j++;
  }

  // std::cout << "Resizing" << std::endl;

  correspondences.resize(j);
}

void filter_correspondences_by_median_vertical_pixel_shift(const std::vector<cv::KeyPoint> &static_kps,
                                                           const std::vector<cv::KeyPoint> &moving_kps,
                                                           std::vector<cv::DMatch> &kp_matches, double threshold)
{
  // We do not process if correspondences is empty (get_median function will complain)
  if (kp_matches.empty())
  {
    return;
  }

  std::vector<double> vertical_pixel_shifts;

  for (const auto &kp_match : kp_matches)
  {
    vertical_pixel_shifts.push_back(static_kps[kp_match.queryIdx].pt.y - moving_kps[kp_match.trainIdx].pt.y);
  }

  double median_shift = get_median(vertical_pixel_shifts);

  size_t j = 0;
  for (size_t i = 0; i < vertical_pixel_shifts.size(); ++i)
  {
    if (vertical_pixel_shifts[i] > median_shift + threshold || vertical_pixel_shifts[i] < median_shift - threshold)
      continue;
    kp_matches[j] = kp_matches[i];
    j++;
  }
  kp_matches.resize(j);
}

void update_correspondences_by_deviation_from_median(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                                     const pcl::PointCloud<pcl::PointXYZ> &moving_pc,
                                                     pcl::Correspondences &correspondences)
{
  std::vector<float> l2_distance;  // Can improve by using an array instead
  std::vector<float> l2_distance_filtered;
  int N = correspondences.size();

  // Get vector of euclidian distances
  for (const auto &correspondence : correspondences)
  {
    pcl::PointXYZ source_pt = static_pc[correspondence.index_query];
    pcl::PointXYZ target_pt = moving_pc[correspondence.index_match];
    float l2_dist = std::sqrt(pow(source_pt.x - target_pt.x, 2) + pow(source_pt.y - target_pt.y, 2) +
                              pow(source_pt.z - target_pt.z, 2));
    l2_distance.push_back(l2_dist);

    if (!std::isnan(l2_dist))
    {
      l2_distance_filtered.push_back(l2_dist);
    }
  }

  // Compute median
  std::sort(l2_distance_filtered.begin(), l2_distance_filtered.end());
  float median = get_median(l2_distance_filtered);
  // std::cout << "Median" << median << std::endl;

  // 'Distance' is abs deviation from median
  for (int i = 0; i < N; i++)
  {
    correspondences[i].distance = std::abs(l2_distance[i] - median);
    // std::cout << correspondences[i].distance << std::endl;
  }
}

void remove_nan_correspondences(const pcl::PointCloud<pcl::PointXYZ> &static_pc,
                                const pcl::PointCloud<pcl::PointXYZ> &moving_pc, pcl::Correspondences &correspondences)
{
  pcl::Correspondences filtered_correspondences;

  for (const auto &correspondence : correspondences)
  {
    pcl::PointXYZ source_pt = static_pc[correspondence.index_query];
    pcl::PointXYZ target_pt = moving_pc[correspondence.index_match];

    bool not_nan_entry = !(isnan(source_pt.x) || isnan(source_pt.y) || isnan(source_pt.z) || isnan(target_pt.x) ||
                           isnan(target_pt.y) || isnan(target_pt.z));

    if (not_nan_entry)
    {
      filtered_correspondences.push_back(correspondence);
    }
  }

  correspondences = filtered_correspondences;
}

void subsample_image_sequence(const std::vector<cv::Mat> &image_sequence_input,
                              std::vector<cv::Mat> &image_sequence_output, double subsample_factor)
{
  image_sequence_output.clear();

  for (const auto &input_image : image_sequence_input)
  {
    image_sequence_output.push_back(cv::Mat{ input_image.size(), input_image.type() });
    cv::resize(input_image, image_sequence_output.back(), cv::Size(), subsample_factor, subsample_factor,
               cv::InterpolationFlags::INTER_NEAREST);
  }
}

}  // namespace cvtools
}  // namespace slstudio
}  // namespace sl_sensor