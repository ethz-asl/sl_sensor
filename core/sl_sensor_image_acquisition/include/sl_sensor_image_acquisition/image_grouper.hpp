#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace image_acquisition
{
/**
 * @brief Class object that helps to group images for a single camera
 *
 */
class ImageGrouper
{
public:
  /**
   * @brief Construct a new Image Grouper object
   *
   * @param image_sub_topic - Topic to subscribe to for Images
   * @param number_images_per_group - Number of images would be grouped together
   * @param image_trigger_period - Time between camera triggers from the versavis
   * @param lower_bound_tol - Images within the range -1.0 * lower_bound_tol <= target time <= upper_bound_tol will be
   * accepted
   * @param upper_bound_tol - See above
   */
  ImageGrouper(std::string image_sub_topic = "/image", int number_images_per_group = 1,
               double image_trigger_period = 0.01, double lower_bound_tol = 0.01, double upper_bound_tol = 0.01);

  /**
   * @brief Setup subscriber
   *
   * @param nh
   */
  void Init(ros::NodeHandle nh);

  /**
   * @brief Image Callback
   *
   * @param image_ptr
   */
  void ImageCb(const sensor_msgs::ImageConstPtr& image_ptr);

  /**
   * @brief To be called before the ImageGrouper start storing images
   *
   */
  void Start();

  /**
   * @brief To be called when ImageGroup is no longer in use (clears image buffer)
   *
   */
  void Stop();

  /**
   * @brief Set the Tolerance Bounds for accepting images
   *
   * @param lower_tol
   * @param upper_tol
   */
  void SetToleranceBounds(double lower_tol, double upper_tol);

  /**
   * @brief Set the number of Images in a group Group
   *
   * @param number_images_per_group
   */
  void SetImagesPerGroup(int number_images_per_group);

  /**
   * @brief Set the Image Trigger Period
   *
   * @param trigger_period
   */
  void SetImageTriggerPeriod(double trigger_period);

  /**
   * @brief Attempt to obtain a group of images that belongs to a given projector time
   *
   * @param projector_time - Time projector trigger
   * @param result_image_vec - If successful, this will be a non-empty vector with number_images_per_group_ number of
   * images
   * @param clear_image_buffer_if_successful - If successful retrieval, clear the image buffer up till the most recent
   * image in result_image_vec
   * @return true - Image group retrieval successful
   * @return false - Image group retrieval unsuccessful
   */
  bool RetrieveImageGroup(const ros::Time& projector_time, std::vector<sensor_msgs::ImageConstPtr>& result_image_vec,
                          bool clear_image_buffer_if_successful = true);

  /**
   * @brief Clear image buffer of all images before target_time
   *
   * @param target_time
   */
  void ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time);

  /**
   * @brief Clear entire Image Buffer
   *
   */
  void ClearBuffer();

  /**
   * @brief Get the Latest Image in the image buffer and clear it. If buffer is empty, returns a null pointer
   *
   * @return sensor_msgs::ImageConstPtr - Ptr to retrieved image
   */
  sensor_msgs::ImageConstPtr GetLatestImageAndClearBuffer();

private:
  /**
   * @brief Clear image buffer of all images before target_time (no mutex lock, to be managed internally)
   *
   * @param target_time
   */
  void ClearAllImagesFromBufferBeforeTimingNoLock(const ros::Time& target_time);

  /**
   * @brief Get the Image that belongs to a target time
   *
   * @param target_time - Target time
   * @param image_ptr - Pointer to image if successful
   * @return true - Image retrieval successful
   * @return false - Image retrieval unsuccessful
   */
  bool GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  boost::mutex mutex_;

  std::string image_sub_topic_;

  std::vector<sensor_msgs::ImageConstPtr> image_ptr_buffer_ = {};

  bool is_running_ = false;
  int number_images_per_group_;
  double image_trigger_period_;
  double lower_bound_tol_;
  double upper_bound_tol_;
};
}  // namespace image_acquisition
}  // namespace sl_sensor