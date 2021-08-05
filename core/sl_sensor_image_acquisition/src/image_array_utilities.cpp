#include "sl_sensor_image_acquisition/image_array_utilities.hpp"

namespace sl_sensor
{
namespace image_acquisition
{
void PublishImageArray(ros::Publisher& publisher, const std::vector<sensor_msgs::ImageConstPtr>& image_vec,
                       const ros::Time& timestamp, const std::string& frame_id, int number_cameras)
{
  sl_sensor_image_acquisition::ImageArrayPtr img_arr_ptr =
      boost::make_shared<sl_sensor_image_acquisition::ImageArray>();

  img_arr_ptr->header.stamp = timestamp;
  img_arr_ptr->header.frame_id = frame_id;
  img_arr_ptr->number_cameras = number_cameras;

  for (const auto img_ptr : image_vec)
  {
    auto& img = *img_ptr;
    img_arr_ptr->data.emplace_back(std::move(img));
  }

  publisher.publish(img_arr_ptr);
}

void ConvertImgArrToCvPtrVec(const sl_sensor_image_acquisition::ImageArrayConstPtr image_array_ptr,
                             std::vector<cv_bridge::CvImageConstPtr>& cv_ptr_vec)
{
  cv_ptr_vec.clear();

  for (const auto& img : image_array_ptr->data)
  {
    cv_ptr_vec.push_back(cv_bridge::toCvShare(img, nullptr));
  }
}

void PublishCvMatVec(ros::Publisher& publisher, const std::vector<cv::Mat>& cv_mat_vec, const std::string& frame_id,
                     const ros::Time& array_timestamp, const ros::Time& image_timestamp,
                     const std::vector<std::string>& encodings_vec, int number_cameras)
{
  sl_sensor_image_acquisition::ImageArrayPtr output_message_ptr =
      boost::make_shared<sl_sensor_image_acquisition::ImageArray>();

  output_message_ptr->header.stamp = array_timestamp;
  output_message_ptr->header.frame_id = frame_id;
  output_message_ptr->data.resize(cv_mat_vec.size(), sensor_msgs::Image());
  output_message_ptr->number_cameras = number_cameras;

  std_msgs::Header image_header;
  image_header.stamp = image_timestamp;
  image_header.frame_id = frame_id;

  for (unsigned int i = 0; i < cv_mat_vec.size(); i++)
  {
    cv_bridge::CvImage(image_header, encodings_vec[i], cv_mat_vec[i]).toImageMsg(output_message_ptr->data[i]);
  }

  publisher.publish(output_message_ptr);
};

}  // namespace image_acquisition

}  // namespace sl_sensor