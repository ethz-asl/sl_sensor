#include "sl_sensor_codec/decoder_nodelet.hpp"

#include "sl_sensor_codec/codec_factory.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

namespace sl_sensor
{
namespace codec
{
DecoderNodelet::DecoderNodelet(){};

void DecoderNodelet::onInit()
{
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", image_array_sub_topic_, image_array_sub_topic_);
  private_nh_.param<std::string>("output_topic", decoded_pub_topic_, decoded_pub_topic_);
  private_nh_.param<std::string>("decoder_name", decoder_name_, decoder_name_);
  private_nh_.param<int>("number_cameras", number_cameras_, number_cameras_);

  // Generate Decoder
  decoder_ptr_ = CodecFactory::GetInstanceDecoder(decoder_name_, private_nh_);

  // Setup publisher and subscriber
  decoded_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(decoded_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &DecoderNodelet::ImageArrayCb, this);
};

void DecoderNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array)
{
  // Do not continue if image array does not contain the expected number of images
  unsigned int number_patterns = decoder_ptr_->GetNumberPatterns();
  if (image_array->data.size() != number_cameras_ * number_patterns)
  {
    ROS_INFO("[DecoderNodelet] Number of images in image array does not correspond to (number of images required for "
             "the decoder * number cameras). Ignoring this image array message");
    return;
  }

  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
  image_acquisition::ConvertImgArrToCvPtrVec(image_array, cv_img_ptr_vec);

  // Decode pattern sequence for each camera, store results
  std::vector<cv::Mat> decoder_results;
  decoder_results.resize(number_cameras_ * 4, cv::Mat(0, 0, CV_8UC1));

  for (unsigned int i = 0; i < number_cameras_; i++)
  {
    for (unsigned int j = 0; j < number_patterns; j++)
    {
      decoder_ptr_->SetFrame(cv_img_ptr_vec[i * number_patterns + j]->image, j);
    }

    decoder_ptr_->DecodeFrames(decoder_results[i * number_patterns], decoder_results[i * number_patterns + 1],
                               decoder_results[i * number_patterns + 2], decoder_results[i * number_patterns + 3]);
  }

  // Publish results
  image_acquisition::PublishCvMatVec(decoded_pub_, decoder_results, image_array->header.frame_id,
                                     image_array->header.stamp, ros::Time::now());
};

}  // namespace codec
}  // namespace sl_sensor
