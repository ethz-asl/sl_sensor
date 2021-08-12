#include "sl_sensor_codec/decoder_nodelet.hpp"

#include "sl_sensor_codec/codec_factory.hpp"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

namespace sl_sensor
{
namespace codec
{
DecoderNodelet::DecoderNodelet(){};

std::vector<int> StringToIntVec(const std::string& s, const std::string& delimiter)
{
  std::vector<std::string> res_str;
  std::vector<int> res;
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res_str.push_back(token);
  }

  res_str.push_back(s.substr(pos_start));

  for (const auto& str : res_str)
  {
    res.push_back(std::stoi(str));
  }

  return res;
};

void DecoderNodelet::onInit()
{
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", image_array_sub_topic_, image_array_sub_topic_);
  private_nh_.param<std::string>("output_topic", decoded_pub_topic_, decoded_pub_topic_);
  private_nh_.param<std::string>("decoder_name", decoder_name_, decoder_name_);
  std::string cameras_to_decode_indices_str;
  private_nh_.param<std::string>("cameras_to_decode_indices", cameras_to_decode_indices_str,
                                 cameras_to_decode_indices_str);
  cameras_to_decode_indices_ = StringToIntVec(cameras_to_decode_indices_str, " ");

  private_nh_.param<bool>("colour_shading_enabled", colour_shading_enabled_, colour_shading_enabled_);
  private_nh_.param<int>("colour_image_index", colour_image_index_, colour_image_index_);
  private_nh_.param<int>("colour_camera_index", colour_camera_index_, colour_camera_index_);
  private_nh_.param<std::string>("codec_yaml_directory", codec_yaml_directory_, codec_yaml_directory_);
  private_nh_.param<std::string>("projector_yaml_directory", projector_yaml_directory_, projector_yaml_directory_);
  private_nh_.param<std::string>("direction", direction_, direction_);

  // Load YAML node and set some information from private node handle required to set up a decoder
  YAML::Node node = YAML::LoadFile(codec_yaml_directory_);

  if (!node)
  {
    ROS_WARN("[DecoderNodelet] Failed to load YAML file with codec information");
  }
  else
  {
    if (!node[decoder_name_])
    {
      std::string warning =
          "[DecoderNodelet] Codec YAML file does not have an entry with the decoder name " + decoder_name_;
      ROS_WARN("%s", warning.c_str());
    }

    node[decoder_name_]["direction"] = direction_;
    node[decoder_name_]["projector_yaml_directory"] = projector_yaml_directory_;
  }

  // Generate Decoder
  decoder_ptr_ = CodecFactory::GetInstanceDecoder(decoder_name_, node[decoder_name_]);

  // Setup publisher and subscriber
  decoded_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(decoded_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &DecoderNodelet::ImageArrayCb, this);
};

void DecoderNodelet::ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr)
{
  // We expect an equal number of images for each camera, and the number of images per camera should be equal or larger
  // (in the case there might be an additional image for colour) to the number of images required for the decoder to
  // operate
  bool equal_number_images_per_camera = ((int)image_array_ptr->data.size() % (int)image_array_ptr->number_cameras) == 0;
  int images_per_camera = (int)image_array_ptr->data.size() / (int)image_array_ptr->number_cameras;
  int images_required_for_decoder = decoder_ptr_->GetNumberPatterns();

  // Do not continue if image array does not contain the expected number of images
  if (!equal_number_images_per_camera || images_per_camera < images_required_for_decoder)
  {
    ROS_INFO("[DecoderNodelet] Number of images in image array does not match requirements for decoder. Ignoring this "
             "image array message");
    return;
  }

  // Convert image msg to cv img
  std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
  image_acquisition::ConvertImgArrToCvPtrVec(image_array_ptr, cv_img_ptr_vec);

  // Decode pattern sequence for each camera, store results
  int number_output_images = cameras_to_decode_indices_.size() * 4 + ((colour_shading_enabled_) ? 1 : 0);
  std::vector<cv::Mat> decoder_results(number_output_images, cv::Mat());

  for (size_t i = 0; i < cameras_to_decode_indices_.size(); i++)
  {
    std::vector<cv::Mat> frames;

    // For now, we assume that all the images required for decoding are at the front
    for (size_t j = 0; j < (size_t)images_required_for_decoder; j++)
    {
      frames.emplace_back(cv_img_ptr_vec.at(cameras_to_decode_indices_[i] * images_required_for_decoder + j)->image);
    }

    decoder_ptr_->SetFrames(frames);

    decoder_ptr_->DecodeFrames(decoder_results.at(i * 4), decoder_results.at(i * 4 + 1), decoder_results.at(i * 4 + 2),
                               decoder_results.at(i * 4 + 3));
  }

  // Assign last image in the output to be the one used for colouring
  if (colour_shading_enabled_)
  {
    decoder_results.back() = cv_img_ptr_vec.at(images_per_camera * colour_camera_index_ + colour_image_index_)->image;
  }

  // Construct output image format vector
  std::vector<std::string> output_image_format_vec = {};

  for (uint8_t i = 0; i < cameras_to_decode_indices_.size(); i++)
  {
    output_image_format_vec.push_back("32FC1");
    output_image_format_vec.push_back("32FC1");
    output_image_format_vec.push_back("8UC1");
    output_image_format_vec.push_back("8UC1");
  }

  if (colour_shading_enabled_)
  {
    output_image_format_vec.push_back("bgr8");
  }

  // Publish results
  image_acquisition::PublishCvMatVec(decoded_pub_, decoder_results, image_array_ptr->header.frame_id,
                                     image_array_ptr->header.stamp, ros::Time::now(), output_image_format_vec);
};

}  // namespace codec
}  // namespace sl_sensor
