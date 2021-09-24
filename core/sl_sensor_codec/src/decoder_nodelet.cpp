/***************************************************************************************************
 * This file is part of sl_sensor.
 *
 * sl_sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sl_sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sl_sensor.  If not, see <https://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#include "sl_sensor_codec/decoder_nodelet.hpp"

#include "sl_sensor_codec/codec_factory.hpp"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

namespace sl_sensor {
namespace codec {
DecoderNodelet::DecoderNodelet(){};

std::vector<int> StringToIntVec(const std::string& s, const std::string& delimiter) {
  std::vector<std::string> res_str;
  std::vector<int> res;
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res_str.push_back(token);
  }

  res_str.push_back(s.substr(pos_start));

  for (const auto& str : res_str) {
    res.push_back(std::stoi(str));
  }

  return res;
};

void DecoderNodelet::onInit() {
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

  private_nh_.param<bool>("colour_shading_enabled", colour_shading_enabled_,
                          colour_shading_enabled_);
  private_nh_.param<int>("colour_image_index", colour_image_index_, colour_image_index_);
  private_nh_.param<int>("colour_camera_index", colour_camera_index_, colour_camera_index_);
  private_nh_.param<std::string>("codec_yaml_directory", codec_yaml_directory_,
                                 codec_yaml_directory_);
  private_nh_.param<std::string>("projector_yaml_directory", projector_yaml_directory_,
                                 projector_yaml_directory_);
  private_nh_.param<std::string>("direction", direction_, direction_);
  private_nh_.param<std::string>("filter_id", filter_id_, filter_id_);

  // Load YAML node and set some information from private node handle required to set up a decoder
  YAML::Node node = YAML::LoadFile(codec_yaml_directory_);

  if (!node) {
    ROS_WARN("[DecoderNodelet] Failed to load YAML file with codec information");
  } else {
    if (!node[decoder_name_]) {
      std::string warning =
          "[DecoderNodelet] Codec YAML file does not have an entry with the decoder name " +
          decoder_name_;
      ROS_WARN("%s", warning.c_str());
    }

    node[decoder_name_]["direction"] = direction_;
    node[decoder_name_]["projector_yaml_directory"] = projector_yaml_directory_;
  }

  // Generate Decoder
  decoder_ptr_ = CodecFactory::GetInstanceDecoder(decoder_name_, node[decoder_name_]);

  // Generate output image format vec
  number_output_images_ =
      cameras_to_decode_indices_.size() * 4 + ((colour_shading_enabled_) ? 1 : 0);
  output_image_format_vec_ = std::vector(number_output_images_, std::string());

  for (size_t i = 0; i < cameras_to_decode_indices_.size(); i++) {
    output_image_format_vec_[i * 4] = "32FC1";
    output_image_format_vec_[i * 4 + 1] = "32FC1";
    output_image_format_vec_[i * 4 + 2] = "8UC1";
    output_image_format_vec_[i * 4 + 3] = "8UC1";
  }

  if (colour_shading_enabled_) {
    output_image_format_vec_.back() = "bgr8";
  }

  // Setup publisher and subscriber
  decoded_pub_ = nh_.advertise<sl_sensor_image_acquisition::ImageArray>(decoded_pub_topic_, 10);
  image_array_sub_ = nh_.subscribe(image_array_sub_topic_, 10, &DecoderNodelet::ImageArrayCb, this);
};

void DecoderNodelet::ImageArrayCb(
    const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr) {
  // If a filter id is specified, make sure that the id from the message is correct before
  // continuing with processing
  if (filter_id_.empty() || (!filter_id_.empty() && filter_id_ == image_array_ptr->id)) {
    // We expect an equal number of images for each camera, and the number of images per camera
    // should be equal or larger (in the case there might be an additional image for colour) to the
    // number of images required for the decoder to operate
    bool equal_number_images_per_camera =
        (image_array_ptr->data.size() % (size_t)image_array_ptr->number_cameras) == 0;
    size_t images_per_camera = image_array_ptr->data.size() / image_array_ptr->number_cameras;
    size_t images_required_for_decoder = decoder_ptr_->GetNumberPatterns();

    // Do not continue if image array does not contain the expected number of images
    if (!equal_number_images_per_camera || images_per_camera < images_required_for_decoder) {
      ROS_INFO(
          "[DecoderNodelet] Number of images in image array does not match requirements for "
          "decoder. Ignoring "
          "this image array message");
    } else {
      // Convert image msg to cv img
      std::vector<cv_bridge::CvImageConstPtr> cv_img_ptr_vec;
      image_acquisition::ConvertImgArrToCvPtrVec(image_array_ptr, cv_img_ptr_vec);

      // Decode pattern sequence for each camera, store results
      std::vector<cv::Mat> decoder_results(number_output_images_, cv::Mat());

      for (size_t i = 0; i < cameras_to_decode_indices_.size(); i++) {
        std::vector<cv::Mat> frames;

        // For now, we assume that all the images required for decoding are at the front
        for (size_t j = 0; j < images_required_for_decoder; j++) {
          frames.emplace_back(
              cv_img_ptr_vec.at(cameras_to_decode_indices_[i] * images_required_for_decoder + j)
                  ->image);
        }

        decoder_ptr_->SetFrames(frames);
        decoder_ptr_->DecodeFrames(decoder_results.at(i * 4), decoder_results.at(i * 4 + 1),
                                   decoder_results.at(i * 4 + 2), decoder_results.at(i * 4 + 3));
      }

      // Assign last image in the output to be the one used for colouring if required
      if (colour_shading_enabled_) {
        decoder_results.back() =
            cv_img_ptr_vec.at(images_per_camera * colour_camera_index_ + colour_image_index_)
                ->image;
      }

      // Publish results
      image_acquisition::PublishCvMatVec(
          decoded_pub_, decoder_results, image_array_ptr->header.frame_id,
          image_array_ptr->header.stamp, ros::Time::now(), output_image_format_vec_);
    }
  }
};

}  // namespace codec
}  // namespace sl_sensor
