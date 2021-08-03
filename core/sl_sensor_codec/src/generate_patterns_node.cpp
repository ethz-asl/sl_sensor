

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sl_sensor_calibration/projector_parameters.hpp>
#include <sl_sensor_projector/projector_utils.hpp>

#include "sl_sensor_codec/codec_factory.hpp"
#include "sl_sensor_codec/encoder.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <experimental/filesystem>

using namespace sl_sensor::codec;
using namespace sl_sensor::calibration;
using namespace sl_sensor::projector;

cv::Mat AccessMatInVec(const std::vector<cv::Mat>& mat_vec, size_t index);
bool DirectoryExists(const char* path);
void SavePatterns(const Encoder& encoder, const cv::Mat& map1, const cv::Mat& map2, unsigned int screen_cols,
                  unsigned int screen_rows, bool is_diamond_pixel, const std::string& encoder_name,
                  const std::string& label, const std::string& current_folder_directory);

int main(int argc, char** argv)
{
  // Init ros node
  ros::init(argc, argv, "calibrator");
  ros::NodeHandle nh_public;
  ros::NodeHandle private_nh("~");

  std::string projector_yaml_directory;
  std::string save_folder;
  std::string projector_parameters_file;
  std::vector<std::string> encoder_names = CodecFactory::GetAllCodecNames();

  private_nh.param<std::string>("projector_yaml_directory", projector_yaml_directory, projector_yaml_directory);
  private_nh.param<std::string>("save_folder", save_folder, save_folder);
  private_nh.param<std::string>("projector_parameters_file", projector_parameters_file, projector_parameters_file);

  // Load Projector params
  unsigned int screen_cols, screen_rows;
  GetProjectorResolution(projector_yaml_directory, screen_rows, screen_cols);
  bool is_diamond_pixel = GetIsDiamondPixel(projector_yaml_directory);
  ProjectorParameters projector_parameters;

  if (projector_parameters.Load(projector_parameters_file))
  {
    ROS_INFO("[GeneratePatternsNode] Projector parameters successfully loaded");
    std::cout << "Projector parameters: " << std::endl;
    std::cout << projector_parameters << std::endl;
  }
  else
  {
    ROS_ERROR("[GeneratePatternsNode] Failed to load projector parameters");
    return 0;
  }

  // For each codec, we create a folder and save the patterns there
  for (const auto& encoder_name : encoder_names)
  {
    std::string current_folder_directory = save_folder + encoder_name;

    bool folder_exists = DirectoryExists(current_folder_directory.c_str());

    if (!folder_exists)
    {
      folder_exists = std::experimental::filesystem::create_directory(current_folder_directory);
    }

    if (!folder_exists)
    {
      std::string info = "[GeneratePatternsNode] Could not create folder at " + current_folder_directory +
                         " . Skipping this encoding pattern";
      ROS_WARN("%s", info.c_str());
      continue;
    }

    // We generate two encoders -> One for horizontal and one for vertical. This is to ensure that when we compress the
    // images into a BGR bit map, we can separate them cleanly (i.e. one BGR image should only have pattern sequences
    // for one direction). The rationale is that if you want to perform hardware triggering, the less BGR images you
    // have to load, the better.

    private_nh.setParam("direction", "horizontal");
    auto horz_encoder_ptr = CodecFactory::GetInstanceEncoder(encoder_name, private_nh);
    private_nh.setParam("direction", "vertical");
    auto vert_encoder_ptr = CodecFactory::GetInstanceEncoder(encoder_name, private_nh);

    if (!horz_encoder_ptr || !vert_encoder_ptr)
    {
      std::string info = "[GeneratePatternsNode] Failed to load patterns from encoders with name " + encoder_name;
      ROS_INFO("%s", info.c_str());
      continue;
    }

    // Initialise distortion maps
    cv::Mat map1, map2;
    cv::Size map_size = cv::Size(screen_cols, screen_rows);
    Encoder::InitDistortMap(projector_parameters.intrinsic_mat(), projector_parameters.lens_distortion(), map_size,
                            map1, map2);

    // Generate patterns
    SavePatterns(*horz_encoder_ptr, map1, map2, screen_cols, screen_rows, is_diamond_pixel, encoder_name,
                 std::string{ "horizontal" }, current_folder_directory);
    SavePatterns(*vert_encoder_ptr, map1, map2, screen_cols, screen_rows, is_diamond_pixel, encoder_name,
                 std::string{ "vertical" }, current_folder_directory);
  }

  ROS_INFO("Patterns Generated Successfully!");

  return 0;
};

cv::Mat AccessMatInVec(const std::vector<cv::Mat>& mat_vec, size_t index)
{
  // If Index is out of range, we return a dark image
  if (index >= mat_vec.size())
  {
    cv::Mat temp = mat_vec[0].clone();
    temp.setTo(cv::Scalar(0));
    return temp;
  }
  else
  {
    // If not we return a cloned copy of the matrix
    return mat_vec[index].clone();
  }
}

bool DirectoryExists(const char* path)
{
  struct stat info;

  if (stat(path, &info) != 0)
  {
    return false;
  }
  else if (info.st_mode & S_IFDIR)
  {
    return true;
  }
  else
  {
    return false;
  }
};

void SavePatterns(const Encoder& encoder, const cv::Mat& map1, const cv::Mat& map2, unsigned int screen_cols,
                  unsigned int screen_rows, bool is_diamond_pixel, const std::string& encoder_name,
                  const std::string& label, const std::string& current_folder_directory)
{
  std::vector<cv::Mat> patterns = encoder.GetEncodingPatterns();
  std::vector<cv::Mat> processed_patterns;

  // For each pattern, apply lens distortion, diamond downsample if necessary and then save. These are just for
  // checking that the generated patterns are correct
  for (unsigned int i = 0; i < patterns.size(); i++)
  {
    cv::Mat current_pattern = patterns[i].clone();

    // general repmat
    current_pattern =
        cv::repeat(current_pattern, screen_rows / current_pattern.rows + 1, screen_cols / current_pattern.cols + 1);
    current_pattern = current_pattern(cv::Range(0, screen_rows), cv::Range(0, screen_cols));

    // correct for lens distortion
    cv::remap(current_pattern, current_pattern, map1, map2, CV_INTER_CUBIC);

    if (is_diamond_pixel)
    {
      current_pattern = Encoder::DiamondDownsample(current_pattern);
    }

    // Save the processed pattern (only one channel is required)
    cv::Mat bgr[3];
    cv::split(current_pattern, bgr);
    processed_patterns.push_back(bgr[0]);

    std::string format_str = current_folder_directory + "/" + label + "_%d.bmp";
    cv::imwrite(cv::format(format_str.c_str(), i), current_pattern);
  }

  // We also save the patterns in groups of 3 in BGR bitmap images which will be uploaded onto projector
  size_t number_compressed_images =
      processed_patterns.size() / 3 + (processed_patterns.size() % 3 != 0);  // Integer division and round up

  for (size_t i = 0; i < number_compressed_images; i++)
  {
    cv::Mat bgr_image;

    // bitmat format is BGR, projector image index mapping is 0->G 1->R 2->B
    std::vector<cv::Mat> channels = { AccessMatInVec(processed_patterns, i * 3 + 2),
                                      AccessMatInVec(processed_patterns, i * 3),
                                      AccessMatInVec(processed_patterns, i * 3 + 1) };

    cv::merge(channels, bgr_image);

    std::string format_str = current_folder_directory + "/" + encoder_name + "_" + label + "_%d.bmp";
    cv::imwrite(cv::format(format_str.c_str(), i + 1), bgr_image);
  }

  std::string info = "[GeneratePatternsNode] Patterns from encoder " + encoder_name + " (" + label + ") " +
                     " generated successfully to folder " + current_folder_directory + ".";
  ROS_INFO("%s", info.c_str());
};