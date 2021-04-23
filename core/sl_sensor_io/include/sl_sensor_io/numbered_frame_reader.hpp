#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace sl_sensor
{
namespace io
{
/**
 * @brief Class object that reads out a sequence of images based on filename
 * Images should be labelled in the following manner: folder_directory + file_title + "_" + frame_no_str + "_" +
 * pat_no_str + file_format frame_no_str and pat_no_str are integers zero padded to be at least 2 digits
 */
class NumberedFrameReader
{
public:
  NumberedFrameReader(){};

  /**
   * @brief Construct a new Numbered Frame Reader object
   *
   * @param folder_directory - Directory where images are located
   * @param file_title - See class description
   * @param file_format - See class description
   * @param frame_no_start - First frame to read
   * @param frame_no_end - Last frame to read
   * @param frame_increment - How much to increment frame_no by after every read
   * @param pat_no_min - Lowest pattern indice
   * @param pat_no_max - Largest pattern indice
   */
  NumberedFrameReader(const std::string &folder_directory, const std::string &file_title,
                      const std::string &file_format, int frame_no_start, int frame_no_end, int frame_increment,
                      int pat_no_min, int pat_no_max);

  /**
   * @brief Get the Next Image Sequence
   *
   * @param image_sequence - vector where images will be written, if successful
   * @return true - Retrieval successful
   * @return false - Retrieval unsuccessful
   */
  bool GetNextImageSequence(std::vector<cv::Mat> &image_sequence);

  /**
   * @brief Reset reader such that next image sequence willbe for frame_no_start
   *
   */
  void reset();

  std::string folder_directory_;
  std::string file_title_;
  std::string file_format_;
  int frame_no_start_;
  int frame_no_end_;
  int pat_no_min_;
  int pat_no_max_;
  int frame_increment_ = 1;

private:
  int last_read_frame_no_ = -1;

  /**
   * @brief Read an image requence based on the provided information
   *
   * @param folder_directory
   * @param file_title
   * @param file_format
   * @param frame_no
   * @param pat_no_min
   * @param pat_no_max
   * @param image_sequence - vector where images will be written, if successful
   */
  void ReadImageSequence(const std::string &folder_directory, const std::string &file_title,
                         const std::string &file_format, int frame_no, int pat_no_min, int pat_no_max,
                         std::vector<cv::Mat> &image_sequence);
};

}  // namespace io
}  // namespace sl_sensor