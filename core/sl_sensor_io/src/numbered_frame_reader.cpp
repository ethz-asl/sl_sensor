#include "sl_sensor_io/numbered_frame_reader.hpp"

namespace sl_sensor
{
namespace io
{
NumberedFrameReader::NumberedFrameReader(const std::string &folder_directory, const std::string &file_title,
                                         const std::string &file_format, int frame_no_start, int frame_no_end,
                                         int frame_increment, int pat_no_min, int pat_no_max)
  : folder_directory_(folder_directory)
  , file_title_(file_title)
  , file_format_(file_format)
  , frame_no_start_(frame_no_start)
  , frame_no_end_(frame_no_end)
  , frame_increment_(frame_increment)
  , pat_no_min_(pat_no_min)
  , pat_no_max_(pat_no_max){};

bool NumberedFrameReader::GetNextImageSequence(std::vector<cv::Mat> &image_sequence)
{
  int frame_to_read = (last_read_frame_no_ < 0) ? frame_no_start_ : last_read_frame_no_ + frame_increment_;

  if (frame_to_read > frame_no_end_)
  {
    return false;
  };

  ReadImageSequence(folder_directory_, file_title_, file_format_, frame_to_read, pat_no_min_, pat_no_max_,
                    image_sequence);

  last_read_frame_no_ = frame_to_read;

  return true;
};

void NumberedFrameReader::reset()
{
  last_read_frame_no_ = -1;
};

void NumberedFrameReader::ReadImageSequence(const std::string &folder_directory, const std::string &file_title,
                                            const std::string &file_format, int frame_no, int pat_no_min,
                                            int pat_no_max, std::vector<cv::Mat> &image_sequence)

{
  image_sequence.clear();

  char frame_no_char[50];
  sprintf(frame_no_char, "%02d", frame_no);
  std::string frame_no_str(frame_no_char);

  for (int i = pat_no_min; i <= pat_no_max; i++)
  {
    char pat_no_char[50];
    sprintf(pat_no_char, "%02d", i);
    std::string pat_no_str(pat_no_char);

    std::string full_filename = folder_directory + file_title + "_" + frame_no_str + "_" + pat_no_str + file_format;
    cv::Mat img = cv::imread(full_filename, cv::IMREAD_GRAYSCALE);
    img.convertTo(img, CV_32F, 1.0f / 255.0f);
    image_sequence.push_back(img);
  }
};

}  // namespace io
}  // namespace sl_sensor