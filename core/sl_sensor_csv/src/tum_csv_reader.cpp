#include "sl_sensor_csv/tum_csv_reader.hpp"

namespace sl_sensor
{
namespace csv
{
TumCsvReader::TumCsvReader(const std::string& filename)
{
  // Create an input filestream

  stream_ptr_ = std::make_unique<std::ifstream>(filename);

  // Make sure the file is open
  if (!stream_ptr_->is_open())
  {
    throw std::runtime_error("Could not open file " + filename);
  }
}

bool TumCsvReader::GetNextRow(TumPose& pose)
{
  std::string line = "";
  std::string word = "";

  // read an entire row and
  // store it in a string variable 'line'
  // Automatically takes care of line break '\n'
  std::getline(*stream_ptr_, line);

  // used for breaking words
  std::stringstream stream(line);

  // read every column data of a row and
  // store it in a string variable, 'word'
  std::vector<std::string> row_entries = {};
  while (std::getline(stream, word, delimiter_))
  {
    row_entries.push_back(word);
  }

  // Store data in pose
  if (row_entries.size() == 8)
  {
    pose.timestamp = std::stoull(row_entries.at(0));
    pose.x = std::stod(row_entries.at(1));
    pose.y = std::stod(row_entries.at(2));
    pose.z = std::stod(row_entries.at(3));
    pose.qx = std::stod(row_entries.at(4));
    pose.qy = std::stod(row_entries.at(5));
    pose.qz = std::stod(row_entries.at(6));
    pose.qw = std::stod(row_entries.at(7));

    return true;
  }
  else
  {
    return false;
  }
}

}  // namespace csv
}  // namespace sl_sensor