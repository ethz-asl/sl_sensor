#include "sl_sensor_io/tum_csv_writer.hpp"

namespace sl_sensor
{
namespace io
{
TumCsvWriter::TumCsvWriter(const std::string& filename)
{
  // Create an output filestream, create file if it does not exist, clear file if it exists
  stream_.open(filename, std::ofstream::out | std::ofstream::trunc);
}

void TumCsvWriter::WriteNextRow(const TumPose& pose)
{
  stream_ << std::to_string(pose.timestamp) << delimiter_;
  stream_ << std::to_string(pose.x) << delimiter_;
  stream_ << std::to_string(pose.y) << delimiter_;
  stream_ << std::to_string(pose.z) << delimiter_;
  stream_ << std::to_string(pose.qx) << delimiter_;
  stream_ << std::to_string(pose.qy) << delimiter_;
  stream_ << std::to_string(pose.qz) << delimiter_;
  stream_ << std::to_string(pose.qw) << '\n';
}

}  // namespace io
}  // namespace sl_sensor