#include "sl_sensor_io/frame_timings_csv_writer.hpp"

namespace sl_sensor {
namespace io {
FrameTimingsCsvWriter::FrameTimingsCsvWriter(const std::string& filename) {
  // Create an output filestream, create file if it does not exist, clear file if it exists
  stream_.open(filename, std::ofstream::out | std::ofstream::trunc);
}

void FrameTimingsCsvWriter::WriteNextRow(const std::vector<uint64_t>& timestamp_vec) {
  int N = timestamp_vec.size();

  for (int i = 0; i < N; i++) {
    stream_ << std::to_string(timestamp_vec[i]) << ((i != (N - 1)) ? delimiter_ : '\n');
  }
}
}  // namespace io
}  // namespace sl_sensor
