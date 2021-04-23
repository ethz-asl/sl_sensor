#pragma once
#include <stdio.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace io
{
/**
 * @brief Writes a csv file were each row is a series of timestamps that correspond to a single pattern sequence
 *
 */
class FrameTimingsCsvWriter
{
public:
  /**
   * @brief Construct a new Frame Timings Csv Writer object
   *
   * @param filename - Name of csv file to be written
   */
  FrameTimingsCsvWriter(const std::string& filename);

  /**
   * @brief Write next row in csv file
   *
   * @param timestamp_vec - vector of timestamps to be writted
   */
  void WriteNextRow(const std::vector<uint64_t>& timestamp_vec);

private:
  char delimiter_ = ' ';
  std::fstream stream_;
};

}  // namespace io
}  // namespace sl_sensor
