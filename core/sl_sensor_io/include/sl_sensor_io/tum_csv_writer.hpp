#pragma once
#include <stdio.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "sl_sensor_io/tum_pose.hpp"

namespace sl_sensor
{
namespace io
{
/**
 * @brief Class objects that writes a sequence of poses in TUM format
 *
 */
class TumCsvWriter
{
public:
  /**
   * @brief Construct a new Tum Csv Writer object
   *
   * @param filename - Name of output TUM csv file
   */
  TumCsvWriter(const std::string& filename);

  /**
   * @brief Write a single row in TUM csv file
   *
   * @param pose - pose to write
   */
  void WriteNextRow(const TumPose& pose);

private:
  char delimiter_ = ' ';

  std::fstream stream_;
};

}  // namespace io
}  // namespace sl_sensor