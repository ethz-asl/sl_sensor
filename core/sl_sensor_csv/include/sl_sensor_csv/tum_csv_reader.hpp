#pragma once
#include <stdio.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "sl_sensor_csv/tum_pose.hpp"

namespace sl_sensor
{
namespace csv
{
/**
 * @brief Class objects that reads a sequence of poses in TUM format
 *
 */
class TumCsvReader
{
public:
  /**
   * @brief Construct a new Tum Csv Reader object
   *
   * @param filename - Name of csv file to be read
   */
  TumCsvReader(const std::string& filename);

  /**
   * @brief Read the next row in the csv file
   *
   * @param pose - pose where output will be writtin, if successful
   * @return true - row successfully read
   * @return false - row not successfully read
   */
  bool GetNextRow(TumPose& pose);

private:
  char delimiter_ = ' ';
  std::unique_ptr<std::ifstream> stream_ptr_;
};

}  // namespace csv
}  // namespace sl_sensor