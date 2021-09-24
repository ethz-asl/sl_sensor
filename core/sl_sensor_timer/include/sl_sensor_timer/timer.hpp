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

#ifndef SL_SENSOR_TIMER_TIMER_HPP_
#define SL_SENSOR_TIMER_TIMER_HPP_

#include <chrono>
#include <iostream>
#include <string>

namespace sl_sensor {
namespace timer {
/**
 * @brief Timer class object to measure average time taken a specific process
 *
 */
class Timer {
 public:
  /**
   * @brief Construct a new Timer object
   *
   * @param name
   */
  Timer(std::string name = "");

  /**
   * @brief Start timer session
   *
   */
  void Start();

  /**
   * @brief End timer session
   *
   */
  void End();

  /**
   * @brief Get the Average Time from all past timer sessions
   *
   * @return double - Average time taken
   */
  double GetAverageTime();

  /**
   * @brief Print average time form all past timer sessions
   *
   */
  void PrintAverageTiming();

  /**
   * @brief Destroy the Timer object, will print the final average timing
   *
   */
  ~Timer();

 private:
  std::string name_ = "";
  bool timing_in_progess_ = false;
  int counter_ = 0;
  double total_time_ = 0.0f;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

}  // namespace timer
}  // namespace sl_sensor

#endif  // SL_SENSOR_TIMER_TIMER_HPP_
