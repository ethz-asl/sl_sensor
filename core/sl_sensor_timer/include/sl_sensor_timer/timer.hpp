#pragma once
#include <chrono>
#include <iostream>
#include <string>

namespace sl_sensor
{
namespace timer
{
/**
 * @brief Timer class object to measure average time taken a specific process
 *
 */
class Timer
{
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
  std::string _name = "";
  bool _timing_in_progess = false;
  int _counter = 0;
  double _total_time = 0.0f;
  std::chrono::time_point<std::chrono::high_resolution_clock> _start_time;
};

}  // namespace timer

}  // namespace sl_sensor
