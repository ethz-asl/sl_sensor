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

#include "sl_sensor_timer/timer.hpp"

namespace sl_sensor {
namespace timer {
Timer::Timer(std::string name) : name_(name){};

void Timer::Start() {
  start_time_ = std::chrono::high_resolution_clock::now();
  timing_in_progess_ = true;
};

void Timer::End() {
  if (timing_in_progess_) {
    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time_;
    counter_++;
    total_time_ += elapsed.count();
    timing_in_progess_ = false;
  }
}

double Timer::GetAverageTime() { return total_time_ / (double)counter_; }

void Timer::PrintAverageTiming() {
  if (counter_ > 0) {
    std::cout << "Timer " << name_ << " Average Time [ms]: " << 1000 * this->GetAverageTime()
              << " over " << counter_ << " samples" << std::endl;
  }
}

Timer::~Timer() { this->PrintAverageTiming(); }

}  // namespace timer
}  // namespace sl_sensor
