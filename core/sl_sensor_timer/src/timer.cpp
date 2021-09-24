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
