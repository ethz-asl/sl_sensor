#include "sl_sensor_timer/timer.hpp"

namespace sl_sensor {
namespace timer {
Timer::Timer(std::string name) : _name(name){};

void Timer::Start() {
  _start_time = std::chrono::high_resolution_clock::now();
  _timing_in_progess = true;
};

void Timer::End() {
  if (_timing_in_progess) {
    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - _start_time;
    _counter++;
    _total_time += elapsed.count();
    _timing_in_progess = false;
  }
}

double Timer::GetAverageTime() { return _total_time / (double)_counter; }

void Timer::PrintAverageTiming() {
  if (_counter > 0) {
    std::cout << "Timer " << _name << " Average Time [ms]: " << 1000 * this->GetAverageTime()
              << " over " << _counter << " samples" << std::endl;
  }
}

Timer::~Timer() { this->PrintAverageTiming(); }

}  // namespace timer

}  // namespace sl_sensor
