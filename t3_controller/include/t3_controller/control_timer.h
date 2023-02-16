#ifndef CONTROL_TIMER_H
#define CONTROL_TIMER_H

#include <chrono>

namespace control
{
class Timer
{
  using ms = std::chrono::milliseconds;
  using us = std::chrono::microseconds;
  using clock = std::chrono::system_clock;

  clock::time_point start;

public:
  Timer() : start(clock::now())
  {
  }

  void reset()
  {
    start = clock::now();
  }

  long long stamp_ms()
  {
    return std::chrono::duration_cast<ms>(clock::now() - start).count();
  }

  long long stamp_us()
  {
    return std::chrono::duration_cast<us>(clock::now() - start).count();
  }
};
}  // namespace control

#endif
