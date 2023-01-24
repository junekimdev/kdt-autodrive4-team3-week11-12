#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <string>
#include <vector>

namespace control
{

constexpr int16_t ANGLE_CENTER = 0;
constexpr int16_t ANGLE_MIN = -50;
constexpr int16_t ANGLE_MAX = 50;

enum struct Mode
{
  Stop,
  Go,
  TurnLeft,
  TurnRight
};

struct State
{
  Mode mode;
  int16_t speed;
  int16_t angle;
  bool started;

  State() : mode(Mode::Stop), speed(0), angle(ANGLE_CENTER), started(false)
  {
  }

  void reduce(){};
};

}  // namespace control

#endif
