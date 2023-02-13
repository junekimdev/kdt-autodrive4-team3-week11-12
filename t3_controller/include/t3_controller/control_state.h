#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <string>
#include <vector>
#include <memory>

// #include "t3_controller/sensor_state.h"
// #include "t3_controller/control_timer.h"
#include "sensor_state.h"
#include "control_timer.h"

namespace control
{

constexpr int16_t ANGLE_CENTER = 5;
constexpr int16_t ANGLE_MORE = 5;
constexpr int16_t ANGLE_MIN = -50;
constexpr int16_t ANGLE_MAX = 50;
constexpr int16_t DRIVE_SPEED = 10;
constexpr int64_t TIMER_IGNORE_MS = 10000LL;

class Mode
{
  bool isStopGroupDetected(const sensor::State& sensor_state);
  void setAngle(const sensor::State& sensor_state, int16_t center_pos);
  void setAngleToFollowLeft(const sensor::State& sensor_state);
  void setAngleToFollowRight(const sensor::State& sensor_state);
  Mode transitFromInit(const sensor::State& sensor_state);
  Mode transitFromReady(const sensor::State& sensor_state);
  Mode transitFromLeft(const sensor::State& sensor_state);
  Mode transitFromLeftPrepToStop(const sensor::State& sensor_state);
  Mode transitFromLeftStop(const sensor::State& sensor_state);
  Mode transitFromRight(const sensor::State& sensor_state);
  Mode transitFromRightPrepToStop(const sensor::State& sensor_state);
  Mode transitFromRightStop(const sensor::State& sensor_state);

public:
  enum struct Id
  {
    Init,
    Ready,
    Left,
    LeftPrepToStop,
    LeftStop,
    Right,
    RightPrepToStop,
    RightStop,
  };
  Id id;
  int16_t speed;
  int16_t angle;
  Timer timer;

  Mode() : id(Id::Init), speed(0), angle(0)
  {
  }
  Mode(Id id, int16_t speed, int16_t angle) : id(id), speed(speed), angle(angle)
  {
  }
  Mode(Id id, Mode* mode) : id(id), speed(mode->speed), angle(mode->angle)
  {
  }

  Mode next(const sensor::State& sensor_state);
};

struct State
{
  Mode mode;

  State()
  {
  }

  void reduce(const sensor::State& sensor_state)
  {
    mode = mode.next(sensor_state);
  }
};

}  // namespace control

#endif
