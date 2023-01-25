#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <numeric>

#include "t3_msgs/lane_data.h"
#include "t3_msgs/object_data.h"
#include "t3_msgs/stop_line_data.h"
#include "t3_msgs/traffic_light_data.h"

namespace sensor_state
{

constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

// Define states
struct Cam
{
  int width;
  bool left_detected;
  bool right_detected;
  int lpos;
  int rpos;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;

  Cam()
    : width(WIDTH)
    , left_detected(false)
    , right_detected(false)
    , lpos(0)
    , rpos(WIDTH - 1)
    , lposMemo(std::vector<int>(SMA_NUM, 0))
    , rposMemo(std::vector<int>(SMA_NUM, WIDTH - 1))
    , lposSMA(0)
    , rposSMA(WIDTH - 1)
  {
  }
  void reduce(const t3_msgs::lane_data::ConstPtr& msg)
  {
    left_detected = msg->left_detected;
    right_detected = msg->right_detected;
    width = msg->width;
    if (msg->left_detected)
    {
      lpos = msg->lpos;
      lposMemo.erase(lposMemo.begin());
      lposMemo.emplace_back(lpos);
      lposSMA =
          static_cast<int>(std::accumulate(lposMemo.begin(), lposMemo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
    if (msg->right_detected)
    {
      rpos = msg->rpos;
      rposMemo.erase(rposMemo.begin());
      rposMemo.emplace_back(rpos);
      rposSMA =
          static_cast<int>(std::accumulate(rposMemo.begin(), rposMemo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
  }
};

struct Object
{
  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
  }
};

struct StopLine
{
  void reduce(const t3_msgs::stop_line_data::ConstPtr& msg)
  {
  }
};

struct TrafficLight
{
  void reduce(const t3_msgs::traffic_light_data::ConstPtr& msg)
  {
  }
};

// Combine states
struct State
{
  Cam cam;
  Object object;
  StopLine stop_line;
  TrafficLight traffic_light;
};

}  // namespace sensor

#endif
