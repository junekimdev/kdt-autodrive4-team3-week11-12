#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>

#include "t3_msgs/lane_data.h"
#include "t3_msgs/object_data.h"
#include "t3_msgs/stop_line_data.h"
#include "t3_msgs/traffic_light_data.h"

namespace sensor
{

constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

// Define states
struct Cam
{
  int width;
  bool leftDetected;
  bool rightDetected;
  int lpos;
  int rpos;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;

  Cam()
    : width(WIDTH)
    , leftDetected(false)
    , rightDetected(false)
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
    leftDetected = msg->leftDetected;
    rightDetected = msg->rightDetected;
    width = msg->width;
    if (msg->leftDetected)
    {
      lpos = msg->lpos;
      lposMemo.erase(lposMemo.begin());
      lposMemo.emplace_back(lpos);
      lposSMA =
          static_cast<int>(std::accumulate(lposMemo.begin(), lposMemo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
    if (msg->rightDetected)
    {
      rpos = msg->rpos;
      rposMemo.erase(rposMemo.begin());
      rposMemo.emplace_back(rpos);
      rposSMA =
          static_cast<int>(std::accumulate(rposMemo.begin(), rposMemo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
  }
};

struct BoundingBox
{
  int id;
  int xmin;
  int ymin;
  int xmax;
  int ymax;
  float probability;

  BoundingBox()
    : id(-1)
   , xmin(0)
   , ymin(0)
   , xmax(0)
   , ymax(0)
   ,probability(0)
   {};
};

struct Object
{
  BoundingBox boundingBox;
  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
    boundingBox.id = msg->id;
    boundingBox.xmin = msg->xmin;
    boundingBox.ymin = msg->ymin;
    boundingBox.xmax = msg->xmax;
    boundingBox.ymax = msg->ymax;
    boundingBox.probability = msg->probability;
  };
};

struct StopLine
{

  bool detected;

  StopLine()
    : detected(false)
  {};

  void reduce(const t3_msgs::stop_line_data::ConstPtr& msg)
  {
    detected = msg->detected;
  };
  }
};

struct TrafficLight
{

  bool detected;
  uint8_t color;
  BoundingBox boundingBox;
  TrafficLight()
    : color(-1),bounding(BoundingBox()),detected(false){};


  void reduce(const t3_msgs::traffic_light_data::ConstPtr& msg)
  {
    color = msg->color;
    boundingBox.id = 5;
    boundingBox.xmax = msg->bounding.xmax;
    boundingBox.xmin = msg->bounding.xmin;
    boundingBox.ymax = msg->bounding.ymax;
    boundingBox.ymin = msg->bounding.ymin;
    boundingBox.probability = msg->probability;
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
