#ifndef TRAFFIC_STATE_H
#define TRAFFIC_STATE_H

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "t3_msgs/BoundingBox.h"
#include "t3_msgs/traffic_light_data.h"
#include "t3_msgs/traffic_light_image.h"

// Include OpenCV
#include "opencv2/opencv.hpp"

namespace sensor
{
constexpr int IMG_SIZE = 352;

struct TrafficLight
{
public:
  t3_msgs::BoundingBox bounding_box;
  int x;
  int y;
  int height;
  int width;
  int square;
  // RED = 0; YELLOW = 1; GREEN = 2;
  int8_t color;

  TrafficLight() : height(0), width(0), square(0), color(0){};
  TrafficLight(const t3_msgs::BoundingBox& bbox)
    : bounding_box(bbox)
    , x((int)bbox.xmin * 640 / 352)
    , y((int)bbox.ymin * 480 / 352)
    , height((int)bbox.ymax * 480 / 352 - (int)bbox.ymin * 480 / 352)
    , width((int)bbox.xmax * 640 / 352 - (int)bbox.xmin * 640 / 352)
    , color(0)
  {
    square = (int)height * width;
  };
};

}  // namespace sensor
#endif
