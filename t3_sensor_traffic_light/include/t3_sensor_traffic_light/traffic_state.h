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

struct BoundingBox
{
  int id;
  float probability;
  int xmin;
  int ymin;
  int xmax;
  int ymax;

  BoundingBox() : id(5), probability(0.f), xmin(0), ymin(0), xmax(0), ymax(0){};
  BoundingBox(const t3_msgs::BoundingBox& bbox)
    : id(bbox.id), probability(bbox.probability), xmin(bbox.xmin), ymin(bbox.ymin), xmax(bbox.xmax), ymax(bbox.ymax){};
}

struct TrafficLight
{
public:
  BoundingBox bounding_box;
  int height;
  int width;
  int square;
  // RED = 0; YELLOW = 1; GREEN = 2;
  int8_t color;

  TrafficLight() : height(0), width(0), square(0), color(0){};
  TrafficLight(const t3_msgs::BoundingBox& bbox)
    : bounding_box(bbox), height(bbox.ymax - bbox.ymin), width(bbox.xmax - bbox.xmin), color(0)
  {
    square = height * width;
  };
};

}  // namespace sensor
#endif
