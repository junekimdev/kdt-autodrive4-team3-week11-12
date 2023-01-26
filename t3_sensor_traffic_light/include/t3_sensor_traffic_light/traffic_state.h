#ifndef TRAFFIC_STATE_H
#define TRAFFIC_STATE_H
#include "ros/console.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "t3_msgs/BoundingBox.h"
#include "t3_msgs/traffic_light_data.h"
#include "t3_msgs/traffic_light_image.h"
#include <opencv2/opencv.hpp>
#include <vector>

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

  TrafficLight() : height(0), width(0), square(0), color(-1){};
  TrafficLight(const t3_msgs::BoundingBox& bbox)
    : bounding_box(bbox), height(bbox.ymax - bbox.ymin), width(bbox.xmax - bbox.xmin), color(-1)
  {
    square = height * width;
  };
};

}  // namespace sensor
#endif
