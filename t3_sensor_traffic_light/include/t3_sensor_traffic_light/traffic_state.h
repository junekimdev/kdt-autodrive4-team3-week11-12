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
const std::string SUB_TOPIC = "traffic_light_image";
const std::string PUB_TOPIC = "traffic_light_data";
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

class Image_process
{
public:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat light_image;
  TrafficLight traffic_light;

  Image_process()
  {
    this->sub = node.subscribe(SUB_TOPIC, 1, &Image_process::callbackTraffic, this);
    this->pub = node.advertise<t3_msgs::traffic_light_data>(PUB_TOPIC, 1);
  };
  void callbackTraffic(const t3_msgs::traffic_light_image::ConstPtr& msg)
  {
    light_image = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, const_cast<uchar*>(&msg->image_data[0]), msg->step);
    traffic_light = TrafficLight(msg);
  };
  void publish()
  {
    if (traffic_light.color != -1)
    {
      t3_msgs::traffic_light_light_data msg;
      msg.header.stamp = ros::Time::now();
      msg.detected = true;
      msg.color = traffic_light.color;
      msg.bounding_box = traffic_light.bounding_box;
      pub.publish(msg);
    }
  };
  void detect_traffic_light()
  {
    cv::Scalar lower_blue(160, 50, 50);
    cv::Scalar upper_blue(180, 255, 255);
    BoundingBox bbox = traffic_light.bounding_box;
    cv::Mat roi = light_image(cv::Range(bbox.xmin, bbox.xmax), cv::Range(bbox.ymin, bbox.ymax_));

    cv::Mat new_img;
    cv::cvtColor(roi, new_img, cv::COLOR_BGR2HSV);

    cv::Mat hsv_img;
    cv::cvtColor(new_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv_img, lower_blue, upper_blue, mask);

    cv::Mat mask_img;
    cv::bitwise_and(new_img, new_img, mask_img, mask);

    cv::Mat gray_img;
    cv::cvtColor(mask_img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Mat th_img;
    cv::threshold(gray_img, th_img, 1, 255, cv::THRESH_BINARY_INV);

    int row = static_cast<int>(traffic_light.height * 416 * 0.3);

    int count1 = 0;
    int count2 = 0;
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < traffic_light.width; j++)
      {
        if (th_img.at<int>(i, j) == 0)
        {
          count1++;
        }
        else if (th_img.at<int>(i + static_cast<int>(0.66 * traffic_light.height * 416), j) == 0)
        {
          count2++;
        }
      }
    }
    int decision1 = static_cast<int>(count1 / traffic_light.square * 1000);
    int decision2 = static_cast<int>(count2 / traffic_light.square * 1000);
    if (decision1 == 0 && decision2 == 0)
    {
      traffic_light.color = 1;
    }
    else if (decision1 == 0 || decision2 == 0)
    {
      traffic_light.color = 2;
    }
    else
    {
      traffic_light.color = 0;
    }

    if (traffic_light.color != -1)
    {
      publish();
    }
  };
};

}  // namespace sensor
#endif
