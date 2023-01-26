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

const std::string SUB_TOPIC = "traffic_light_image";
const std::string PUB_TOPIC = "traffic_light_data";

struct BoundingBox
{
  float probability_;
  int xmin_;
  int ymin_;
  int xmax_;
  int ymax_;

  BoundingBox(float probability, int xmin, int ymin, int xmax, int ymax)
    : probability_(0), xmin_(0), ymin_(0), xmax_(0), ymax_(0){};
}

struct Traffic
{
public:
  BoundingBox boundingBox_;
  int height_;
  int weight_;
  int square_;
  cv::Mat image_;
  // RED = 0; YELLOW = 1; GREEN = 2;
  int8_t color_ = -1;

    :height_(0),weight_(0),square_(0),image_(cv::Mat(352,352,CV_8UC3)){};

    void update(const t3_msgs::traffic_light_image::ConstPtr& msg)
    {
      image_ = cv::Mat(352, 352, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
      boundingBox_ = BoundingBox(msg->bounding_box.probability, msg->xmin, msg->ymin, msg->xmax, msg->ymax);

      height_ = boundingBox_.ymax_ - boundingBox_.ymin_;
      weight_ = boundingBox_.xmax_ - boundingBox_.xmin_;
      square_ = height_ * weight_;
    };
};

class Image_process
{
  public:
    ros::NodeHandle node;
    ros::Subscriber sub;
    ros::Subscriber sub_cam;
    ros::Publisher pub;
    Traffic traffic;

    Image_process()
    {
      this->sub = node.subscribe(SUB_TOPIC, 1, &Image_process::callbackTraffic, this);
      this->pub = node.advertise<t3_msgs::traffic_light_data>(PUB_TOPIC, 1);
    };
    void callbackTraffic(const t3_msgs::traffic_light_image::ConstPtr& msg)
    {
      traffic->update(msg);
    };
    void publish()
    {
      if (traffic.color_ != -1)
      {
        t3_msgs::traffic_light_data msg;
        msg.header.stamp = ros::Time::now();
        msg.detected = true;
        msg.color = traffic.color;
        msg.bounding_box = traffic.bounding_box;
        pub.publish(msg);
      }
    };
    void detect_traffic_light()
    {
      cv::Scalar lower_blue(160, 50, 50);
      cv::Scalar upper_blue(180, 255, 255);
      cv::Mat roi = traffic.image_(cv::Range(traffic.xmin_, traffic.xmax_), cv::Range(traffic.ymin_, traffic.ymax_));

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

      int row = static_cast<int>(traffic.height_ * 416 * 0.3);

      int count1 = 0;
      int count2 = 0;
      for (int i = 0; i < row; i++)
      {
        for (int j = 0; j < traffic.width_; j++)
        {
          if (th_img.at<int>(i, j) == 0)
          {
            count1++;
          }
          else if (th_img.at<int>(i + static_cast<int>(0.66 * traffic.height_ * 416), j) == 0)
          {
            count2++;
          }
        }
      }
      int decision1 = static_cast<int>(count1 / traffic.square_ * 1000);
      int decision2 = static_cast<int>(count2 / traffic.square_ * 1000);
      if (decision1 == 0 && decision2 == 0)
      {
        traffic.color_ = 1;
      }
      else if (decision1 == 0 || decision2 == 0)
      {
        traffic.color_ = 2;
      }
      else
      {
        traffic.color_ = 0;
      }

      if (traffic.color_ != -1)
      {
        publish();
      }
    };
};

#endif
