// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "t3_msgs/traffic_light_data.h"
#include "t3_msgs/traffic_light_image.h"

// Include header
#include "t3_sensor_traffic_light/traffic_state.h"

namespace sensor
{
const std::string NAME = "traffic_light";
const std::string SUB_TOPIC = "traffic_light_image";
const std::string PUB_TOPIC = "traffic_light_data";
const cv::Scalar LOWER_BLUE = cv::Scalar(160, 50, 50);
const cv::Scalar UPPER_BLUE = cv::Scalar(180, 255, 255);

class Processor
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat light_image;
  TrafficLight traffic_light;

public:
  Processor()
  {
    this->sub = node.subscribe(SUB_TOPIC, 1, &Processor::callback, this);
    this->pub = node.advertise<t3_msgs::traffic_light_data>(PUB_TOPIC, 1);
  };

  void callback(const t3_msgs::traffic_light_image::ConstPtr& msg);
  void publish();
  void process();
};
void Processor::callback(const t3_msgs::traffic_light_image::ConstPtr& msg)
{
  light_image = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, const_cast<uchar*>(&msg->image_data[0]), msg->step);
  traffic_light = TrafficLight(msg->bounding_box);
};

void Processor::publish()
{
  t3_msgs::traffic_light_light_data msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;
  msg.detected = traffic_light.color != -1;
  msg.color = traffic_light.color;
  msg.bounding_box = traffic_light.bounding_box;
  pub.publish(msg);
};

void Processor::process()
{
  const auto& bbox = traffic_light.bounding_box;
  cv::Mat roi = light_image(cv::Range(bbox.xmin, bbox.xmax), cv::Range(bbox.ymin, bbox.ymax_));

  cv::Mat new_img;
  cv::cvtColor(roi, new_img, cv::COLOR_BGR2HSV);

  cv::Mat hsv_img;
  cv::cvtColor(new_img, hsv_img, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv_img, LOWER_BLUE, UPPER_BLUE, mask);

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

  publish();
};

}  // namespace sensor

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, sensor::NAME);
  sensor::Processor processor;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  while (ros::ok())
  {
    ros::spinOnce();
    processor.process();
  }

  return 0;
}
