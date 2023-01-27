#include <algorithm>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "t3_msgs/traffic_light_data.h"
#include "t3_msgs/traffic_light_image.h"

// Include header
#include "t3_sensor_traffic_light/traffic_state.h"

namespace sensor
{
const std::string NAME = "traffic_light";
const std::string SUB_TOPIC_CAM = "usb_cam/image_raw";
const std::string SUB_TOPIC_TRAFFIC = "traffic_light_image";
const std::string PUB_TOPIC = "traffic_light_data";
const cv::Scalar LOWER_BLUE = cv::Scalar(160, 50, 50);
const cv::Scalar UPPER_BLUE = cv::Scalar(180, 255, 255);
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

class Processor
{
  ros::NodeHandle node;
  ros::Subscriber sub_cam;
  ros::Subscriber sub_traffic;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  TrafficLight traffic_light;

public:
  bool enable_debug;

  Processor()
  {
    node.param<bool>("sensor_traffic_light_enable_debug", enable_debug, true);
    sub_cam = node.subscribe(SUB_TOPIC_CAM, 1, &Processor::callbackCam, this);
    sub_traffic = node.subscribe(SUB_TOPIC_TRAFFIC, 1, &Processor::callbackTraffic, this);
    pub = node.advertise<t3_msgs::traffic_light_data>(PUB_TOPIC, 1);
  };

  void callbackCam(const sensor_msgs::ImageConstPtr& msg);
  void callbackTraffic(const t3_msgs::traffic_light_image::ConstPtr& msg);
  void publish();
  void process();
};

void Processor::callbackCam(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
};

void Processor::callbackTraffic(const t3_msgs::traffic_light_image::ConstPtr& msg)
{
  try
  {
    traffic_light = TrafficLight(msg->bounding_box);
    if (!vFrame.empty())
      process();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
};

void Processor::publish()
{
  t3_msgs::traffic_light_data msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;
  msg.detected = traffic_light.color != -1;
  msg.color = traffic_light.color;
  msg.bounding_box = traffic_light.bounding_box;
  pub.publish(msg);
};

void Processor::process()
{
  cv::Mat frame = vFrame.clone();
  int small_x = traffic_light.x + (traffic_light.width >> 2);
  int small_w = traffic_light.width >> 1;
  cv::Mat roi = frame(cv::Rect(small_x, traffic_light.y, small_w, traffic_light.height));

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
  cv::threshold(gray_img, th_img, 10, 255, cv::THRESH_BINARY_INV);

  int row = static_cast<int>(traffic_light.height / 3);

  int count1 = 0;
  int count2 = 0;
  int count3 = 0;
  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < traffic_light.width; ++j)
    {
      if (th_img.at<int>(i, j) < 50)
      {
        count1++;
      }
      if (th_img.at<int>(i + row, j) < 50)
      {
        count2++;
      }
      if (th_img.at<int>(i + row * 2, j) < 50)
      {
        count3++;
      }
    }
  }
  int decision1 = static_cast<int>(count1 * 100 / traffic_light.square);
  int decision2 = static_cast<int>(count2 * 100 / traffic_light.square);
  int decision3 = static_cast<int>(count3 * 100 / traffic_light.square);

  int min = std::min(decision1, decision3);
  int max = std::max(decision1, decision3);

  // ROS_INFO("D1 %d | D2 %d | D3 %d | min %d, | max %d", decision1, decision2, decision3, min, max);

  if (decision2 > 100)
  {
    traffic_light.color = 1;
  }
  else if (max > 30 && max < 60)
  {
    traffic_light.color = 0;
  }
  else if (max > 100)
  {
    traffic_light.color = 2;
  }
  else
  {
    traffic_light.color = -1;
  }

  publish();
  if (enable_debug)
    ROS_INFO("Traffic light #[%d] detected", traffic_light.color);
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
    cv::waitKey(1);
  }

  return 0;
}
