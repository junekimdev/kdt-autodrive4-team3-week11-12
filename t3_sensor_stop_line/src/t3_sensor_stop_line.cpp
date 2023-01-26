#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "t3_msgs/stop_line_data.h"

// Include OpenCV
#include "opencv2/opencv.hpp"

namespace color
{
const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
}  // namespace color

namespace sensor
{

const std::string NAME = "stop_line";
const std::string SUB_TOPIC = "usb_cam/image_raw";
const std::string PUB_TOPIC = "stop_line_data";

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

class StopLine
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  bool detected;
  int count;
  cv::Mat vFrame;


public:
  bool enable_debug;
  t3_msgs::stop_line_data msg;

  StopLine()
  {
    node.param<bool>("sensor_stop_line_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &StopLine::callback, this);
    pub = node.advertise<t3_msgs::stop_line_data>(PUB_TOPIC, 1);
    if (enable_debug)
      cv::namedWindow(NAME);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void publish();
  void process();
};

void StopLine::callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
    process();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

void StopLine::publish()
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;
  pub.publish(msg);
  
  if (enable_debug)
    ROS_INFO("stop line detected: %s", msg.detected ? "true" : "false");
}

void StopLine::process()
{
  cv::Mat gray_image;
  cv::Mat blur_image;
  cv::Mat canny_image;
  cv::cvtColor(this->vFrame, this->vFrame, cv::COLOR_BGR2RGB);

  cv::cvtColor(this->vFrame,gray_image, cv::COLOR_RGB2GRAY);
  cv::GaussianBlur(gray_image, blur_image, cv::Size(5, 5), 2);
  cv::Canny(blur_image, canny_image, 50, 150);
  cv::Mat roi = canny_image(cv::Rect(100, 370, 440, 50));
  
  std::vector<cv::Vec4i> all_lines;
  cv::HoughLinesP(roi, all_lines, 1.0, CV_PI / 180.0, 40, 40, 5);
  if (all_lines.size() >= 0){
    count = 0;
    for(size_t i = 0 ; i <all_lines.size(); i++){
      cv::Vec4i l = all_lines[i];
      if (abs(l[1]-l[3]) < 10)
        //line(this->vFrame, cv::Point(l[0] +100, l[1]+370), cv::Point(l[2]+100, l[3]+370), cv::Scalar(0,0,255), 3, cv::LINE_AA);
        count++;
    }
  }
  if (count >= 4)
    msg.detected = true;
  else 
    msg.detected = false;

  this->publish();
  //cv::imshow(NAME, this->vFrame);

}

}  // namespace sensor

int main(int argc, char** argv)
{
  ros::init(argc, argv, sensor::NAME);
  sensor::StopLine stop_line;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  while (ros::ok())
  {
    ros::spinOnce();
    if (stop_line.enable_debug)
    {
      int k = cv::waitKey(1);
      if (k == 27 || k == ' ')  // ESC key or space bar
        break;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
