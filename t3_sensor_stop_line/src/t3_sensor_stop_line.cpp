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
constexpr int ROI_X = 100;
constexpr int ROI_Y = 370;
constexpr int ROI_W = 440;
constexpr int ROI_H = 50;
constexpr int GAUSSIAN_KERNEL_SIZE = 5;
constexpr double GAUSSIAN_SIGMA = 2.;
constexpr double CANNY_THRESH_1 = 50;
constexpr double CANNY_THRESH_2 = 150;
constexpr double HOUGH_RHO = 1.;
constexpr double HOUGH_THETA = CV_PI / 180.0;
constexpr int HOUGH_THRESH = 40;
constexpr double HOUGH_MIN_LINE_LENGTH = 40.;
constexpr double HOUGH_MAX_LINE_GAP = 5.;

class StopLine
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  bool detected;
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
  msg.detected = detected;
  pub.publish(msg);

  if (enable_debug)
    ROS_INFO("stop line detected: %s", detected ? "true" : "false");
}

void StopLine::process()
{
  cv::Mat gray_image;
  cv::Mat blur_image;
  cv::Mat canny_image;
  cv::cvtColor(this->vFrame, this->vFrame, cv::COLOR_BGR2RGB);

  cv::cvtColor(this->vFrame, gray_image, cv::COLOR_RGB2GRAY);
  cv::GaussianBlur(gray_image, blur_image, cv::Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_SIGMA);
  cv::Canny(blur_image, canny_image, CANNY_THRESH_1, CANNY_THRESH_2);
  cv::Mat roi = canny_image(cv::Rect(ROI_X, ROI_Y, ROI_W, ROI_H));

  std::vector<cv::Vec4i> all_lines;
  cv::HoughLinesP(roi, all_lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
  detected = false;
  int count = 0;
  if (all_lines.size() >= 0)
  {
    for (const auto& line : all_lines)
    {
      if (abs(line[1] - line[3]) < 10)
        count++;
    }
  }
  if (count >= 4)
    detected = true;

  publish();
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
