#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "t3_msgs/lane_data.h"

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
const std::string NAME = "lane_cam";
const std::string SUB_TOPIC = "usb_cam/image_raw";
const std::string PUB_TOPIC = "lane_data";

constexpr int WIDTH = 640;
constexpr int WIDTH_HALF = 320;
constexpr int HEIGHT = 480;
constexpr double GAUSIAN_BLUR_SIGMA = 2.;
constexpr int SCAN_ROW = 380;
constexpr int ROI_HEIGHT_HALF = 20;

const cv::Rect ROI_L_RECT =
    cv::Rect(cv::Point(0, SCAN_ROW - ROI_HEIGHT_HALF), cv::Point(WIDTH_HALF - 1, SCAN_ROW + ROI_HEIGHT_HALF));
const cv::Rect ROI_R_RECT =
    cv::Rect(cv::Point(WIDTH_HALF, SCAN_ROW - ROI_HEIGHT_HALF), cv::Point(WIDTH - 1, SCAN_ROW + ROI_HEIGHT_HALF));

cv::Point getLinePositionInside(const cv::Mat& src, bool is_left)
{
  // Smooth out
  cv::Mat smooth;
  cv::GaussianBlur(src, smooth, cv::Size(), GAUSIAN_BLUR_SIGMA);

  // Find edges by Canny
  cv::Mat canny;
  cv::Canny(smooth, canny, 50, 150);

  // Scan a row
  int scan_row = src.rows / 2;
  cv::Mat a_row = canny.row(scan_row);

  // Find points of edges
  std::vector<cv::Point> pts;
  cv::findNonZero(a_row, pts);

  // Set min/max points
  cv::Point min_point(0, scan_row);
  cv::Point max_point(src.cols, scan_row);
  if (pts.size())
  {
    min_point = *pts.begin() + cv::Point(0, scan_row);
    max_point = *pts.rbegin() + cv::Point(0, scan_row);
  }
  else
  {
    std::swap(min_point, max_point);
  }

  return is_left ? max_point : min_point;
}

class LaneCam
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  bool left_detected;
  bool right_detected;
  int lpos;
  int rpos;

public:
  bool enable_debug;

  LaneCam() : left_detected(false), right_detected(false), lpos(0), rpos(WIDTH - 1)
  {
    node.param<bool>("sensor_lane_cam_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &LaneCam::callback, this);
    pub = node.advertise<t3_msgs::lane_data>(PUB_TOPIC, 1);
    if (enable_debug)
      cv::namedWindow(NAME);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void publish();
  void process();
};

void LaneCam::callback(const sensor_msgs::ImageConstPtr& msg)
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

void LaneCam::publish()
{
  t3_msgs::lane_data msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;

  msg.width = vFrame.cols;
  msg.left_detected = left_detected;
  msg.right_detected = right_detected;
  msg.lpos = lpos;
  msg.rpos = rpos;

  pub.publish(msg);
  if (enable_debug)
    ROS_INFO("lpos: %d | rpos: %d", lpos, rpos);
}

void LaneCam::process()
{
  // Convert to Gray
  cv::Mat gray_frame;
  cv::cvtColor(vFrame, gray_frame, cv::COLOR_BGR2GRAY);

  // Find lines
  const cv::Mat roiL = gray_frame(ROI_L_RECT);
  const cv::Mat roiR = gray_frame(ROI_R_RECT);
  const cv::Point pL = getLinePositionInside(roiL, true) + ROI_L_RECT.tl();
  const cv::Point pR = getLinePositionInside(roiR, true) + ROI_R_RECT.tl();
  const cv::Point pC = (pL + pR) / 2;

  if (enable_debug)
  {
    cv::rectangle(vFrame, ROI_L_RECT, color::BLACK, 2);
    cv::rectangle(vFrame, ROI_R_RECT, color::BLACK, 2);
    cv::drawMarker(vFrame, cv::Point(lpos, SCAN_ROW), color::YELLOW, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    cv::drawMarker(vFrame, cv::Point(rpos, SCAN_ROW), color::BLUE, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    cv::line(vFrame, cv::Point(0, SCAN_ROW), cv::Point(WIDTH, SCAN_ROW), color::BLUE, 1);
    cv::imshow(NAME, vFrame);
  }
  publish();
}

}  // namespace sensor

int main(int argc, char** argv)
{
  ros::init(argc, argv, sensor::NAME);
  sensor::LaneCam lane_cam;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  while (ros::ok())
  {
    ros::spinOnce();

    if (lane_cam.enable_debug)
    {
      int k = cv::waitKey(1);
      if (k == 27 || k == ' ')  // ESC key or space bar
        break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}
