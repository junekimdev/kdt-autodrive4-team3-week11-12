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
constexpr int HEIGHT = 480;
constexpr int SCAN_ROW = 380;
constexpr double GAUSIAN_BLUR_SIGMA = 2.;
constexpr int ROI_HEIGHT = 30;
constexpr int ROI_Y = SCAN_ROW - (ROI_HEIGHT >> 1);
constexpr int ROI_GAP = 8;

const cv::Size ROI_SIZE_FULL = cv::Size(WIDTH, ROI_HEIGHT);
const cv::Size ROI_SIZE_WIDE = cv::Size(WIDTH >> 1, ROI_HEIGHT);
const cv::Size ROI_SIZE_NORM = cv::Size(WIDTH >> 2, ROI_HEIGHT);
const cv::Rect ROI_FULL = cv::Rect(0, ROI_Y, WIDTH, ROI_HEIGHT);
const cv::Rect ROI_L_NULL = cv::Rect(0, ROI_Y, 1, ROI_HEIGHT);
const cv::Rect ROI_R_NULL = cv::Rect(WIDTH - 1, ROI_Y, 1, ROI_HEIGHT);
const cv::Rect ROI_L_INIT = cv::Rect(cv::Point(0, ROI_Y), ROI_SIZE_WIDE);
const cv::Rect ROI_R_INIT = cv::Rect(cv::Point(WIDTH >> 1, ROI_Y), ROI_SIZE_WIDE);

inline cv::Rect getRoiRectL(int lx, int w, int rightCut = WIDTH)
{
  // Prep args
  if (lx < 0)
    lx = 0;
  if (rightCut < 1)
    rightCut = 1;
  if (lx == rightCut)
    lx--;

  int rx = lx + w;
  if (rx > rightCut)
    rx = rightCut;
  return cv::Rect(cv::Point(lx, ROI_Y), cv::Point(rx, ROI_Y + ROI_HEIGHT));
}

inline cv::Rect getRoiRectR(int rx, int w, int leftCut = 0)
{
  // Prep args
  if (rx > WIDTH)
    rx = WIDTH;
  if (leftCut > WIDTH - 1)
    leftCut = WIDTH - 1;
  if (rx == leftCut)
    rx++;

  int lx = rx - w;
  if (lx < leftCut)
    lx = leftCut;
  return cv::Rect(cv::Point(lx, ROI_Y), cv::Point(rx, ROI_Y + ROI_HEIGHT));
}

inline std::vector<int> convertToVideoPos(const std::vector<cv::Point>& pts, const int topLeftX, const int lostPos)
{
  int lp = pts[0].x + topLeftX, rp = pts[1].x + topLeftX;
  if (rp - lp < 1)  // lane lost
  {
    lp = lostPos;
    rp = lostPos;
  }
  return { lp, rp };
}

std::vector<cv::Point> findEdges(const cv::Mat& img)
{
  cv::Mat img32, blr, dx;
  img.convertTo(img32, CV_32F);
  cv::GaussianBlur(img32, blr, cv::Size(), GAUSIAN_BLUR_SIGMA);
  cv::Sobel(blr, dx, CV_32F, 1, 0);

  double leftsideV1, rightsideV1, leftsideV2, rightsideV2, leftsideV3, rightsideV3;
  cv::Point leftsidePt1, rightsidePt1, leftsidePt2, rightsidePt2, leftsidePt3, rightsidePt3;

  int centerY = ROI_HEIGHT / 2;
  cv::Mat roi1 = dx.row(centerY);
  cv::Mat roi2 = dx.row(centerY + ROI_GAP);
  cv::Mat roi3 = dx.row(centerY - ROI_GAP);
  cv::minMaxLoc(roi1, &leftsideV1, &rightsideV1, &leftsidePt1, &rightsidePt1);
  cv::minMaxLoc(roi2, &leftsideV2, &rightsideV2, &leftsidePt2, &rightsidePt2);
  cv::minMaxLoc(roi3, &leftsideV3, &rightsideV3, &leftsidePt3, &rightsidePt3);

  cv::Point leftsidePt, rightsidePt;
  leftsidePt.x = (int)((leftsidePt1.x + leftsidePt2.x + leftsidePt3.x) / 3.f + .5f);
  rightsidePt.x = (int)((rightsidePt1.x + rightsidePt2.x + rightsidePt3.x) / 3.f + .5f);

  return { leftsidePt, rightsidePt };
}

class LaneCam
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  cv::Rect roiRectL;
  cv::Rect roiRectR;
  bool leftDetected;
  bool rightDetected;
  int lpos;
  int rpos;

public:
  bool enable_debug;

  LaneCam()
    : roiRectL(ROI_L_INIT), roiRectR(ROI_R_INIT), leftDetected(false), rightDetected(false), lpos(0), rpos(WIDTH - 1)
  {
    node.param<bool>("sensor_lane_cam_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::callback, this);
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
  msg.header.frame_id = name;

  msg.width = vFrame.cols;
  msg.leftDetected = leftDetected;
  msg.rightDetected = rightDetected;
  msg.lpos = lpos;
  msg.rpos = rpos;

  pub.publish(msg);
  if (enable_debug)
    ROS_INFO("lpos: %d | rpos: %d", lpos, rpos);
}

void LaneCam::process()
{
  // Convert to Gray
  cv::Mat grayFrame;
  cv::cvtColor(vFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // Find lines
  cv::Mat roiL = grayFrame(roiRectL);
  cv::Mat roiR = grayFrame(roiRectR);
  std::vector<cv::Point> ptsL = findEdges(roiL);
  std::vector<cv::Point> ptsR = findEdges(roiR);
  std::vector<int> pxL = convertToVideoPos(ptsL, roiRectL.x, 0);
  std::vector<int> pxR = convertToVideoPos(ptsR, roiRectR.x, WIDTH - 1);

  int left = cvRound((pxL[0] + pxL[1]) / 2.f);
  int right = cvRound((pxR[0] + pxR[1]) / 2.f);
  lpos = left;
  rpos = right;

  // Update roi for next
  // When undetected, lpos & rpos will be kept as previous values
  bool goodL = pxL[0] != pxL[1];
  bool goodR = pxR[0] != pxR[1];
  if (goodL && goodR)
  {
    // None lost
    // lpos = left;
    // rpos = right;

    int lx = left - (ROI_SIZE_NORM.width >> 1);
    int rx = right + (ROI_SIZE_NORM.width >> 1);
    int mid = (int)((left + right) / 2.f + .5f);
    roiRectL = getRoiRectL(lx, ROI_SIZE_NORM.width, mid);
    roiRectR = getRoiRectR(rx, ROI_SIZE_NORM.width, mid);
  }
  else if (goodL)
  {
    // Right line lost
    // lpos = left;

    int lx = left - (ROI_SIZE_NORM.width >> 1);
    int rx = right + (ROI_SIZE_WIDE.width >> 1);
    roiRectL = getRoiRectL(lx, ROI_SIZE_NORM.width);  // the order is important
    roiRectR = getRoiRectR(rx, ROI_SIZE_WIDE.width, roiRectL.br().x);
  }
  else if (goodR)
  {
    // Left line lost
    // rpos = right;

    int lx = left - (ROI_SIZE_WIDE.width >> 1);
    int rx = right + (ROI_SIZE_NORM.width >> 1);
    roiRectR = getRoiRectR(rx, ROI_SIZE_NORM.width);  // the order is important
    roiRectL = getRoiRectL(lx, ROI_SIZE_WIDE.width, roiRectR.tl().x);
  }
  else
  {
    // All lost
    if (leftDetected)
    {
      // Stand by to find L line
      roiRectL = ROI_FULL;
      roiRectR = ROI_R_NULL;
    }
    else if (rightDetected)
    {
      // Stand by to find R line
      roiRectL = ROI_L_NULL;
      roiRectR = ROI_FULL;
    }
  }
  leftDetected = goodL;
  rightDetected = goodR;

  if (enable_debug)
  {
    cv::rectangle(vFrame, roiRectL, BLACK, 2);
    cv::rectangle(vFrame, roiRectR, BLACK, 2);
    cv::drawMarker(vFrame, cv::Point(lpos, SCAN_ROW), YELLOW, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    cv::drawMarker(vFrame, cv::Point(rpos, SCAN_ROW), BLUE, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    cv::line(vFrame, cv::Point(0, SCAN_ROW), cv::Point(WIDTH, SCAN_ROW), BLUE, 1);
    cv::line(vFrame, cv::Point(0, SCAN_ROW + ROI_GAP), cv::Point(WIDTH, SCAN_ROW + ROI_GAP), BLUE, 1);
    cv::line(vFrame, cv::Point(0, SCAN_ROW - ROI_GAP), cv::Point(WIDTH, SCAN_ROW - ROI_GAP), BLUE, 1);
    cv::imshow(WINDOW_TITLE, vFrame);
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
