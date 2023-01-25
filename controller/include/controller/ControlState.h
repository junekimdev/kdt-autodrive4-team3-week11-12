#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <cmath>
#include <numeric>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "sensor_cam_hough/cam_msg.h"
#include "xycar_msgs/xycar_motor.h"
#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"
#include "yolov3_trt_ros/TrafficBox.h"
#include "yolov3_trt_ros/TrafficBoxes.h"

// Const
const std::string CONTROLLER_NAME = "controller";

constexpr int ANGLE_CENTER = 8;
constexpr int MAX_ANGLE = 50;
constexpr int MIN_ANGLE = -50;
constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

enum struct DRIVE_MODE { STOP, GO };

struct ControlState {
  DRIVE_MODE mode;
  int angle;
  int speed;
  bool isStarted;

  ControlState()
      : mode(DRIVE_MODE::STOP),
        angle(ANGLE_CENTER),
        speed(0),
        isStarted(false) {}

  void reduce(DRIVE_MODE mode, int angle, int speed) {
    this->mode = mode;
    this->angle = angle;
    this->speed = speed;
  }
};

struct ObjectState
{
  int id_;
  float probability_;
  int xmin_;
  int ymin_;
  int xmax_;
  int ymax_;

  ObjectState(int id,float probability,int xmin,int ymin,int xmax,int ymax)
      :id_(id),probability_(probability),xmin_(xmin),ymin_(ymin),xmax_(xmax),ymax_(ymax){}

};

struct SensorObjectState{
  int objectNum;
  std::vector<ObjectState> objects;
  
  SensorObjectState()
  	: objectNum(0),
  	  objects(std::vector<ObjectState>()){}
  void reduce(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg);
  void update(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg);
};

void SensorObjectState::reduce(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg){
  this->update(msg);
}

void SensorObjectState::update(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg){
  this->objectNum = msg->bounding_boxes.size();
  
  auto boxes = msg-> bounding_boxes;
  this->objects = {};
  //std::cout << typeid(boxes).name() << std::endl;
  if(msg->bounding_boxes.size() >0){
    for(auto box: msg->bounding_boxes){
      auto object = ObjectState(box.id,
        box.probability,
        box.xmin,
        box.ymin,
        box.xmax,
        box.ymax);
        objects.push_back(object);
    }
  }else{
  	std::cout << "NO MESSAGE" << std::endl;
  }
  std::cout << this->objects.size() <<std::endl;
  for(auto i: this->objects){
  	std::cout << i.id_ << std::endl;
  }
  
}



struct SensorCamState {
  bool isLeftDetected;
  bool isRightDetected;
  int width;
  int lpos;
  int rpos;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;

  SensorCamState()
      : isLeftDetected(false),
        isRightDetected(false),
        width(WIDTH),
        lpos(0),
        rpos(WIDTH - 1),
        lposMemo(std::vector<int>(SMA_NUM, 0)),
        rposMemo(std::vector<int>(SMA_NUM, WIDTH - 1)),
        lposSMA(0),
        rposSMA(WIDTH - 1) {}

  void reduce(const sensor_cam::cam_msg::ConstPtr& msg);
  void update(const sensor_cam::cam_msg::ConstPtr& msg);
  void filter();
};

void SensorCamState::reduce(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->update(msg);
  this->filter();
}

void SensorCamState::update(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->isLeftDetected = msg->isLeftDetected;
  this->isRightDetected = msg->isRightDetected;
  this->width = msg->width;
  if (msg->isLeftDetected) this->lpos = msg->lpos;
  if (msg->isRightDetected) this->rpos = msg->rpos;
}
void SensorCamState::filter() {
  lposMemo.erase(lposMemo.begin());
  rposMemo.erase(rposMemo.begin());
  lposMemo.emplace_back(this->lpos);
  rposMemo.emplace_back(this->rpos);
  this->lposSMA = (int)(std::accumulate(lposMemo.begin(), lposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
  this->rposSMA = (int)(std::accumulate(rposMemo.begin(), rposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
}

struct SensorState {
  SensorCamState cam;
  SensorObjectState objects;
  SensorState() {}
};

#endif