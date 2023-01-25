#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <numeric>
#include <deque>
#include <algorithm>
// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "object_checker/object_control_msg.h"
#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"

// Include States
// #include "object_checker/ControlState.h"

const std::string NODE_NAME = "object_checker";
const std::string SUB_TOPIC = "yolov3_trt_ros/detections";
const std::string PUB_TOPIC = "object_data";


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


// Include OpenCV
//#include "opencv2/opencv.hpp"

class Checker {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  std::vector<ObjectState> objectStates;
  std::deque<int> objectflow;
  object_checker::object_control_msg msg2;
  float max_size_checker;
  float now_size_checker;
  int now_object;
  int object_flag;
public:
  Checker() {
    this->sub = this->node.subscribe(SUB_TOPIC, 1, &Checker::callback, this);
    this->pub = this->node.advertise<object_checker::object_control_msg>(PUB_TOPIC, 1);
  }
  
  void callback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg);
  void process();
  void publish();
};

void Checker::callback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& msg){
  auto boxes = msg-> bounding_boxes;
  objectStates = {};
  max_size_checker = 0;
  now_object = 0;
  object_flag = -1;

  if(msg->bounding_boxes.size() > 0){
    for(auto box: msg->bounding_boxes){
      auto object = ObjectState(box.id,
        box.probability,
        box.xmin,
        box.ymin,
        box.xmax,
        box.ymax);
      now_size_checker = (box.xmax - box.xmin) * (box.ymax - box.ymin) ;
      //std::cout << now_size_checker << std::endl;
      if (max_size_checker < now_size_checker){
        if (objectStates.size() > 5000){
          objectStates.pop_back();
        }
        objectStates.push_back(object);
        max_size_checker = now_size_checker;
      }
    }

    for(auto i: objectStates){
      //std::cout << i.id_ << std::endl;
      now_object = i.id_ + 1;
    }  
  }

  if (objectflow.size() >= 20){
    objectflow.pop_front();
  }
  objectflow.push_back(now_object);


  if (count(objectflow.begin(), objectflow.end(), 0) > 13){
    std::cout << "none_object" << std::endl;
  }
  else if (count(objectflow.begin(), objectflow.end(), 1) > 13){
    std::cout << "left" << std::endl;
    object_flag = 0;
  }
  else if (count(objectflow.begin(), objectflow.end(), 2) > 13){
    std::cout << "right" << std::endl;
    object_flag = 1;
  }
  else if (count(objectflow.begin(), objectflow.end(), 3) > 13){
    std::cout << "stop" << std::endl;
    object_flag = 2;

  }
  else if (count(objectflow.begin(), objectflow.end(), 4) > 13){
    std::cout << "crosswalk" << std::endl;
    object_flag = 3;
  }
  else if (count(objectflow.begin(), objectflow.end(), 5) > 13){
    std::cout << "uturn" << std::endl;
    object_flag = 4;

  }
  else if (count(objectflow.begin(), objectflow.end(), 6) > 13){
    std::cout << "traffic_light" << std::endl;
    object_flag = 5;
  }
  else {
    std::cout << "none_object" << std::endl;
  }
  this->process();
}

void Checker::process(){

  this->publish();
}


void Checker::publish(){
  msg2.header.stamp = ros::Time::now();
  msg2.header.frame_id = PUB_TOPIC;
  msg2.object_control_flag = object_flag;
  this->pub.publish(msg2);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, NODE_NAME);
  Checker Checker;
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
