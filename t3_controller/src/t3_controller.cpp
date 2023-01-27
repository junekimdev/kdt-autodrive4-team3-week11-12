#include <string>

#include "ros/console.h"
#include "ros/ros.h"
#include <vector>
#include <unistd.h>
#include <iostream>


// Messages
#include "t3_msgs/lane_data.h"
#include "t3_msgs/object_data.h"
#include "t3_msgs/stop_line_data.h"
#include "t3_msgs/traffic_light_data.h"
#include "xycar_msgs/xycar_motor.h"

#include "t3_controller/control_state.h"
#include "t3_controller/sensor_state.h"




namespace control{
  const std::string NAME = "controller";
  const std::string SUB_TOPIC_LANE = "lane_data";
  const std::string SUB_TOPIC_OBJECT = "object_data";
  const std::string SUB_TOPIC_STOP_LINE = "stop_line_data";
  const std::string SUB_TOPIC_TRAFFIC_LIGHT = "traffic_light_data";
  const std::string PUB_TOPIC = "xycar_motor";
  constexpr int SPEED = 5;
  constexpr float ANGLE_DIV = 1.f;

  control_state::State control_state_;
  sensor_state::State sensor_state_;

  struct TmpObject
  {
    int id;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
    float probability;
    TmpObject(){};

 };


  class Controller
  {
    ros::NodeHandle node;
    ros::Subscriber sub_lane;
    ros::Subscriber sub_object;
    ros::Subscriber sub_stop_line;
    ros::Subscriber sub_traffic_light;
    ros::Publisher pub;
  public:
    bool enable_debug;

    Controller()
    {
      node.param<bool>("controller_enable_debug", enable_debug, false);
      this->sub_lane = this->node.subscribe(SUB_TOPIC_LANE, 1, &Controller::callbackLane, this);
      this->sub_object = this->node.subscribe(SUB_TOPIC_OBJECT, 1, &Controller::callbackObject, this);
      this->sub_stop_line = this->node.subscribe(SUB_TOPIC_STOP_LINE, 1, &Controller::callbackStopLine, this);
      //sub_traffic_light = node.subscribe(SUB_TOPIC_TRAFFIC_LIGHT, 1, &Controller::callbackTrafficLight, this);
      pub = node.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC, 1);
    }

    void callbackLane(const t3_msgs::lane_data::ConstPtr& msg);
    void callbackObject(const t3_msgs::object_data::ConstPtr& msg);
    void callbackStopLine(const t3_msgs::stop_line_data::ConstPtr& msg);
    //void callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg);
    void control();
  };

  int correctAngle(int angle) {
    angle += control_state::ANGLE_CENTER;
    if (angle > control_state::ANGLE_MAX) angle = control_state::ANGLE_MAX;
    if (angle < control_state::ANGLE_MIN) angle = control_state::ANGLE_MIN;
    return angle;
  }
  

  xycar_msgs::xycar_motor create_msg()
  {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = control::NAME;
    msg.angle = control_state_.angle;
    msg.speed = control_state_.speed;
    
    return msg;
  }
  
  
  
  void Controller::callbackLane(const t3_msgs::lane_data::ConstPtr& msg)
  {
    sensor_state_.cam.reduce(msg);
  }
  
  void Controller::callbackObject(const t3_msgs::object_data::ConstPtr& msg)
  {
    sensor_state_.object.reduce(msg);
  }
  
  void Controller::callbackStopLine(const t3_msgs::stop_line_data::ConstPtr& msg)
  {
    sensor_state_.stop_line.reduce(msg);
  }
  /*
  void Controller::callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg)
  {
    //sensor_state.traffic_light.reduce(msg)
  }
 */ 
  void Controller::control()
  {
    std::vector<control::TmpObject> tmpObject;

    // sensor_state_.object.boundingBoxes.size()+sensor_state_.traffic_light.count
    if(sensor_state_.object.boundingBoxes.size()>0)
    {
      for(auto objectBox :sensor_state_.object.boundingBoxes){
        control::TmpObject tmp;
        tmp.id = objectBox.id;
        tmp.xmin = objectBox.xmin;
        tmp.ymin = objectBox.ymin;
        tmp.xmax = objectBox.xmax;
        tmp.ymax = objectBox.ymax;
        tmp.probability = objectBox.probability;
        tmpObject.emplace_back(tmp);
      }
    }
    if(sensor_state_.traffic_light.count>0){
      control::TmpObject tmp1;
        tmp1.id = sensor_state_.traffic_light.boundingBox.id;
        tmp1.xmin = sensor_state_.traffic_light.boundingBox.xmin;
        tmp1.xmax = sensor_state_.traffic_light.boundingBox.xmax;
        tmp1.ymax = sensor_state_.traffic_light.boundingBox.ymax;
        tmp1.probability = sensor_state_.traffic_light.boundingBox.probability;
        tmpObject.emplace_back(tmp1);
    }
    int anchor_box_min = 1000;
    int anchor_box_max = 5000;
    float ratio_min, ratio_max;
    float square;
    int new_id = -1;
    for(auto& i :tmpObject){
      int height = i.ymax-i.ymin;
      int weight = i.xmax-i.xmin;
      float ratio = height/weight;
      //std::cout << ratio << std::endl;
      if(i.id != 5)
      {
        ratio_min = 0.75;
        ratio_max = 1.725;
        square = weight * height;
      } 
      else
      {
        ratio_min = 1.35;
        ratio_max = 2.4;
        square = weight * height * 0.75;
      }

      if (ratio_min < ratio && ratio_max > ratio){
        if (weight > 31 && weight <93){
          if (anchor_box_min> square){
            continue;
          }
          else {
            anchor_box_min = square;
            new_id = i.id;
            }
        }
      }
    }




    // TODO
    // switch (sensor_state_)
    // {
    // case /* constant-expression */:
    //   /* code */
    //   break;
    
    // default:
    //   break;
    // }

    int lpos_PID = sensor_state_.cam.lpos*0.7 + sensor_state_.cam.lposSMA*0.3;
    int rpos_PID = sensor_state_.cam.rpos*0.7 + sensor_state_.cam.rposSMA*0.3;
    if (sensor_state_cam.left_detected == true || sensor_state_cam.left_detected == false ){
     rpos_PID = lpos_PID +358;
    }
    else if (sensor_state_cam.left_detected == true || sensor_state_cam.left_detected == false ){
     lpos_PID = rpos_PID -358;
    }
    float cposViewCam;
    
    if(new_id == 0){
      cposViewCam = lpos_PID + 174;

    }
    else if(new_id == 1){
      cposViewCam = rpos_PID - 174;
    }
    else {
    cposViewCam =
      (lpos_PID + rpos_PID) / 2.f;
    }
    /*
    float cposCam = 
      (sensor_state_.cam.lposSMA + sensor_state_.cam.rposSMA) / 2.f;
    */
    int angle;
    float speed;
    control_state::Mode mode;
    angle = (int)((cposViewCam-320) / ANGLE_DIV + .2f);

    angle = correctAngle(angle);
    /*
    if (angle < -30){
      angle = -50;
    }
    else if (angle > 30){
      angle = 50;
    }
    */
    //std::cout << "Center " << cposViewCam << "Angle " << angle << std::endl;
    speed = 0;
    
    


    // mode = control_state::Mode::Go;

    if (new_id == 2 || new_id == 3){
      if (sensor_state_.stop_line.detected){
        control_state_.reduce(mode, angle, 0);
        pub.publish(create_msg());
        sleep(5);
        control_state_.reduce(mode, angle, 10);
        for (int i = 0 ; i < 50 ; i++){
          pub.publish(create_msg());
          sleep(0.03);
        }
      }
      else {
      control_state_.reduce(mode, angle, 10);
      }
    }
    else{
      control_state_.reduce(mode, angle, 10);

    }
    
    pub.publish(create_msg());

    
  }
}
  // namespace control

int main(int argc, char** argv)
{
  ros::init(argc, argv, control::NAME);
  control::Controller controller;
  ROS_INFO("%s is ONLINE", control::NAME.c_str());

  while (ros::ok())
  {
    
    ros::spinOnce();
    controller.control();
  }

  return 0;
}
