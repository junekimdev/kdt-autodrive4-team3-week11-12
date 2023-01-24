#include <string>

#include "ros/console.h"
#include "ros/ros.h"

// Messages
#include "t3_msgs/lane_data.h"
#include "t3_msgs/object_data.h"
#include "t3_msgs/stop_line_data.h"
#include "t3_msgs/traffic_light_data.h"
#include "xycar_msgs/xycar_motor.h"

#include "t3_controller/control_state.h"
#include "t3_controller/sensor_state.h"

namespace control
{
const std::string NAME = "controller";
const std::string SUB_TOPIC_LANE = "lane_data";
const std::string SUB_TOPIC_OBJECT = "object_data";
const std::string SUB_TOPIC_STOP_LINE = "stop_line_data";
const std::string SUB_TOPIC_TRAFFIC_LIGHT = "traffic_light_data";
const std::string PUB_TOPIC = "xycar_motor";
constexpr int SPEED = 10;
constexpr float ANGLE_DIV = 2.f;

class Controller
{
  ros::NodeHandle node;
  ros::Subscriber sub_lane;
  ros::Subscriber sub_object;
  ros::Subscriber sub_stop_line;
  ros::Subscriber sub_traffic_light;
  ros::Publisher pub;
  control::State control_state;
  sensor::State sensor_state;

public:
  bool enable_debug;

  Controller()
  {
    node.param<bool>("controller_enable_debug", enable_debug, false);
    sub_lane = node.subscribe(SUB_TOPIC_LANE, 1, &Controller::callbackLane, this);
    sub_object = node.subscribe(SUB_TOPIC_OBJECT, 1, &Controller::callbackObject, this);
    sub_stop_line = node.subscribe(SUB_TOPIC_STOP_LINE, 1, &Controller::callbackStopLine, this);
    sub_traffic_light = node.subscribe(SUB_TOPIC_TRAFFIC_LIGHT, 1, &Controller::callbackTrafficLight, this);
    pub = node.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC, 1);
  }

  void callbackLane(const t3_msgs::lane_data::ConstPtr& msg);
  void callbackObject(const t3_msgs::object_data::ConstPtr& msg);
  void callbackStopLine(const t3_msgs::stop_line_data::ConstPtr& msg);
  void callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg);
  void control();

  // xycar_msgs::xycar_motor create_msg()
  // {
  //   xycar_msgs::xycar_motor msg;
  //   msg.header.stamp = ros::Time::now();
  //   msg.header.frame_id = control::NAME;
  //   msg.angle = control_state.angle;
  //   msg.speed = control_state.speed;
  //   return msg;
  // }
};

void Controller::callbackLane(const t3_msgs::lane_data::ConstPtr& msg)
{
  sensor_state.cam.reduce(msg)
}
void Controller::callbackObject(const t3_msgs::object_data::ConstPtr& msg)
{
  sensor_state.object.reduce(msg)
}
void Controller::callbackStopLine(const t3_msgs::stop_line_data::ConstPtr& msg)
{
  sensor_state.stop_line.reduce(msg)
}
void Controller::callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg)
{
  sensor_state.traffic_light.reduce(msg)
}

void Controller::control()
{
  // TODO
}

}  // namespace control

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
