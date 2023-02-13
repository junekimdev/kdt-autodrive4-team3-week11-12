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
constexpr float ANGLE_DIV = 2.f;

bool Mode::isStopGroupDetected(const sensor::State& sensor_state)
{
  return sensor_state.sign.stop_detected || sensor_state.sign.stop_detected ||
         sensor_state.light.color == traffic::Light::Color::Red;
}

void Mode::setAngle(const sensor::State& sensor_state, int16_t center_pos)
{
  angle = static_cast<int16_t>((center_pos - sensor_state.cam.width) / ANGLE_DIV);
  // Compensate angle for lane lost
  if (!sensor_state.cam.left_detected)
    angle -= ANGLE_MORE;
  if (!sensor_state.cam.right_detected)
    angle += ANGLE_MORE;
  // Correct angle
  angle += ANGLE_CENTER;
  if (angle > MAX_ANGLE)
    angle = MAX_ANGLE;
  if (angle < MIN_ANGLE)
    angle = MIN_ANGLE;
}

void Mode::setAngleToFollowLeft(const sensor::State& sensor_state)
{
  int16_t cp = sensor_state.cam.lpos + sensor_state.cam.lane_width / 2;
  setAngle(sensor_state, cp);
}

void Mode::setAngleToFollowRight(const sensor::State& sensor_state)
{
  int16_t cp = sensor_state.cam.rpos - sensor_state.cam.lane_width / 2;
  setAngle(sensor_state, cp);
}

Mode Mode::transitFromInit(const sensor::State& sensor_state)
{
  bool ready =
      sensor_state.cam.online && sensor_state.sign.online && sensor_state.light.online && sensor_state.stop_line.online;
  if (ready)
    return Mode(Id::Ready, this);
  return *this;
}

Mode Mode::transitFromReady(const sensor::State& sensor_state)
{
  if (speed)
  {
    if (sensor_state.sign.left_detected)
    {
      setAngleToFollowLeft(sensor_state);
      return Mode(Id::Left, this);
    }
    if (sensor_state.sign.right_detected)
    {
      setAngleToFollowRight(sensor_state);
      return Mode(Id::Right, this);
    }
  }

  bool need_to_go = sensor_state.light.detected && sensor_state.light.color == traffic::Light::Color::Green;
  if (need_to_go)
    speed = DRIVE_SPEED;
  return *this;
}

Mode Mode::transitFromLeft(const sensor::State& sensor_state)
{
  bool need_to_stop =
      speed != 0 && timer.stamp_ms() > TIMER_IGNORE_MS && isStopGroupDetected() && !sensor_state.stop_line.detected;
  if (need_to_stop)
    return Mode(Id::LeftPrepToStop, this);
  if (sensor_state.sign.right_detected)
  {
    setAngleToFollowRight(sensor_state);
    return Mode(Id::Right, this);
  }

  setAngleToFollowLeft(sensor_state);
  return *this;
}
Mode Mode::transitFromLeftPrepToStop(const sensor::State& sensor_state)
{
  if (sensor_state.stop_line.detected)
  {
    speed = 0;
    return Mode(Id::LeftStop, this);
  }
  return *this;
}
Mode Mode::transitFromLeftStop(const sensor::State& sensor_state)
{
  bool need_to_go = speed == 0 && timer.stamp_ms() > TIMER_IGNORE_MS;
  if (need_to_go)
  {
    speed = DRIVE_SPEED;
    setAngleToFollowLeft(sensor_state);
    return Mode(Id::Left, this);
  }
  return *this;
}
Mode Mode::transitFromRight(const sensor::State& sensor_state)
{
  bool need_to_stop =
      speed != 0 && timer.stamp_ms() > TIMER_IGNORE_MS && isStopGroupDetected() && !sensor_state.stop_line.detected;
  if (need_to_stop)
    return Mode(Id::RightPrepToStop, this);
  if (sensor_state.sign.right_detected)
  {
    setAngleToFollowLeft(sensor_state);
    return Mode(Id::Left, this);
  }

  setAngleToFollowRight(sensor_state);
  return *this;
}
Mode Mode::transitFromRightPrepToStop(const sensor::State& sensor_state)
{
  if (sensor_state.stop_line.detected)
  {
    speed = 0;
    return Mode(Id::RightStop, this);
  }
  return *this;
}
Mode Mode::transitFromRightStop(const sensor::State& sensor_state)
{
  bool need_to_go = speed == 0 && timer.stamp_ms() > TIMER_IGNORE_MS;
  if (need_to_go)
  {
    speed = DRIVE_SPEED;
    setAngleToFollowRight(sensor_state);
    return Mode(Id::Right, this);
  }
  return *this;
}

Mode Mode::next(const sensor::State& sensor_state)
{
  switch (id)
  {
    case Id::Init:
      return transitFromInit(sensor_state);
    case Id::Ready:
      return transitFromReady(sensor_state);
    case Id::Left:
      return transitFromLeft(sensor_state);
    case Id::LeftPrepToStop:
      return transitFromLeftPrepToStop(sensor_state);
    case Id::LeftStop:
      return transitFromLeftStop(sensor_state);
    case Id::Right:
      return transitFromRight(sensor_state);
    case Id::RightPrepToStop:
      return transitFromRightPrepToStop(sensor_state);
    case Id::RightStop:
      return transitFromRightStop(sensor_state);
    default:
      return *this;
  }
}
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
    node.param<bool>("controller_enable_debug", enable_debug, true);
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
  void publish();
  void control();
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
  sensor_state.object.traffic_light.reduce(msg)
}

void Controller::publish()
{
  xycar_msgs::xycar_motor msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = control::NAME;
  msg.angle = control_state.mode.angle;
  msg.speed = control_state.mode.speed;
  pub.publish(msg);
}

void Controller::control()
{
  control_state.reduce(sensor_state);
  publish();
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
