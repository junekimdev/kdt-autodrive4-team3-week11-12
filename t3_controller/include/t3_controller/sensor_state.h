#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <unordered_map>
#include <algorithm>

#include "t3_msgs/BoundingBox.h"
#include "t3_msgs/lane_data.h"
#include "t3_msgs/object_data.h"
#include "t3_msgs/stop_line_data.h"
#include "t3_msgs/traffic_light_data.h"

namespace sensor
{

constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

// Define states
struct Cam
{
  bool online;
  int width;
  bool left_detected;
  bool right_detected;
  int lpos;
  int rpos;
  std::vector<int> lpos_memo;
  std::vector<int> rpos_memo;
  int lpos_sma;
  int rpos_sma;
  int lane_width;

  Cam()
    : online(false)
    , width(WIDTH)
    , left_detected(false)
    , right_detected(false)
    , lpos(0)
    , rpos(WIDTH - 1)
    , lpos_memo(std::vector<int>(SMA_NUM, 0))
    , rpos_memo(std::vector<int>(SMA_NUM, WIDTH - 1))
    , lpos_sma(0)
    , rpos_sma(WIDTH - 1)
    , lane_width(WIDTH / 4)
  {
  }
  void reduce(const t3_msgs::lane_data::ConstPtr& msg)
  {
    if (!online)
      online = true;
    left_detected = msg->left_detected;
    right_detected = msg->right_detected;
    width = msg->width;
    if (msg->left_detected)
    {
      lpos = msg->lpos;
      lpos_memo.erase(lpos_memo.begin());
      lpos_memo.emplace_back(lpos);
      lpos_sma =
          static_cast<int>(std::accumulate(lpos_memo.begin(), lpos_memo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
    if (msg->right_detected)
    {
      rpos = msg->rpos;
      rpos_memo.erase(rpos_memo.begin());
      rpos_memo.emplace_back(rpos);
      rpos_sma =
          static_cast<int>(std::accumulate(rpos_memo.begin(), rpos_memo.end(), 0) / static_cast<float>(SMA_NUM) + .5f);
    }
    if (msg->left_detected && msg->right_detected)
      lane_width = msg->rpos - msg->lpos;
  }
};

namespace traffic
{

enum struct Id
{
  None = -1,
  Left = 0,
  Right = 1,
  Stop = 2,
  Crosswalk = 3
};

struct Light
{
  enum struct Color
  {
    None = -1,
    Red = 0,
    Yellow = 1,
    Green = 2
  };

  bool online;
  bool detected;
  Color color;
  int area;

  Light() : online(false), detected(false), color(Color::Red), distance(0.f)
  {
  }

  void reduce(const t3_msgs::traffic_light_data::ConstPtr& msg)
  {
    if (!online)
      online = true;
    detected = msg->detected;
    color = static_cast<Color>(msg->color);
    // Traffic light is 2x larger than traffic signs
    area = static_cast<int>((msg->bounding_box.xmax - msg->bounding_box.xmin) *
                            (msg->bounding_box.ymax - msg->bounding_box.ymin));
  }
};

struct Sign
{
  bool online;
  bool left_detected;
  bool right_detected;
  bool stop_detected;
  bool crosswalk_detected;
  std::unordered_map<Id, int> area;
  Id closest;

  Sign()
    : online(false)
    , left_detected(false)
    , right_detected(false)
    , stop_detected(false)
    , crosswalk_detected(false)
    , area({ Id::Left, 0 }, { Id::Right, 0 }, { Id::Stop, 0 }, { Id::Crosswalk, 0 })
    , closest(Id::None)
  {
  }

  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
    if (!online)
      online = true;

    // Reset
    left_detected = false;
    right_detected = false;
    stop_detected = false;
    crosswalk_detected = false;
    for (const auto& id : area)
      area[id] = 0;
    closest = Id::None;

    // Overwrite
    if (sizeof(msg->bounding_boxes))
    {
      for (const auto& bbox : msg->bounding_boxes)
      {
        int bbox_area = static_cast<int>((bbox.bounding_box.xmax - bbox.bounding_box.xmin) *
                                         (bbox.bounding_box.ymax - bbox.bounding_box.ymin));
        switch (static_cast<Id>(bbox.id))
        {
          case Id::Left:
            left_detected = true;
            area[Id::Left] = bbox_area;
            break;
          case Id::Right:
            right_detected = true;
            area[Id::Right] = bbox_area;
            break;
          case Id::Stop:
            stop_detected = true;
            area[Id::Stop] = bbox_area;
            break;
          case Id::Crosswalk:
            crosswalk_detected = true;
            area[Id::Crosswalk] = bbox_area;
            break;
          default:
            break;
        }
      }
      const bool compare = [](const auto& a, const auto& b) { return a.second < b.second; };
      max_pair = *std::max_element(area.begin(), area.end(), compare);
      closest = max_pair.first;
    }
  }
};
}  // namespace traffic

struct StopLine
{
  bool online;
  bool detected;

  StopLine() : online(false), detected(false)
  {
  }

  void reduce(const t3_msgs::stop_line_data::ConstPtr& msg)
  {
    if (!online)
      online = true;
    detected = msg->detected;
  }
};

// Combine states
struct State
{
  Cam cam;
  StopLine stop_line;
  traffic::Sign sign;
  traffic::Light light;
};

}  // namespace sensor

#endif
