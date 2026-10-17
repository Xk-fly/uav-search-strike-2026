#ifndef BALLOON_OBSERVE_H
#define BALLOON_OBSERVE_H
#include "base/drone_action_node.h"
#include <common_msgs/DetEnable.h>
#include <common_msgs/Objects.h>
#include <std_msgs/String.h>
#include <mutex>
#include <string>

class BalloonObserve : public DroneActionNode {
public:
  BalloonObserve(const std::string& name, const NodeConfig& config);
  static PortsList providedPorts();
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;
private:
  void objectsCallback(const common_msgs::Objects::ConstPtr& msg);
  void status(const std::string& tag);
  void stopDown();
  ros::Subscriber sub_;
  ros::Publisher down_control_pub_, status_pub_;
  std::mutex mutex_;
  common_msgs::Objects objects_;
  unsigned long rx_seq_{0}, consumed_seq_{0};
  std::string mode_, target_;
  geometry_msgs::Point hover_;
  double starting_yaw_{0};
  double confidence_{0.75}, yaw_rate_{0.30}, timeout_{26.0}, max_drift_{0.5};
  int hits_{0};
  int observed_frames_{0};
  bool confirmed_seen_{false};
  bool observe_full_duration_{false};
  ros::Publisher postcheck_pub_;
  bool down_on_{false};
  ros::Time start_, last_hit_;
};
#endif
