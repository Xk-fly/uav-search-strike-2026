#ifndef STABLE_HOVER_H
#define STABLE_HOVER_H

#include "base/drone_action_node.h"
#include <ros/time.h>

// 打击后安全交接：只根据里程计位置差分估速，避免依赖可能恒零的 twist.linear。
// 高速先下发零速度指令制动；低速才捕获定点悬停；持续稳定达标后SUCCESS。
class StableHover : public DroneActionNode {
public:
    StableHover(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;

private:
    enum class Phase { BRAKE, HOLD };
    Phase phase_{Phase::BRAKE};
    double speed_threshold_{0.16};
    double settle_time_{0.90};
    double timeout_{9.0};
    double sample_interval_{0.20};
    double max_hold_drift_{0.30};
    ros::Time start_time_, sample_time_, stable_since_;
    ros::Time last_odom_stamp_;
    geometry_msgs::Point sample_pos_, hold_pos_;
    double hold_yaw_{0.0};
    double speed_estimate_{0.0};
    bool speed_ready_{false};

    bool odomOk(const ros::Time& now) const;
    void zeroVelocity();
    NodeStatus stopWithFailure(const char* reason);
};
#endif
