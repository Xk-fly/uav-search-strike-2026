/******************************************************************************
 * @file       precision_move.h
 * @brief      平滑定点运动行为树节点：ArUco 往返专用
 *
 * 仅生成受限速度/加速度的位置参考点，不修改 PX4 内部 PID。
 ******************************************************************************/
#ifndef PRECISION_MOVE_H
#define PRECISION_MOVE_H

#include "base/drone_action_node.h"
#include <ros/time.h>

class PrecisionMove : public DroneActionNode {
public:
    PrecisionMove(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;

private:
    enum class Stage { ACCELERATE, CRUISE, DECELERATE, SETTLE };
    Stage stage_{Stage::ACCELERATE};

    geometry_msgs::Point start_pos_;
    geometry_msgs::Point target_pos_;
    geometry_msgs::Point dir_;
    double total_distance_{0.0};
    double reference_distance_{0.0};
    double reference_speed_{0.0};

    double max_speed_{0.35};
    double max_accel_{0.25};
    double max_lead_{0.25};
    double tolerance_{0.12};
    double speed_tolerance_{0.10};
    double settle_time_{0.70};
    double timeout_{60.0};
    double odom_timeout_{1.0};
    double abort_speed_{0.70};
    double yaw_rate_limit_{0.50};
    double yaw_tolerance_{0.15};

    bool check_yaw_{false};
    double start_yaw_{0.0};
    double target_yaw_{0.0};
    double commanded_yaw_{0.0};
    ros::Time started_at_;
    ros::Time last_tick_;
    ros::Time stable_since_;

    bool odomValid() const;
    double actualSpeed() const;
    void updateYaw(double desired_yaw, double dt);
    void setStage(Stage next);
    NodeStatus failSafe(const std::string& reason);
};

#endif  // PRECISION_MOVE_H
