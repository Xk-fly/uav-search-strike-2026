/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning Control 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <mutex>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void velocityOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void initializeTarget(const Position3D& pos);
    void beginSettling();
    bool isStoppedAndSettled();
    bool settleTimedOut() const;

    Position3D target_pos_;
    double tolerance;
    double offset_z;
    bool control_yaw;
    double initial_yaw_{0.0};
    bool target_initialized{false};

    // Optional gates; existing PlanningControl calls keep their original behavior.
    bool wait_before_start_{false};
    bool wait_for_stop_{false};
    bool arrival_hold_active_{false};
    geometry_msgs::Point entry_hold_pos_;
    double stop_speed_tolerance_{0.12};
    double stop_settle_time_{0.5};
    double stop_timeout_{10.0};
    ros::Time settle_started_at_;
    ros::Time velocity_stamp_;
    ros::Time velocity_receipt_time_;
    ros::Time velocity_stable_since_;
    std::mutex velocity_mutex_;
    std::mutex planner_mutex_;

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    ros::Subscriber velocity_odom_sub_;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd;
};

#endif // PLANNING_CONTROL_H
