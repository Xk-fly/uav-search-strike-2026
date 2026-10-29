/******************************************************************************
 * @file       planning_control.cpp
 * @brief      Planning Control 行为树节点 - 规划器控制
 *
 * @details    与外部路径规划器交互，执行复杂导航任务
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/planning_control.h"
#include "utils/pose_utils.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PlanningControl::PlanningControl(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_planner_cmd(false) {
    
    auto nh = mct->getROSHandle();
    std::string goal_topic, cmd_topic;
    nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh->param<std::string>("planner_cmd_topic", cmd_topic, "/planning/pos_cmd");

    planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    planner_cmd_sub = nh->subscribe(cmd_topic, 1, &PlanningControl::plannerCmdCallback, this);
    std::string velocity_odom_topic;
    nh->param<std::string>("settle_velocity_odom_topic", velocity_odom_topic,
                           "/mavros/local_position/odom");
    velocity_odom_sub_ = nh->subscribe(velocity_odom_topic, 10,
                                      &PlanningControl::velocityOdomCallback, this);
}

PortsList PlanningControl::providedPorts() {
    return { InputPort<Position3D>("goal", "目标点(x;y;z)"),
             InputPort<double>("offset_z", 0.0, "相对于目标的Z轴偏移(随飞机航向)"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<bool>("control_yaw", false, "是否控制偏航角"),
             InputPort<bool>("wait_before_start", false, "发规划目标前等待实际停稳"),
             InputPort<bool>("stop_only", false, "仅制动并等待停稳，不发送规划目标"),
             InputPort<bool>("brake_with_velocity", false, "气球撞击后先用零速度制动，停稳再锁定位置"),
             InputPort<bool>("require_goal_match", false, "仅执行当前目标对应的规划指令"),
             InputPort<bool>("wait_for_stop", false, "到达后等待实际停稳"),
             InputPort<double>("stop_speed_tolerance", 0.12, "停稳判据(m/s)，不是飞行限速"),
             InputPort<double>("stop_settle_time", 0.5, "连续停稳时间(s)"),
             InputPort<double>("stop_timeout", 10.0, "停稳超时(s)"),
             OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)") };
}

void PlanningControl::plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    last_planner_cmd = *msg;
    has_planner_cmd = true;
}

void PlanningControl::velocityOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    const auto& v = msg->twist.twist.linear;
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    const ros::Time now = ros::Time::now();
    const ros::Time stamp = msg->header.stamp;
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    // Norm is invariant under the MAVROS/world rotation. Never mix their positions.
    if (!std::isfinite(speed) || stamp.isZero() || (stamp - now).toSec() > 0.1 ||
        (now - stamp).toSec() > 0.5 || stamp < velocity_stamp_) {
        velocity_stable_since_ = ros::Time(0);
        velocity_stamp_ = ros::Time(0);
        return;
    }
    if (stamp == velocity_stamp_) return;
    // MAVROS stamps can lead receipt time by a few milliseconds; measure dwell locally.
    if ((now - velocity_receipt_time_).toSec() > 0.5 || speed > stop_speed_tolerance_) {
        velocity_stable_since_ = ros::Time(0);
    }
    velocity_stamp_ = stamp;
    velocity_receipt_time_ = now;
    if (speed <= stop_speed_tolerance_ && velocity_stable_since_.isZero()) {
        velocity_stable_since_ = now;
    }
}

void PlanningControl::beginSettling() {
    settle_started_at_ = ros::Time::now();
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    velocity_stable_since_ = ros::Time(0);
}

bool PlanningControl::isStoppedAndSettled() {
    const ros::Time now = ros::Time::now();
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    if (velocity_stamp_.isZero() || velocity_receipt_time_ > now ||
        (now - velocity_receipt_time_).toSec() > 0.5) {
        velocity_stable_since_ = ros::Time(0);
        ROS_WARN_THROTTLE(2.0, "[%s]: Waiting for fresh MAVROS velocity odometry.", name().c_str());
        return false;
    }
    return !velocity_stable_since_.isZero() &&
           (velocity_receipt_time_ - velocity_stable_since_).toSec() >= stop_settle_time_;
}

bool PlanningControl::settleTimedOut() const {
    if ((ros::Time::now() - settle_started_at_).toSec() <= stop_timeout_) return false;
    ROS_ERROR("[%s]: Stop gate timed out after %.1f s; no handoff.",
              name().c_str(), stop_timeout_);
    return true;
}

void PlanningControl::initializeTarget(const Position3D& pos) {
    target_pos_ = pos;

    // 计算带有偏移的目标点 (沿着飞机当前位姿的机头方向投射)
    tf2::Quaternion q_curr;
    tf2::fromMsg(fcu_odom_ptr->pose.pose.orientation, q_curr);
    double r, p, yaw;
    tf2::Matrix3x3(q_curr).getRPY(r, p, yaw);

    double target_x = target_pos_.x + offset_z * cos(yaw);
    double target_y = target_pos_.y + offset_z * sin(yaw);
    double target_z = target_pos_.z;

    // 向规划器发送目标（都使用 ENU，直接发送）
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    ps.pose.position.x = target_x;
    ps.pose.position.y = target_y;
    ps.pose.position.z = target_z;
    
    tf2::Quaternion q_tgt;
    q_tgt.setRPY(0, 0, 0); 
    ps.pose.orientation = tf2::toMsg(q_tgt);

    planner_goal_pub.publish(ps);
    {
        std::lock_guard<std::mutex> lock(planner_mutex_);
        has_planner_cmd = false;
    }

    ROS_ERROR("[%s]: >>> NED goal to ego=(%.2f, %.2f, %.2f), yaw=%.2f, offset=%.2f <<<",
             name().c_str(), ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, yaw, offset_z);

    // 为了 checkArrival 判断，更新目标点
    target_pos_.x = target_x;
    target_pos_.y = target_y;

    target_initialized = true;
    ROS_INFO("[%s]: Target acquired! Planning navigation start -> (%.2f, %.2f, %.2f), offset: %.2f", 
             name().c_str(), target_x, target_y, target_z, offset_z);
}

NodeStatus PlanningControl::onStart() {
    auto off_res = getInput<double>("offset_z");
    offset_z = off_res.value();

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }

    auto yaw_res = getInput<bool>("control_yaw");
    control_yaw = yaw_res.value();
    wait_before_start_ = getInput<bool>("wait_before_start").value();
    stop_only_ = getInput<bool>("stop_only").value();
    brake_with_velocity_ = stop_only_ && getInput<bool>("brake_with_velocity").value();
    require_goal_match_ = getInput<bool>("require_goal_match").value();
    if (stop_only_) wait_before_start_ = true;
    wait_for_stop_ = getInput<bool>("wait_for_stop").value();
    const double speed_tolerance = getInput<double>("stop_speed_tolerance").value();
    stop_settle_time_ = getInput<double>("stop_settle_time").value();
    stop_timeout_ = getInput<double>("stop_timeout").value();
    if (!std::isfinite(speed_tolerance) || speed_tolerance < 0.0 ||
        !std::isfinite(stop_settle_time_) || stop_settle_time_ <= 0.0 ||
        !std::isfinite(stop_timeout_) || stop_timeout_ <= stop_settle_time_) {
        ROS_ERROR("[%s]: Invalid stop gate parameters.", name().c_str());
        sendHoverCommand();
        return NodeStatus::FAILURE;
    }
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        stop_speed_tolerance_ = speed_tolerance;
    }
    target_initialized = false;
    arrival_hold_active_ = false;

    ROS_INFO("[%s]: Initialized with offset_z=%.2f, tolerance=%.2f, control_yaw=%d",
             name().c_str(), offset_z, tolerance, control_yaw);

    // 捕获初始偏航角 (供 !control_yaw 使用)
    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    entry_hold_pos_ = fcu_odom_ptr->pose.pose.position;
    if (wait_before_start_) {
        beginSettling();
        if (brake_with_velocity_) publishVelocityWithYawRate(0, 0, 0, 0);
        else publishPosition(entry_hold_pos_, initial_yaw_);
        ROS_INFO("[%s]: %s; waiting for actual stop.", name().c_str(),
                 brake_with_velocity_ ? "BRAKE: zero velocity before position hold" :
                 (stop_only_ ? "BRAKE: holding fixed position" : "Holding before Ego takeover"));
        return NodeStatus::RUNNING;
    }

    
    // 尝试获取目标点
    auto goal_res = getInput<Position3D>("goal");
    if (!goal_res) {
        ROS_INFO("[%s]: Waiting for goal...", name().c_str());
        target_initialized = false;
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }

    // 初始化目标
    initializeTarget(goal_res.value());
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::onRunning() {
    if (wait_before_start_) {
        if (brake_with_velocity_) publishVelocityWithYawRate(0, 0, 0, 0);
        else publishPosition(entry_hold_pos_, initial_yaw_);
        if (settleTimedOut()) return NodeStatus::FAILURE;
        if (!isStoppedAndSettled()) return NodeStatus::RUNNING;
        wait_before_start_ = false;
        if (stop_only_) {
            if (brake_with_velocity_) {
                entry_hold_pos_ = fcu_odom_ptr->pose.pose.position;
                initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                publishPosition(entry_hold_pos_, initial_yaw_);
            }
            ROS_INFO("[%s]: BRAKE complete: actual stop verified.", name().c_str());
            return NodeStatus::SUCCESS;
        }
        ROS_INFO("[%s]: Actual stop verified; enabling Ego navigation.", name().c_str());
    }
    // 如果目标还未初始化，尝试获取
    if (!target_initialized) {
        auto goal_res = getInput<Position3D>("goal");
        if (!goal_res) {
            sendHoverCommand();
            return NodeStatus::RUNNING;
        }
        
        initializeTarget(goal_res.value());
        ROS_INFO("[%s]: Target acquired!", name().c_str());
    }

    // 检查是否到达 (3D 距离)
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    const bool arrived = checkArrival(target, tolerance);
    if (arrival_hold_active_) {
        // Hold the original planned endpoint instead of capturing a drifting position.
        publishPosition(target, initial_yaw_);
        if (settleTimedOut()) return NodeStatus::FAILURE;
        if (!arrived) {
            std::lock_guard<std::mutex> lock(velocity_mutex_);
            velocity_stable_since_ = ros::Time(0);
            return NodeStatus::RUNNING;
        }
        if (!isStoppedAndSettled()) return NodeStatus::RUNNING;
    } else if (arrived && wait_for_stop_) {
        arrival_hold_active_ = true;
        beginSettling();
        publishPosition(target, initial_yaw_);
        ROS_INFO("[%s]: Holding endpoint until speed stays below %.2f m/s for %.2f s.",
                 name().c_str(), stop_speed_tolerance_, stop_settle_time_);
        return NodeStatus::RUNNING;
    }

    if (arrived) {
        ROS_INFO("[%s]: Planning navigation completed.", name().c_str());
        
        // 输出最终到达的位置
        Position3D final_pos;
        final_pos.x = fcu_odom_ptr->pose.pose.position.x;
        final_pos.y = fcu_odom_ptr->pose.pose.position.y;
        final_pos.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_pos);
        
        ROS_INFO("[%s]: Final position: (%.2f, %.2f, %.2f)", 
                 name().c_str(), final_pos.x, final_pos.y, final_pos.z);
        
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    quadrotor_msgs::PositionCommand planner_cmd;
    {
        std::lock_guard<std::mutex> lock(planner_mutex_);
        if (!has_planner_cmd) return NodeStatus::RUNNING;
        planner_cmd = last_planner_cmd;
    }
    // traj_server 会继续发布上一段轨迹；返航须等待新目标的轨迹再接管。
    if (require_goal_match_ &&
        (std::abs(planner_cmd.goal_pos.x - target_pos_.x) > 0.05 ||
         std::abs(planner_cmd.goal_pos.y - target_pos_.y) > 0.05 ||
         std::abs(planner_cmd.goal_pos.z - target_pos_.z) > 0.05)) {
        publishPosition(entry_hold_pos_, initial_yaw_);
        ROS_INFO_THROTTLE(2.0, "[%s]: Waiting for trajectory for the current goal.", name().c_str());
        return NodeStatus::RUNNING;
    }

    // Ego-planner 输出经过 publishPosition 做 ENU→NED 转换
    geometry_msgs::Point ego_pos = planner_cmd.position;
    double ego_yaw = control_yaw ? planner_cmd.yaw : initial_yaw_;
    
    publishPosition(ego_pos, ego_yaw);
    
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
