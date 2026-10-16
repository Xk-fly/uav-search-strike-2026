/******************************************************************************
 * @file       precision_move.cpp
 * @brief      受限位置参考轨迹 + 稳定到点判定 (不用额外 PID)
 *
 * 适用：本轮 ArUco 去程、返回穿框准备/过框后短行程。
 * 注意：沿起点与目标点的直线运动，不是障碍物规划器。
 ******************************************************************************/
#include "nodes/action/precision_move.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinLength = 1e-5;
constexpr double kFinishReferenceDistance = 0.015;  // 轨迹参考点末端吸附 (m)
bool finitePoint(const geometry_msgs::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
double dist3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
const char* stageName(int stage) {
    switch (stage) {
        case 0: return "ACCELERATE";
        case 1: return "CRUISE";
        case 2: return "DECELERATE";
        case 3: return "SETTLE";
        default: return "UNKNOWN";
    }
}
}  // namespace

PrecisionMove::PrecisionMove(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList PrecisionMove::providedPorts() {
    return {
        InputPort<Position3D>("goal", "可选终点坐标 x;y;z"),
        InputPort<double>("height", "可选世界绝对高度(m)，保持当前XY"),
        InputPort<double>("yaw", "可选终点绝对偏航(rad)"),
        InputPort<int>("frame", 0, "0=world ENU, 1=相对起始机体坐标")
    };
}

bool PrecisionMove::odomValid() const {
    if (!fcu_odom_ptr || fcu_odom_ptr->header.stamp.isZero()) return false;
    const auto& p = fcu_odom_ptr->pose.pose.position;
    const auto& q = fcu_odom_ptr->pose.pose.orientation;
    const auto& v = fcu_odom_ptr->twist.twist.linear;
    if (!finitePoint(p)) return false;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
    if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
        !std::isfinite(v.z)) return false;
    const double age = (ros::Time::now() - fcu_odom_ptr->header.stamp).toSec();
    return std::isfinite(age) && age >= -0.5 && age <= odom_timeout_;
}

double PrecisionMove::actualSpeed() const {
    const auto& v = fcu_odom_ptr->twist.twist.linear;
    // 速度范数与 twist 所在坐标架朝向无关，无需假设 body/world。
    return std::hypot(std::hypot(v.x, v.y), v.z);
}

void PrecisionMove::updateYaw(double desired_yaw, double dt) {
    const double diff = std::remainder(desired_yaw - commanded_yaw_, 2.0 * kPi);
    const double max_step = yaw_rate_limit_ * dt;
    commanded_yaw_ = std::remainder(
        commanded_yaw_ + std::clamp(diff, -max_step, max_step), 2.0 * kPi);
}

void PrecisionMove::setStage(Stage next) {
    if (stage_ == next) return;
    stage_ = next;
    ROS_INFO("[%s] PrecisionMove stage -> %s",
             name().c_str(), stageName(static_cast<int>(stage_)));
}

NodeStatus PrecisionMove::failSafe(const std::string& reason) {
    ROS_ERROR("[%s] PrecisionMove FAILURE: %s; commanding hover",
              name().c_str(), reason.c_str());
    sendHoverCommand();
    return NodeStatus::FAILURE;
}

NodeStatus PrecisionMove::onStart() {
    // 与原 GoTo 一样使用 FCU/RflySim odom ENU，但额外检查其有效性。
    auto nh = mct->getROSHandle();
    nh->param("precision_move/max_speed", max_speed_, 0.35);
    nh->param("precision_move/max_accel", max_accel_, 0.25);
    nh->param("precision_move/max_lead", max_lead_, 0.25);
    nh->param("precision_move/tolerance", tolerance_, 0.12);
    nh->param("precision_move/speed_tolerance", speed_tolerance_, 0.10);
    nh->param("precision_move/settle_time", settle_time_, 0.70);
    nh->param("precision_move/timeout", timeout_, 60.0);
    nh->param("precision_move/odom_timeout", odom_timeout_, 1.0);
    nh->param("precision_move/abort_speed", abort_speed_, 0.70);
    nh->param("precision_move/yaw_rate", yaw_rate_limit_, 0.50);
    nh->param("yaw_tolerance", yaw_tolerance_, 0.15);

    if (!(std::isfinite(max_speed_) && max_speed_ > 0 &&
          std::isfinite(max_accel_) && max_accel_ > 0 &&
          std::isfinite(max_lead_) && max_lead_ > 0 &&
          std::isfinite(tolerance_) && tolerance_ > 0 &&
          std::isfinite(speed_tolerance_) && speed_tolerance_ > 0 &&
          std::isfinite(settle_time_) && settle_time_ >= 0 &&
          std::isfinite(timeout_) && timeout_ > 0 &&
          std::isfinite(odom_timeout_) && odom_timeout_ > 0 &&
          std::isfinite(abort_speed_) && abort_speed_ > max_speed_ &&
          std::isfinite(yaw_rate_limit_) && yaw_rate_limit_ > 0 &&
          std::isfinite(yaw_tolerance_) && yaw_tolerance_ > 0)) {
        return failSafe("invalid precision_move params");
    }
    if (!odomValid()) return failSafe("odom missing, stale or invalid");

    auto goal_res = getInput<Position3D>("goal");
    auto height_res = getInput<double>("height");
    auto yaw_res = getInput<double>("yaw");
    if (goal_res && height_res) return failSafe("goal and height cannot coexist");
    if (!goal_res && !height_res && !yaw_res) return failSafe("missing goal/height/yaw");
    auto frame_res = getInput<int>("frame");
    const int frame = frame_res.value();
    if (frame != 0 && frame != 1) return failSafe("frame must be 0 or 1");

    start_pos_ = fcu_odom_ptr->pose.pose.position;
    start_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    if (!std::isfinite(start_yaw_)) return failSafe("invalid starting yaw");

    // yaw-only：完全保持初始XYZ；height-only：只改变Z，XY不变。
    target_pos_ = start_pos_;
    if (goal_res) {
        const Position3D goal = goal_res.value();
        if (!(std::isfinite(goal.x) && std::isfinite(goal.y) && std::isfinite(goal.z)))
            return failSafe("non-finite goal");
        if (frame == 1) {
            target_pos_.x = start_pos_.x + goal.x * std::cos(start_yaw_) - goal.y * std::sin(start_yaw_);
            target_pos_.y = start_pos_.y + goal.x * std::sin(start_yaw_) + goal.y * std::cos(start_yaw_);
            target_pos_.z = start_pos_.z + goal.z;
        } else {
            target_pos_.x = goal.x;
            target_pos_.y = goal.y;
            target_pos_.z = goal.z;
        }
    } else if (height_res) {
        if (frame != 0) return failSafe("height requires world frame");
        target_pos_.z = height_res.value();
    }


    check_yaw_ = static_cast<bool>(yaw_res);
    target_yaw_ = check_yaw_ ? yaw_res.value() : start_yaw_;
    if (!finitePoint(target_pos_) || !std::isfinite(target_yaw_))
        return failSafe("invalid transformed target/yaw");

    total_distance_ = dist3(start_pos_, target_pos_);
    dir_.x = dir_.y = dir_.z = 0.0;
    if (total_distance_ > kMinLength) {
        dir_.x = (target_pos_.x - start_pos_.x) / total_distance_;
        dir_.y = (target_pos_.y - start_pos_.y) / total_distance_;
        dir_.z = (target_pos_.z - start_pos_.z) / total_distance_;
    }
    reference_distance_ = 0.0;
    reference_speed_ = 0.0;
    commanded_yaw_ = start_yaw_;
    started_at_ = last_tick_ = ros::Time::now();
    stable_since_ = ros::Time(0);
    stage_ = (total_distance_ <= kMinLength) ? Stage::SETTLE : Stage::ACCELERATE;

    ROS_INFO("[%s] PrecisionMove BEGIN start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) "
             "length=%.2f frame=%d vmax=%.2f amax=%.2f lead=%.2f",
             name().c_str(), start_pos_.x, start_pos_.y, start_pos_.z,
             target_pos_.x, target_pos_.y, target_pos_.z,
             total_distance_, frame, max_speed_, max_accel_, max_lead_);
    return NodeStatus::RUNNING;
}

NodeStatus PrecisionMove::onRunning() {
    const ros::Time now = ros::Time::now();
    if ((now - started_at_).toSec() > timeout_)
        return failSafe("segment timeout");
    if (!odomValid())
        return failSafe("odom timeout / invalid data");

    // 控制循环实际间隔，避免调度停顿后参考点突然前跳。
    const double dt = std::clamp((now - last_tick_).toSec(), 0.0, 0.10);
    last_tick_ = now;

    const geometry_msgs::Point actual = fcu_odom_ptr->pose.pose.position;
    const double actual_speed = actualSpeed();
    if (actual_speed > abort_speed_)
        return failSafe("measured speed exceeds abort_speed");

    geometry_msgs::Point ref = target_pos_;
    if (stage_ != Stage::SETTLE) {
        const double actual_projection = std::clamp(
            (actual.x - start_pos_.x) * dir_.x +
            (actual.y - start_pos_.y) * dir_.y +
            (actual.z - start_pos_.z) * dir_.z, 0.0, total_distance_);

        // 基于剩余参考行程计算制动速度：v <= sqrt(2*a*remaining)。
        const double remaining_ref = std::max(0.0, total_distance_ - reference_distance_);
        const double desired_speed = std::min(
            max_speed_, std::sqrt(2.0 * max_accel_ * remaining_ref));
        reference_speed_ += std::clamp(
            desired_speed - reference_speed_, -max_accel_ * dt, max_accel_ * dt);

        // 控制设定点推进速度 + 跟踪超前量，避免远处终点直接吸引 PX4 加速。
        reference_distance_ = std::min({
            total_distance_,
            reference_distance_ + reference_speed_ * dt,
            actual_projection + max_lead_
        });

        if (total_distance_ - reference_distance_ <= kFinishReferenceDistance) {
            reference_distance_ = total_distance_;
            reference_speed_ = 0.0;
            setStage(Stage::SETTLE);
            stable_since_ = ros::Time(0);
        } else if (desired_speed < max_speed_ - 0.02 &&
                   reference_speed_ > desired_speed) {
            setStage(Stage::DECELERATE);
        } else if (reference_speed_ >= max_speed_ - 0.02) {
            setStage(Stage::CRUISE);
        } else {
            setStage(Stage::ACCELERATE);
        }

        ref.x = start_pos_.x + reference_distance_ * dir_.x;
        ref.y = start_pos_.y + reference_distance_ * dir_.y;
        ref.z = start_pos_.z + reference_distance_ * dir_.z;
    }

    // 位移过程中保持进入此动作时的航向；稳定到点后再平滑完成显式转向。
    updateYaw((stage_ == Stage::SETTLE) ? target_yaw_ : start_yaw_, dt);
    publishPosition(ref, commanded_yaw_);

    const double error = dist3(actual, target_pos_);
    const double actual_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    const double yaw_error = std::abs(std::remainder(target_yaw_ - actual_yaw, 2.0 * kPi));
    const bool ready = stage_ == Stage::SETTLE &&
                       error <= tolerance_ && actual_speed <= speed_tolerance_ &&
                       std::isfinite(actual_yaw) &&
                       (!check_yaw_ || yaw_error <= yaw_tolerance_);

    if (ready) {
        if (stable_since_.isZero()) stable_since_ = now;
        if ((now - stable_since_).toSec() >= settle_time_) {
            ROS_INFO("[%s] PrecisionMove SUCCESS dist=%.3f speed=%.3f stable=%.2fs",
                     name().c_str(), error, actual_speed, settle_time_);
            sendHoverCommand();
            return NodeStatus::SUCCESS;
        }
    } else {
        stable_since_ = ros::Time(0);
    }

    ROS_INFO_THROTTLE(1.0,
        "[%s] PrecisionMove %s ref=%.2f/%.2f goal_err=%.3f speed=%.3f yaw_err=%.3f",
        name().c_str(), stageName(static_cast<int>(stage_)),
        reference_distance_, total_distance_, error, actual_speed, yaw_error);
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PrecisionMove>("PrecisionMove");
}
