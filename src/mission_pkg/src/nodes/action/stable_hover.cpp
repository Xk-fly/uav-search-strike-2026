#include "nodes/action/stable_hover.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>

namespace {
double d3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::hypot(std::hypot(a.x-b.x, a.y-b.y), a.z-b.z);
}
bool finitePos(const geometry_msgs::Point& p) {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
}
}

StableHover::StableHover(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList StableHover::providedPorts() {
    return {
        InputPort<double>("speed_threshold", 0.16, "实测速度阈值(m/s)"),
        InputPort<double>("settle_time", 0.90, "需持续稳定的时间(s)"),
        InputPort<double>("timeout", 9.0, "最大制动+悬停等待(s)")
    };
}

bool StableHover::odomOk(const ros::Time& now) const {
    if (!fcu_odom_ptr || fcu_odom_ptr->header.stamp.isZero()) return false;
    if (!finitePos(fcu_odom_ptr->pose.pose.position)) return false;
    const double age=(now - fcu_odom_ptr->header.stamp).toSec();
    return std::isfinite(age)&&age>=-0.3&&age<=1.0;
}
void StableHover::zeroVelocity() {
    // 速度为零时机体系/世界系一致，使用现有PX4 OFFBOARD速度闭环做减速。
    publishVelocityWithYawRate(0.0,0.0,0.0,0.0);
}
NodeStatus StableHover::stopWithFailure(const char* reason) {
    ROS_ERROR("[%s] StableHover FAILURE %s, HOLD current position; no Ego handoff",
              name().c_str(), reason);
    sendHoverCommand();
    return NodeStatus::FAILURE;
}

NodeStatus StableHover::onStart() {
    speed_threshold_=getInput<double>("speed_threshold").value();
    settle_time_=getInput<double>("settle_time").value();
    timeout_=getInput<double>("timeout").value();
    if (!(std::isfinite(speed_threshold_) && speed_threshold_>0 &&
          std::isfinite(settle_time_) && settle_time_>0 &&
          std::isfinite(timeout_) && timeout_>settle_time_))
        return stopWithFailure("invalid parameters");

    const ros::Time now=ros::Time::now();
    if (!odomOk(now)) return stopWithFailure("odom unavailable/stale");
    start_time_=sample_time_=now;
    last_odom_stamp_=fcu_odom_ptr->header.stamp;
    sample_pos_=fcu_odom_ptr->pose.pose.position;
    stable_since_=ros::Time(0);
    speed_ready_=false;
    speed_estimate_=0;
    phase_=Phase::BRAKE;
    zeroVelocity();
    ROS_INFO("[%s] StableHover BEGIN (speed <= %.2f m/s for %.2fs; timeout %.1fs)",
             name().c_str(),speed_threshold_,settle_time_,timeout_);
    return NodeStatus::RUNNING;
}

NodeStatus StableHover::onRunning() {
    const ros::Time now=ros::Time::now();
    if (!odomOk(now)) return stopWithFailure("odom stale/non-finite");
    if ((now-start_time_).toSec()>timeout_) return stopWithFailure("braking timeout");

    // 先看当前里程计是否有新数据，再判断缓存采样是否超时。
    // 旧版先用last_odom_stamp_作超时门槛并直接return；
    // 如曾经超过0.4s，后来新数据即使恢复也永远无法更新last_odom_stamp_。
    const ros::Time current_odom_stamp=fcu_odom_ptr->header.stamp;
    const bool has_new_odom=current_odom_stamp>last_odom_stamp_;
    const double sample_age=(now-last_odom_stamp_).toSec();
    if (!has_new_odom && sample_age>0.40) {
        speed_ready_=false;
        stable_since_=ros::Time(0);
        phase_=Phase::BRAKE;
        zeroVelocity();
        ROS_WARN_THROTTLE(1.0,"[%s] StableHover waiting fresh odom: sample_age=%.2fs latest_header_age=%.2fs",
                          name().c_str(),sample_age,(now-current_odom_stamp).toSec());
        return NodeStatus::RUNNING;
    }
    const auto pos=fcu_odom_ptr->pose.pose.position;
    const double dt=(now-sample_time_).toSec();
    if (dt>=sample_interval_ && has_new_odom) {
        speed_estimate_=d3(pos,sample_pos_)/dt;
        sample_pos_=pos;
        sample_time_=now;
        last_odom_stamp_=current_odom_stamp;
        speed_ready_=true;
    }

    if (phase_==Phase::BRAKE) {
        // 不捕获可能很远的旧位置目标；先用零速度闭环抑制惯性。
        zeroVelocity();
        if (speed_ready_ && speed_estimate_<=speed_threshold_) {
            phase_=Phase::HOLD;
            hold_pos_=pos; // 在已经减速的位置捕获悬停点
            hold_yaw_=PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
            if (!std::isfinite(hold_yaw_))return stopWithFailure("invalid yaw");
            stable_since_=now;
            ROS_INFO("[%s] StableHover BRAKE->HOLD at (%.2f,%.2f,%.2f), speed=%.3f",
                     name().c_str(),pos.x,pos.y,pos.z,speed_estimate_);
        }
    } else {
        publishPosition(hold_pos_,hold_yaw_);
        if (!speed_ready_ || speed_estimate_>speed_threshold_ ||
            d3(pos,hold_pos_)>max_hold_drift_) {
            phase_=Phase::BRAKE;
            stable_since_=ros::Time(0);
            ROS_WARN("[%s] StableHover unsettled speed=%.3f drift=%.3f, rebrake",
                     name().c_str(),speed_estimate_,d3(pos,hold_pos_));
        } else if ((now-stable_since_).toSec()>=settle_time_) {
            ROS_INFO("[%s] StableHover SUCCESS speed=%.3f stable %.2fs; Ego can take over",
                     name().c_str(),speed_estimate_,settle_time_);
            publishPosition(hold_pos_,hold_yaw_);
            return NodeStatus::SUCCESS;
        }
    }
    ROS_INFO_THROTTLE(0.8,"[%s] StableHover phase=%s speed_est=%.3f measured=%d elapsed=%.1f",
                      name().c_str(),phase_==Phase::BRAKE?"BRAKE":"HOLD",
                      speed_estimate_,static_cast<int>(speed_ready_),(now-start_time_).toSec());
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<StableHover>("StableHover");
}
