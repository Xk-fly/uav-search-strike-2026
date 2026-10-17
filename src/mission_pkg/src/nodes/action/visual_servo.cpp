/******************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 *
 * @details    三阶段：对齐(ALIGN) -> 靠近(APPROACH) -> 冲刺(SPRINT)
 *             全程使用 publishVelocityWithYawRate（body 坐标系）
 *             控制变量：VX(前进) + VZ(升降) + yaw_rate(偏航角速度)
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
#include <algorithm>
#include <std_msgs/Bool.h>
#include "utils/pose_utils.h"
#include <cmath>


VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), 
      current_state_(ALIGN),
      has_ever_received_data_(false),
      max_area_ratio_(0.0) {
    auto nh = mct->getROSHandle();
    nh->param<double>("cx", img_cx_, 320.0);
    nh->param<double>("cy", img_cy_, 240.0);
    nh->param<int>("img_w", img_w_, 640);
    nh->param<int>("img_h", img_h_, 480);
    visible_pub_=nh->advertise<std_msgs::Bool>("/balloon_attack/target_visible",10);
    hit_pub_=nh->advertise<std_msgs::Bool>("/balloon_attack/hit_candidate",10,true);
}

PortsList VisualServo::providedPorts() {
    return { InputPort<PixelTarget>("pixel_target", "像素目标"),
             
             // yaw_rate PD 控制
             InputPort<double>("k_p_yaw_rate", 0.003, "偏航角速度 P 增益 (rad/s per pixel)"),
             InputPort<double>("k_d_yaw_rate", 0.008, "偏航角速度 D 增益"),
             InputPort<double>("max_yaw_rate", 0.5, "最大偏航角速度 (rad/s)"),
             InputPort<double>("deadzone_yaw", 8.0, "偏航死区 (pixel)"),
             
             // VZ PD 控制
             InputPort<double>("k_p_vz", 0.002, "垂直速度 P 增益"),
             InputPort<double>("k_d_vz", 0.006, "垂直速度 D 增益"),
             InputPort<double>("max_vz", 0.3, "最大垂直速度 (m/s)"),
             InputPort<double>("deadzone_vz", 10.0, "垂直死区 (pixel)"),
             
             // 速度参数
             InputPort<double>("forward_velocity", 0.35, "逼近阶段前进速度 (m/s)"),
             InputPort<double>("sprint_velocity", 1.5, "冲刺速度 (m/s)"),
             InputPort<double>("sprint_time", 0.60, "冲刺持续时间 (s)"),
             InputPort<double>("max_sprint_yaw_rate", 0.08, "冲刺最大偏航速率"),
             InputPort<double>("approach_center_limit_px", 48.0, "靠近时允许的最大水平偏差px"),
             InputPort<double>("sprint_center_limit_px", 24.0, "触发冲刺的最大水平偏差px"),
             InputPort<double>("brake_time", 0.50, "冲刺后悬停秒数"),
             InputPort<double>("min_world_z", 0.90, "最低允许绝对高度m"),
             InputPort<double>("max_descent_from_entry", 0.15, "视觉伺服最大下降量m"),
             InputPort<double>("max_vertical_speed", 0.10, "最大升降速度m/s"),
             
             // 其他
             InputPort<double>("data_timeout", 1.0, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.08, "触发冲刺的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             InputPort<double>("search_yaw_rate", 0.3, "搜索模式旋转速度 (rad/s)"),
             InputPort<bool>("lock_yaw_after_first_detection", false, "首次识别后丢失则固定航向悬停"),
             InputPort<bool>("horizontal_only_alignment", false, "A区只需水平对准即可接近"),
             InputPort<int>("align_frames_required", 5, "对准需要不同YOLO检测帧数"),
             InputPort<double>("sprint_vertical_limit_px", 55.0, "冲刺安全纵向误差"),
             InputPort<double>("initial_search_max_yaw_rad", 0.60, "首次见球前相对搜索偏航上限"),
             InputPort<int>("lock_confirmation_frames", 3, "首次锁定需要连续有效帧"),
             InputPort<int>("sprint_confirmation_frames", 5, "冲刺需要连续满足门槛的有效帧"),
             InputPort<double>("min_tracking_after_lock_sec", 1.5, "首次锁定后至少跟踪秒数") };
}

NodeStatus VisualServo::onStart() {
    // 加载参数
    k_p_yaw_rate_ = getInput<double>("k_p_yaw_rate").value();
    k_d_yaw_rate_ = getInput<double>("k_d_yaw_rate").value();
    max_yaw_rate_ = getInput<double>("max_yaw_rate").value();
    deadzone_yaw_ = getInput<double>("deadzone_yaw").value();
    
    k_p_vz_ = getInput<double>("k_p_vz").value();
    k_d_vz_ = getInput<double>("k_d_vz").value();
    max_vz_ = getInput<double>("max_vz").value();
    deadzone_vz_ = getInput<double>("deadzone_vz").value();
    
    forward_velocity_ = getInput<double>("forward_velocity").value();
    sprint_velocity_ = getInput<double>("sprint_velocity").value();
    sprint_time_ = getInput<double>("sprint_time").value();
    max_sprint_yaw_rate_ = getInput<double>("max_sprint_yaw_rate").value();
    approach_center_limit_px_ = getInput<double>("approach_center_limit_px").value();
    sprint_center_limit_px_ = getInput<double>("sprint_center_limit_px").value();
    brake_time_ = getInput<double>("brake_time").value();
    min_world_z_ = getInput<double>("min_world_z").value();
    max_descent_ = getInput<double>("max_descent_from_entry").value();
    max_servo_vz_ = getInput<double>("max_vertical_speed").value();
    if (!std::isfinite(forward_velocity_) || forward_velocity_ <= 0 ||
        !std::isfinite(sprint_velocity_) || sprint_velocity_ <= 0 ||
        !std::isfinite(sprint_time_) || sprint_time_ <= 0 ||
        !std::isfinite(max_sprint_yaw_rate_) || max_sprint_yaw_rate_ <= 0 ||
        !std::isfinite(approach_center_limit_px_) || approach_center_limit_px_ <= 0 ||
        !std::isfinite(sprint_center_limit_px_) || sprint_center_limit_px_ <= 0 ||
        !std::isfinite(brake_time_) || brake_time_ < 0 ||
        !std::isfinite(min_world_z_) || min_world_z_ < 0.30 ||
        !std::isfinite(max_descent_) || max_descent_ < 0.0 ||
        !std::isfinite(max_servo_vz_) || max_servo_vz_ <= 0.0) {
        throw BT::RuntimeError("VisualServo invalid speed/alignment/brake parameters");
    }
    
    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    search_yaw_rate_ = getInput<double>("search_yaw_rate").value();
    lock_yaw_after_first_detection_ = getInput<bool>("lock_yaw_after_first_detection").value();
    horizontal_only_alignment_ = getInput<bool>("horizontal_only_alignment").value();
    align_frames_required_ = getInput<int>("align_frames_required").value();
    sprint_vertical_limit_px_ = getInput<double>("sprint_vertical_limit_px").value();
    initial_search_max_yaw_rad_ = getInput<double>("initial_search_max_yaw_rad").value();
    lock_confirmation_required_ = getInput<int>("lock_confirmation_frames").value();
    sprint_confirmation_required_ = getInput<int>("sprint_confirmation_frames").value();
    min_tracking_after_lock_sec_ = getInput<double>("min_tracking_after_lock_sec").value();
    if (lock_confirmation_required_ < 1 || lock_confirmation_required_ > 15 ||
        sprint_confirmation_required_ < 1 || sprint_confirmation_required_ > 30 ||
        !std::isfinite(min_tracking_after_lock_sec_) ||
        min_tracking_after_lock_sec_ < 0 || min_tracking_after_lock_sec_ > 15 ||
        align_frames_required_ < 1 || align_frames_required_ > 20 ||
        !std::isfinite(sprint_vertical_limit_px_) || sprint_vertical_limit_px_ < 0 ||
        !std::isfinite(initial_search_max_yaw_rad_) ||
        initial_search_max_yaw_rad_ < 0 || initial_search_max_yaw_rad_ > 3.14 ||
        !std::isfinite(search_yaw_rate_) || search_yaw_rate_ < 0) {
        throw BT::RuntimeError("VisualServo invalid reacquisition parameters");
    }
    
    // 初始化状态机
    current_state_ = ALIGN;  // 先对准，再允许前进
    if (!fcu_odom_ptr || fcu_odom_ptr->header.stamp.isZero())
        throw BT::RuntimeError("VisualServo missing odom");
    const auto initial=fcu_odom_ptr->pose.pose.position;
    if (!std::isfinite(initial.x)||!std::isfinite(initial.y)||!std::isfinite(initial.z))
        throw BT::RuntimeError("VisualServo invalid start pose");
    floor_z_=std::max(min_world_z_,initial.z-max_descent_);
    climbing_to_safe_z_=false;
    target_visible_=false; strike_was_visible_=false; explicit_no_target_frame_=false;
    std_msgs::Bool flag;flag.data=false;visible_pub_.publish(flag);hit_pub_.publish(flag);
    ROS_INFO("[%s] VisualServo entry z=%.2f floor z=%.2f",name().c_str(),initial.z,floor_z_);
    if(initial.z<floor_z_-0.02){sendHoverCommand();return NodeStatus::FAILURE;}
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    last_align_detection_stamp_ = 0.0;
    target_seen_once_ = false;
    lock_confirmation_streak_ = sprint_confirmation_streak_ = 0;
    lock_last_detection_stamp_ = sprint_last_detection_stamp_ = 0.0;
    lock_confirmed_at_ = ros::Time(0);
    servo_start_stamp_ = ros::Time::now().toSec();
    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    if (!std::isfinite(initial_yaw_)) {
        sendHoverCommand();
        return NodeStatus::FAILURE;
    }

    // 初始化微分状态
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();
    
    updateTargetData();
    
    ROS_INFO("[%s]: Visual Servo Started, yaw-lock=%d align=%d lock-frames=%d sprint-frames=%d track-time=%.1f",
             name().c_str(), static_cast<int>(lock_yaw_after_first_detection_), align_frames_required_,
             lock_confirmation_required_, sprint_confirmation_required_, min_tracking_after_lock_sec_);
    ROS_INFO("[%s]: k_p_yaw_rate=%.4f k_d=%.4f max_rate=%.2f deadzone=%.1f", 
             name().c_str(), k_p_yaw_rate_, k_d_yaw_rate_, max_yaw_rate_, deadzone_yaw_);
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.1f area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_, approach_area_threshold_);
    
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    updateTargetData();
    const ros::Time now = ros::Time::now();
    const double dt = std::clamp((now - prev_time_).toSec(), 0.02, 0.10);
    prev_time_ = now;

    if (!fcu_odom_ptr || fcu_odom_ptr->header.stamp.isZero() ||
        !std::isfinite(fcu_odom_ptr->pose.pose.position.z)){
        sendHoverCommand();
        return NodeStatus::FAILURE;
    }
    const auto& pose=fcu_odom_ptr->pose.pose;
    const double z=pose.position.z;
    if(climbing_to_safe_z_){
        publishPosition(climb_goal_,climb_yaw_);
        if(z>=floor_z_+0.09){
            climbing_to_safe_z_=false;
            current_state_=ALIGN;
            align_stable_frames_=0;
            prev_err_x_=prev_err_y_=0;
        }else if((now-climb_start_).toSec()>4.0){
            ROS_ERROR("[%s] VisualServo safe altitude recovery timeout",name().c_str());
            sendHoverCommand();return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }
    if(current_state_!=BRAKING && z<=floor_z_+0.04){
        climbing_to_safe_z_=true;climb_start_=now;
        climb_goal_=pose.position;climb_goal_.z=floor_z_+0.12;
        climb_yaw_=PoseUtils::getYawFromQuat(pose.orientation);
        ROS_WARN("[%s] VisualServo floor protection z=%.2f floor=%.2f",
                 name().c_str(),z,floor_z_);
        publishPosition(climb_goal_,climb_yaw_);
        return NodeStatus::RUNNING;
    }

    if (current_state_ == BRAKING) {
        // 悬停前先让速度闭环降速，避免在高速时抓取一个位置设定值。
        publishVelocityWithYawRate(0,0,0,0);
        if ((now - brake_start_time_).toSec() >= brake_time_) {
            ROS_INFO("[%s] VisualServo BRAKE done: strike attempt complete.", name().c_str());
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    // 按用户要求：冲刺开始并看见红球后，第一帧未见该颜色即触发停止。
    if(current_state_==SPRINTING && strike_was_visible_ && explicit_no_target_frame_){
        std_msgs::Bool hit;hit.data=true;hit_pub_.publish(hit);
        ROS_INFO("[%s] HIT_CANDIDATE: balloon vanished during strike, BRAKING now",name().c_str());
        current_state_=BRAKING;brake_start_time_=now;
        publishVelocityWithYawRate(0,0,0,0);
        return NodeStatus::RUNNING;
    }
    // 明确空检测立即制动；若整个感知流中断，data_timeout到期制动。
    // 只有正在冲刺且曾见到目标时的消失才可能是命中候选。
    if (explicit_no_target_frame_ || isTargetLost()) {
        if (current_state_ == SPRINTING) {
            ROS_WARN("[%s] SPRINT stopped after target timeout/no-detection",name().c_str());
            current_state_=BRAKING;brake_start_time_=now;
            publishVelocityWithYawRate(0,0,0,0);
            return NodeStatus::RUNNING;
        }
        current_state_ = ALIGN;
        align_stable_frames_ = 0;
        last_align_detection_stamp_ = 0.0;
        sprint_confirmation_streak_ = 0;
        sprint_last_detection_stamp_ = 0.0;
        if (!target_seen_once_) {
            lock_confirmation_streak_ = 0;
            lock_last_detection_stamp_ = 0.0;
        }
        prev_err_x_ = prev_err_y_ = 0;
        if (lock_yaw_after_first_detection_) {
            if (target_seen_once_) {
                // A区红车往复移动：首次看见之后失去视野，保持当前位置与航向。
                // 绝不重新旋转或向原目标方向盲追。
                sendHoverCommand();
                ROS_INFO_THROTTLE(1.0, "[%s] A_ZONE_WAIT_TARGET: seen once, HOLD pose/yaw",
                                  name().c_str());
            } else {
                // 首次见球前只做有限角度扫描，不允许原地连转多圈。
                const double yaw = PoseUtils::getYawFromQuat(pose.orientation);
                const double dyaw = std::atan2(std::sin(yaw - initial_yaw_),
                                               std::cos(yaw - initial_yaw_));
                if (std::isfinite(yaw) && search_yaw_rate_ > 0.0 &&
                    dyaw < initial_search_max_yaw_rad_) {
                    publishVelocityWithYawRate(0,0,0,search_yaw_rate_);
                    ROS_INFO_THROTTLE(1.0, "[%s] A_ZONE_SEARCH_FIRST: yaw_offset=%.2f/%.2f",
                                      name().c_str(),dyaw,initial_search_max_yaw_rad_);
                } else {
                    sendHoverCommand();
                    ROS_INFO_THROTTLE(1.0, "[%s] A_ZONE_WAIT_FIRST: limited scan done, HOLD at lane",
                                      name().c_str());
                }
            }
        } else {
            publishVelocityWithYawRate(0,0,0,search_yaw_rate_);
            ROS_WARN_THROTTLE(1.0, "[%s] VisualServo lost target, searching with VX=0",name().c_str());
        }
        return NodeStatus::RUNNING;
    }

    const double img_area = static_cast<double>(img_w_) * img_h_;
    const double ratio = last_target_.area() / img_area;
    const double err_x = last_target_.center_x - img_cx_;
    const double err_y = last_target_.center_y - img_cy_;
    max_area_ratio_ = std::max(max_area_ratio_, ratio);
    const double yaw_rate = computeYawRate(err_x, dt);
    // 保留小幅垂直对准；距离下限近时暂停所有升降像素伺服。
    const double vz_raw = horizontal_only_alignment_ ? 0.0 : computeVz(err_y,dt);
    const double vz = horizontal_only_alignment_ || z<floor_z_+0.14 ? 0.0 :
        std::clamp(vz_raw,-max_servo_vz_,max_servo_vz_);

    switch (current_state_) {
        case ALIGN: {
            // 水平偏差过大时绝不允许向前移动，只调yaw及少量高度。
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            if ((!lock_yaw_after_first_detection_ || target_seen_once_) &&
                std::abs(err_x) <= sprint_center_limit_px_ &&
                (horizontal_only_alignment_ || std::abs(err_y) <= 55.0)) {
                // 只统计不同的YOLO检测帧，而不是高频BT tick。
                if (last_target_.timestamp > last_align_detection_stamp_ + 1e-4) {
                    last_align_detection_stamp_ = last_target_.timestamp;
                    ++align_stable_frames_;
                }
                if (align_stable_frames_ >= align_frames_required_) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    ROS_INFO("[%s] ALIGN stable -> APPROACHING (x=%.1fpx y=%.1fpx)",
                             name().c_str(),err_x,err_y);
                }
            } else {
                align_stable_frames_ = 0;
                last_align_detection_stamp_ = 0.0;
            }
            ROS_INFO_THROTTLE(1.0, "[%s] ALIGN x=%.1f y=%.1f stable=%d",
                              name().c_str(), err_x, err_y, align_stable_frames_);
            break;
        }
        case APPROACHING: {
            if (std::abs(err_x) > approach_center_limit_px_) {
                // 显著偏移时停止前进并重新对准，避免跟着转弯向墙体飘。
                publishVelocityWithYawRate(0, 0, vz, yaw_rate);
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                sprint_confirmation_streak_ = 0;
                sprint_last_detection_stamp_ = 0.0;
                ROS_WARN("[%s] APPROACH misaligned x=%.1fpx, stop VX -> ALIGN",
                         name().c_str(), err_x);
                break;
            }
            const double align_ratio = std::clamp(
                1.0 - std::abs(err_x) / approach_center_limit_px_, 0.0, 1.0);
            const double vx = forward_velocity_ * align_ratio;
            publishVelocityWithYawRate(vx, 0, vz, yaw_rate);

            // 追踪时间+连续有效帧双重门槛：避免红车短暂经过就仓促撞击。
            const bool can_sprint = ratio >= approach_area_threshold_ &&
                std::abs(err_x) <= sprint_center_limit_px_ &&
                std::abs(err_y) <= sprint_vertical_limit_px_ &&
                (!lock_yaw_after_first_detection_ ||
                 (target_seen_once_ &&
                  (now - lock_confirmed_at_).toSec() >= min_tracking_after_lock_sec_));
            if (can_sprint) {
                if (last_target_.timestamp > sprint_last_detection_stamp_ + 1e-4) {
                    sprint_last_detection_stamp_ = last_target_.timestamp;
                    ++sprint_confirmation_streak_;
                }
                if (sprint_confirmation_streak_ >= sprint_confirmation_required_) {
                    current_state_ = SPRINTING;
                    sprint_start_time_ = now;
                    strike_was_visible_ = target_visible_;
                    ROS_INFO("[%s] APPROACH -> SPRINT area=%.3f err_x=%.1f err_y=%.1f stable=%d",
                             name().c_str(),ratio,err_x,err_y,sprint_confirmation_streak_);
                }
            } else {
                sprint_confirmation_streak_ = 0;
                sprint_last_detection_stamp_ = 0.0;
            }
            ROS_INFO_THROTTLE(0.5, "[%s] APPROACH vx=%.2f yaw_rate=%.3f err_x=%.1f area=%.3f",
                              name().c_str(), vx, yaw_rate, err_x, ratio);
            break;
        }
        case SPRINTING: {
            const double elapsed = (now - sprint_start_time_).toSec();
            if (elapsed >= sprint_time_) {
                publishVelocityWithYawRate(0,0,0,0);
                brake_start_time_ = now;
                current_state_ = BRAKING;
                ROS_INFO("[%s] SPRINT -> BRAKING after %.2fs", name().c_str(), elapsed);
                break;
            }
            if (std::abs(err_x) > approach_center_limit_px_) {
                publishVelocityWithYawRate(0,0,0,0);
                current_state_=BRAKING;brake_start_time_=now;
                ROS_WARN("[%s] SPRINT stopped on lateral error %.1fpx",name().c_str(),err_x);
                break;
            }
            // 冲刺期间极小幅调yaw，避免机头边大角度旋转边高速前进。
            const double limited_yaw = std::clamp(
                yaw_rate, -max_sprint_yaw_rate_, max_sprint_yaw_rate_);
            publishVelocityWithYawRate(sprint_velocity_, 0,
                std::clamp(vz, -0.15, 0.15), limited_yaw);
            ROS_INFO_THROTTLE(0.3, "[%s] SPRINT vx=%.2f limited_yaw=%.3f t=%.2f/%.2f",
                              name().c_str(), sprint_velocity_, limited_yaw, elapsed, sprint_time_);
            break;
        }
        case BRAKING:
            break; // 前面已处理
    }
    return NodeStatus::RUNNING;
}

// =====================================================================
// PD 控制器
// =====================================================================

double VisualServo::computeYawRate(double err_x, double dt) {
    // 目标在右侧(err_x>0) → 需要右转 → yaw_rate < 0 (NED 逆时针为正)
    double err = err_x;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_yaw_) {
        err_after_dz = err - std::copysign(deadzone_yaw_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_x_) / dt : 0.0;
    double yaw_rate = -(k_p_yaw_rate_ * err_after_dz + k_d_yaw_rate_ * d_err);
    
    // 限幅
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
    
    prev_err_x_ = err_after_dz;
    return yaw_rate;
}

double VisualServo::computeVz(double err_y, double dt) {
    // 目标在下方(err_y>0) → 需要下降 → vz < 0 (看飞控坐标系)
    double err = err_y;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_vz_) {
        err_after_dz = err - std::copysign(deadzone_vz_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_y_) / dt : 0.0;
    double vz = -(k_p_vz_ * err_after_dz + k_d_vz_ * d_err);
    
    // 限幅
    vz = std::clamp(vz, -max_vz_, max_vz_);
    
    prev_err_y_ = err_after_dz;
    return vz;
}

// =====================================================================
// 辅助函数
// =====================================================================

void VisualServo::updateTargetData() {
    auto target_res = getInput<PixelTarget>("pixel_target");
    if (!target_res) return;

    PixelTarget current_target = target_res.value();
    const double now_sec = ros::Time::now().toSec();
    // 不允许旧黑板值每tick伪装成新帧；使用TargetTracker填充的真实帧时间。
    const bool valid = current_target.is_detected &&
        std::isfinite(current_target.center_x) && std::isfinite(current_target.center_y) &&
        std::isfinite(current_target.width) && std::isfinite(current_target.height) &&
        current_target.width > 0 && current_target.height > 0 &&
        std::isfinite(current_target.timestamp) && current_target.timestamp > 0 &&
        (now_sec - current_target.timestamp) >= -0.2 &&
        (now_sec - current_target.timestamp) <= data_timeout_;
    target_visible_=valid;
    // 只有新的无目标检测结果(通过Tracker输出invalid)才当作红球消失。
    explicit_no_target_frame_=(!current_target.is_detected &&
        current_target.timestamp>0 &&
        (now_sec-current_target.timestamp)>=-0.2 &&
        (now_sec-current_target.timestamp)<=data_timeout_);
    if (explicit_no_target_frame_ && !target_seen_once_) {
        lock_confirmation_streak_ = 0;
        lock_last_detection_stamp_ = 0.0;
    }
    std_msgs::Bool visible;visible.data=target_visible_;visible_pub_.publish(visible);
    if (valid) {
        // 首次目标锁定必须连续多帧：单帧闪现/误检不会阻止继续搜转。
        if (lock_yaw_after_first_detection_ && !target_seen_once_ &&
            current_target.timestamp > servo_start_stamp_ &&
            current_target.timestamp > lock_last_detection_stamp_ + 1e-4) {
            if (lock_last_detection_stamp_ > 0.0 &&
                current_target.timestamp - lock_last_detection_stamp_ > data_timeout_)
                lock_confirmation_streak_ = 0;
            lock_last_detection_stamp_ = current_target.timestamp;
            ++lock_confirmation_streak_;
            ROS_INFO_THROTTLE(0.5, "[%s] A_ZONE_LOCK_CANDIDATE %d/%d",
                              name().c_str(),lock_confirmation_streak_,lock_confirmation_required_);
            if (lock_confirmation_streak_ >= lock_confirmation_required_) {
                target_seen_once_ = true;
                lock_confirmed_at_ = ros::Time::now();
                ROS_INFO("[%s] A_ZONE_FIRST_TARGET_LOCKED: %d consecutive frames",
                         name().c_str(),lock_confirmation_streak_);
            }
        }
        last_target_ = current_target;
        last_valid_detection_time_.fromSec(current_target.timestamp);
        has_ever_received_data_ = true;
        
        double img_area = static_cast<double>(img_w_) * img_h_;
        double ratio = last_target_.area() / img_area;
        if (ratio > max_area_ratio_) {
            max_area_ratio_ = ratio;
        }
    }
}

bool VisualServo::isTargetLost() {
    if (!has_ever_received_data_) return true;
    return (ros::Time::now() - last_valid_detection_time_).toSec() > data_timeout_;
}

bool VisualServo::isCentered() const {
    double err_x = last_target_.center_x - img_cx_;
    double err_y = last_target_.center_y - img_cy_;
    double tol_w = last_target_.width * center_tolerance_;
    double tol_h = last_target_.height * center_tolerance_;
    return std::abs(err_x) < tol_w && std::abs(err_y) < tol_h;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<VisualServo>("VisualServo");
}
