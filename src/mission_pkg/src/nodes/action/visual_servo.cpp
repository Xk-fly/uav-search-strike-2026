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
             
             // 其他
             InputPort<double>("data_timeout", 1.0, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.08, "触发冲刺的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             InputPort<double>("search_yaw_rate", 0.3, "搜索模式旋转速度 (rad/s)") };
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
    if (!std::isfinite(forward_velocity_) || forward_velocity_ <= 0 ||
        !std::isfinite(sprint_velocity_) || sprint_velocity_ <= 0 ||
        !std::isfinite(sprint_time_) || sprint_time_ <= 0 ||
        !std::isfinite(max_sprint_yaw_rate_) || max_sprint_yaw_rate_ <= 0 ||
        !std::isfinite(approach_center_limit_px_) || approach_center_limit_px_ <= 0 ||
        !std::isfinite(sprint_center_limit_px_) || sprint_center_limit_px_ <= 0 ||
        !std::isfinite(brake_time_) || brake_time_ < 0) {
        throw BT::RuntimeError("VisualServo invalid speed/alignment/brake parameters");
    }
    
    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    search_yaw_rate_ = getInput<double>("search_yaw_rate").value();
    
    // 初始化状态机
    current_state_ = ALIGN;  // 先对准，再允许前进
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    
    // 初始化微分状态
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();
    
    updateTargetData();
    
    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
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

    if (current_state_ == BRAKING) {
        sendHoverCommand();
        if ((now - brake_start_time_).toSec() >= brake_time_) {
            ROS_INFO("[%s] VisualServo BRAKE done: strike attempt complete.", name().c_str());
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    if (isTargetLost()) {
        // 上一帧黑板目标不代表当前仍然看到球；冲刺丢目标必须立即停止。
        sendHoverCommand();
        if (current_state_ == SPRINTING) {
            ROS_WARN("[%s] VisualServo SPRINT aborted: target lost. No blind sprint.", name().c_str());
            return NodeStatus::FAILURE;
        }
        current_state_ = ALIGN;
        align_stable_frames_ = 0;
        prev_err_x_ = prev_err_y_ = 0;
        publishVelocityWithYawRate(0, 0, 0, search_yaw_rate_);
        ROS_WARN_THROTTLE(1.0, "[%s] VisualServo lost target, searching with VX=0", name().c_str());
        return NodeStatus::RUNNING;
    }

    const double img_area = static_cast<double>(img_w_) * img_h_;
    const double ratio = last_target_.area() / img_area;
    const double err_x = last_target_.center_x - img_cx_;
    const double err_y = last_target_.center_y - img_cy_;
    max_area_ratio_ = std::max(max_area_ratio_, ratio);
    const double yaw_rate = computeYawRate(err_x, dt);
    const double vz = computeVz(err_y, dt);

    switch (current_state_) {
        case ALIGN: {
            // 水平偏差过大时绝不允许向前移动，只调yaw及少量高度。
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            if (std::abs(err_x) <= sprint_center_limit_px_ &&
                std::abs(err_y) <= 55.0) {
                ++align_stable_frames_;
                if (align_stable_frames_ >= ALIGN_STABLE_FRAMES_REQUIRED) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    ROS_INFO("[%s] ALIGN stable -> APPROACHING (x=%.1fpx)", name().c_str(), err_x);
                }
            } else {
                align_stable_frames_ = 0;
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
                ROS_WARN("[%s] APPROACH misaligned x=%.1fpx, stop VX -> ALIGN",
                         name().c_str(), err_x);
                break;
            }
            const double align_ratio = std::clamp(
                1.0 - std::abs(err_x) / approach_center_limit_px_, 0.0, 1.0);
            const double vx = forward_velocity_ * align_ratio;
            publishVelocityWithYawRate(vx, 0, vz, yaw_rate);

            // 只有目标较近且高度/水平方向对准时允许冲刺。
            if (ratio >= approach_area_threshold_ &&
                std::abs(err_x) <= sprint_center_limit_px_ &&
                std::abs(err_y) <= 55.0) {
                current_state_ = SPRINTING;
                sprint_start_time_ = now;
                ROS_INFO("[%s] APPROACH -> SPRINT area=%.3f err_x=%.1f",
                         name().c_str(), ratio, err_x);
            }
            ROS_INFO_THROTTLE(0.5, "[%s] APPROACH vx=%.2f yaw_rate=%.3f err_x=%.1f area=%.3f",
                              name().c_str(), vx, yaw_rate, err_x, ratio);
            break;
        }
        case SPRINTING: {
            const double elapsed = (now - sprint_start_time_).toSec();
            if (elapsed >= sprint_time_) {
                sendHoverCommand();
                brake_start_time_ = now;
                current_state_ = BRAKING;
                ROS_INFO("[%s] SPRINT -> BRAKING after %.2fs", name().c_str(), elapsed);
                break;
            }
            if (std::abs(err_x) > approach_center_limit_px_) {
                sendHoverCommand();
                ROS_WARN("[%s] SPRINT aborted: lateral error %.1fpx", name().c_str(), err_x);
                return NodeStatus::FAILURE;
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
    if (valid) {
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
