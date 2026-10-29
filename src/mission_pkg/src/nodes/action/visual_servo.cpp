/******************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 *
 * @details    三阶段：对齐(ALIGN) -> 靠近(APPROACH) -> 冲刺(SPRINT)
 *             搜索固定位置缓转；见球后使用 publishVelocityWithYawRate（body 坐标系）
 *             控制变量：VX(前进) + VZ(升降) + yaw_rate(偏航角速度)
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
#include "utils/pose_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>
#include <algorithm>
#include <cmath>
#include <limits>


VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), 
      current_state_(ALIGN),
      has_ever_received_data_(false) {
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
             InputPort<double>("sprint_time", 1.2, "冲刺持续时间 (s)"),
             
             // 其他
             InputPort<double>("data_timeout", 1.0, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.08, "触发冲刺的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             InputPort<double>("search_yaw_rate", 0.3, "固定位置搜索的转向速度(rad/s)"),
             InputPort<double>("hit_lost_time", 0.25, "连续丢球时间；仅冲刺中丢球视作完成(s)"),
             InputPort<double>("min_servo_height", 0.40, "末端伺服机体最低高度(m)"),
             InputPort<double>("sprint_center_px", 80.0, "冲刺前最大横向像素误差"),
             InputPort<bool>("avoid_right_wall", false, "红球末端启用右侧墙体净空检查"),
             InputPort<double>("right_wall_margin", 0.50, "右侧墙体最小净空(m)") };
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
    
    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    search_yaw_rate_ = getInput<double>("search_yaw_rate").value();
    hit_lost_time_ = getInput<double>("hit_lost_time").value();
    min_servo_height_ = getInput<double>("min_servo_height").value();
    sprint_center_px_ = getInput<double>("sprint_center_px").value();
    avoid_right_wall_ = getInput<bool>("avoid_right_wall").value();
    right_wall_margin_ = getInput<double>("right_wall_margin").value();
    {
        std::lock_guard<std::mutex> lock(wall_mutex_);
        wall_cloud_received_at_ = ros::Time(0);
        right_wall_clearance_ = std::numeric_limits<double>::infinity();
    }
    if (avoid_right_wall_) {
        auto nh = mct->getROSHandle();
        wall_clearance_pub_ = nh->advertise<std_msgs::Float32>("/visual_servo/right_wall_clearance", 10);
        wall_cloud_sub_ = nh->subscribe("/cloud_registered", 1, &VisualServo::wallCloudCallback, this);
    }
    
    // 初始化状态机
    current_state_ = APPROACHING;  // 直接进入边飞边对齐模式（气球是移动目标）
    has_ever_received_data_ = false;
    align_stable_frames_ = 0;
    
    // 初始化微分状态
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();
    search_pos_ = fcu_odom_ptr->pose.pose.position;
    search_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    
    updateTargetData();
    
    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
    ROS_INFO("[%s]: k_p_yaw_rate=%.4f k_d=%.4f max_rate=%.2f deadzone=%.1f", 
             name().c_str(), k_p_yaw_rate_, k_d_yaw_rate_, max_yaw_rate_, deadzone_yaw_);
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.1f area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_, approach_area_threshold_);
    ROS_INFO("[%s]: min_height=%.2fm sprint_center=%.0fpx right_wall=%d margin=%.2fm",
             name().c_str(), min_servo_height_, sprint_center_px_, avoid_right_wall_, right_wall_margin_);
    
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    const bool target_detected = updateTargetData();
    
    // 计算时间步长
    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt < 0.001) dt = 0.001;

    bool right_wall_near = false;
    bool wall_cloud_ready = !avoid_right_wall_;
    double wall_clearance = std::numeric_limits<double>::infinity();
    if (avoid_right_wall_) {
        std::lock_guard<std::mutex> lock(wall_mutex_);
        wall_cloud_ready = !wall_cloud_received_at_.isZero() &&
            (now - wall_cloud_received_at_).toSec() < 0.5;
        wall_clearance = right_wall_clearance_;
        right_wall_near = wall_cloud_ready && wall_clearance < right_wall_margin_;
    }
    if ((!wall_cloud_ready || right_wall_near) && current_state_ == SPRINTING) {
        current_state_ = APPROACHING;
        ROS_WARN_THROTTLE(1.0, "[%s]: Right wall near during SPRINT; stopping forward motion.", name().c_str());
    }
    if (current_state_ == SPRINTING &&
        fcu_odom_ptr->pose.pose.position.z < min_servo_height_ - 0.05) {
        current_state_ = APPROACHING;
        ROS_WARN_THROTTLE(1.0, "[%s]: Low height during SPRINT; stopping forward motion.", name().c_str());
    }

    if (has_ever_received_data_ && isTargetLost()) {
        const double unseen_time = (now - last_valid_detection_time_).toSec();
        if (current_state_ == SPRINTING) {
            sendHoverCommand();
            ROS_INFO("[%s]: Target lost during SPRINT (%.2fs); BRAKE next.",
                     name().c_str(), unseen_time);
            return NodeStatus::SUCCESS;
        }
        // 对齐/接近时丢球，无论目标曾有多近，都只在当前位置重新搜索。
        has_ever_received_data_ = false;
        current_state_ = APPROACHING;
        align_stable_frames_ = 0;
        search_pos_ = fcu_odom_ptr->pose.pose.position;
        search_pos_.z = std::max(search_pos_.z, min_servo_height_);
        search_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        ROS_WARN("[%s]: Target lost before SPRINT (%.2fs); holding position and searching.",
                 name().c_str(), unseen_time);
    }
    
    // 搜索保持进入搜索时的位置，只缓慢改变航向；零机体速度不能锁住真值位置。
    if (!has_ever_received_data_) {
        search_yaw_ += search_yaw_rate_ * dt;
        while (search_yaw_ > M_PI) search_yaw_ -= 2 * M_PI;
        while (search_yaw_ < -M_PI) search_yaw_ += 2 * M_PI;
        publishPosition(search_pos_, search_yaw_);
        prev_time_ = now;
        ROS_WARN_THROTTLE(2.0, "[%s]: Fixed-position search (yaw_rate=%.2f)...",
                          name().c_str(), search_yaw_rate_);
        return NodeStatus::RUNNING;
    }
    
    // 计算基本量
    double img_area = static_cast<double>(img_w_) * img_h_;
    double current_ratio = last_target_.area() / img_area;
    double err_x = last_target_.center_x - img_cx_;  // 正=目标在右侧
    double err_y = last_target_.center_y - img_cy_;  // 正=目标在下方
    
    // PD 控制计算
    double yaw_rate = computeYawRate(err_x, dt);
    double vz = computeVz(err_y, dt);
    const double height = fcu_odom_ptr->pose.pose.position.z;
    // 越靠近高度下限，越早收住下降；只约束指令，不修改原速度上限。
    vz = std::max(vz, std::clamp(2.0 * (min_servo_height_ - height), -max_vz_, max_vz_));
    if (height < min_servo_height_ - 0.03) {
        ROS_WARN_THROTTLE(1.0, "[%s]: Low servo height %.2fm; correcting upward.", name().c_str(), height);
    }
    // MAVROS机体系y正方向为左；净空不足时只侧移，不继续朝墙前进。
    if (!wall_cloud_ready || right_wall_near) {
        const double vy = right_wall_near ? std::min(forward_velocity_, right_wall_margin_ - wall_clearance + 0.10) : 0.0;
        publishVelocityWithYawRate(0, vy, vz, 0);
        prev_time_ = now;
        ROS_WARN_THROTTLE(1.0, "[%s]: Wall clearance: ready=%d right=%.2fm; holding forward, left correction=%.2f.",
                          name().c_str(), wall_cloud_ready, wall_clearance, vy);
        return NodeStatus::RUNNING;
    }
    
    // 更新微分状态
    prev_time_ = now;
    
    // =========== 三阶段状态机 ===========
    switch (current_state_) {
        case ALIGN: {
            // 阶段1：对齐 — 原地旋转 + 升降，使目标居中
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            
            if (isCentered()) {
                align_stable_frames_++;
                if (align_stable_frames_ >= ALIGN_STABLE_FRAMES_REQUIRED) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    ROS_INFO("[%s]: Aligned! err:(%.0f,%.0f) -> APPROACHING", 
                             name().c_str(), err_x, err_y);
                }
            } else {
                align_stable_frames_ = 0;
            }
            
            ROS_INFO_THROTTLE(1.0, "[%s]: ALIGN - err:(%.0f,%.0f) yaw_rate:%.3f vz:%.3f stable:%d", 
                             name().c_str(), err_x, err_y, yaw_rate, vz, align_stable_frames_);
            break;
        }
        
        case APPROACHING: {
            // 阶段2：靠近 — VX 前进 + yaw_rate + VZ，全程持续修正
            // 根据对齐程度调节前进速度（越偏越慢）
            double align_ratio = std::max(0.0, 1.0 - std::abs(err_x) / (img_w_ * 0.3));
            double vx = forward_velocity_ * align_ratio;
            
            const bool close_to_target = current_ratio >= approach_area_threshold_;
            const bool aligned_for_sprint = std::abs(err_x) <= sprint_center_px_ &&
                height >= min_servo_height_ - 0.03;
            const bool ready_to_sprint = target_detected && close_to_target && aligned_for_sprint;
            if ((close_to_target && !aligned_for_sprint) || height < min_servo_height_ - 0.05) {
                vx = 0.0;
                ROS_WARN_THROTTLE(1.0, "[%s]: Holding forward motion: err_x=%.0fpx height=%.2fm wall=%d.",
                                  name().c_str(), err_x, height, right_wall_near);
            }
            publishVelocityWithYawRate(vx, 0, vz, yaw_rate);
            
            // 对齐偏差过大 → 回到 ALIGN
            if (std::abs(err_x) > img_w_ * center_tolerance_ * 3) {
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                ROS_WARN("[%s]: Alignment lost (err_x=%.0f), back to ALIGN", 
                         name().c_str(), err_x);
                break;
            }
            
            // 当前仍见球且面积达阈值才冲刺，不能用丢球前的旧面积触发。
            if (ready_to_sprint) {
                current_state_ = SPRINTING;
                sprint_start_time_ = ros::Time::now();
                ROS_INFO("[%s]: Area %.1f%% >= %.1f%% -> SPRINT! (sprint_vel=%.2f, time=%.1fs)", 
                         name().c_str(), current_ratio*100, approach_area_threshold_*100,
                         sprint_velocity_, sprint_time_);
            }
            
            ROS_INFO_THROTTLE(0.5, "[%s]: APPROACH - vx:%.2f(%.0f%%) yaw_rate:%.3f vz:%.3f err:(%.0f,%.0f) area:%.1f%%", 
                             name().c_str(), vx, align_ratio*100, yaw_rate, vz, 
                             err_x, err_y, current_ratio*100);
            break;
        }
        
        case SPRINTING: {
            // 阶段3：冲刺 — 全速前进，保持 yaw_rate 和 VZ 修正
            double sprint_dt = (ros::Time::now() - sprint_start_time_).toSec();
            
            // 原冲刺时长上限：到时结束本次尝试并制动，不据此宣称命中。
            if (sprint_dt >= sprint_time_) {
                sendHoverCommand();
                ROS_INFO("[%s]: Sprint time limit reached (%.1fs); BRAKE next.", name().c_str(), sprint_dt);
                return NodeStatus::SUCCESS;
            }
            
            // 丢球已由上方统一处理；有目标时保持官方冲刺速度。
            publishVelocityWithYawRate(sprint_velocity_, 0, vz, yaw_rate);
            
            ROS_INFO_THROTTLE(0.2, "[%s]: SPRINT - vx:%.2f yaw_rate:%.3f vz:%.3f (%.1f/%.1fs)", 
                             name().c_str(), sprint_velocity_, yaw_rate, vz,
                             sprint_dt, sprint_time_);
            break;
        }
    }
    
    return NodeStatus::RUNNING;
}

void VisualServo::wallCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != "world") return;
    const auto& pose = fcu_odom_ptr->pose.pose;
    const double yaw = PoseUtils::getYawFromQuat(pose.orientation);
    const double forward_x = std::cos(yaw);
    const double forward_y = std::sin(yaw);
    double clearance = std::numeric_limits<double>::infinity();
    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        const double dx = *x - pose.position.x;
        const double dy = *y - pose.position.y;
        const double ahead = forward_x * dx + forward_y * dy;
        const double right = forward_y * dx - forward_x * dy;
        // 取机体上方墙面，避开地面、车体和正常低位气球；不把正前方球体当墙。
        const double above = *z - pose.position.z;
        if (ahead > -0.15 && ahead < 1.2 && right > 0.05 &&
            above > 0.35 && above < 1.0) {
            clearance = std::min(clearance, right);
        }
    }
    {
        std::lock_guard<std::mutex> lock(wall_mutex_);
        right_wall_clearance_ = clearance;
        wall_cloud_received_at_ = ros::Time::now();
    }
    std_msgs::Float32 sample;
    sample.data = std::isfinite(clearance) ? clearance : -1.0f;
    wall_clearance_pub_.publish(sample);
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

bool VisualServo::updateTargetData() {
    auto target_res = getInput<PixelTarget>("pixel_target");
    if (!target_res) return false;

    PixelTarget current_target = target_res.value();
    if (current_target.is_detected) {
        if (!has_ever_received_data_) {
            // 首次见球不把搜索期间的零误差当成上一帧，避免微分项突跳。
            const double err_x = current_target.center_x - img_cx_;
            const double err_y = current_target.center_y - img_cy_;
            prev_err_x_ = std::copysign(std::max(0.0, std::abs(err_x) - deadzone_yaw_), err_x);
            prev_err_y_ = std::copysign(std::max(0.0, std::abs(err_y) - deadzone_vz_), err_y);
        }
        last_target_ = current_target;
        last_valid_detection_time_ = ros::Time::now(); 
        has_ever_received_data_ = true;
        return true;
    }
    return false;
}

bool VisualServo::isTargetLost() {
    if (!has_ever_received_data_) return true;
    return (ros::Time::now() - last_valid_detection_time_).toSec() >=
           std::min(data_timeout_, hit_lost_time_);
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
