# 2026-10-16 开发迭代：ArUco返框、目标搜索、规划返航
> 状态：GitHub已改动后，GPUFree编译与实测待验证。旧可运行dev回退点23f9548；main、official-demo-baseline不动。

## 范围与行为
- ArUco成功：PrecisionMove仅yaw=pi原地转向，随后PrecisionMove(goal={planning_end_pos})以首次穿框时记下的完整XYZ（含原高度）斜向返回；不升到固定1.8m、不重新定位方框。反穿框和两次0.5m相对移动保持不变。
- hit_balloon:<color>：先BalloonObserve(front)固定XYZ慢速360度搜索，若可靠看到目标直接进入**原区域航点与原VisualServo**；若找不到则PrecisionMove(height=1.6)原地升高，再BalloonObserve(down)使能独立下视YOLO，观测到或超时后均继续原区域与原VisualServo。下视仅观测，不估距离、不替代视觉伺服、不更改后续撞击路径。
- land：原直线GoTo回家改为1.8s悬停缓冲→PlanningControl(0,0,1.0)→PrecisionMove(0,0,0.5;yaw=0)→Land；规划器没有全图/充分点云时仍可能避障失败。
- 下视图像/rflysim/sensor2/img_rgb→独立det_color.py实例→/objects_down（source_id=2）；启动时模型加载，推理默认关闭，通过/detect_ctrl_down使能；状态/balloon_search/status。
- PrecisionMove新增yaw-only、height-only两种输入，不改原完整goal的航段。
- 已知旧bag中的/rflysim/uav1/local/odom.twist.linear长期为0，PrecisionMove实际速度保护并非已验证；此轮按用户范围不涉及PID/速度来源重构。

## 部署
仅在安全落地、未运行任务时：
```bash
cd ~/catkin_ws
git status --short --branch
git switch dev
git fetch origin
git pull --ff-only origin dev
conda activate demo
set -o pipefail
./build.sh 2>&1 | tee /tmp/search_return_build.log
source devel/setup.bash
ls -l devel/lib/balloon_observe.so
python3 -m py_compile src/object_det/scripts/det_color.py
```
先重启仿真环境的正常冷启动序列：~/PX4PSP/SITLRun.sh，之后conda demo, export ROS_MASTER_URI=http://127.0.0.1:11311, cd ~/catkin_ws, bash ./sim_env_start.sh。该脚本kill_ros_pid.sh会killall -9 python3，不可在飞机在空中时重新运行。
建议检查/down_camera对应ROS topic存在，/objects_down有发布者，/balloon_search/status在一次搜索中有BEGIN/FOUND或NOT_FOUND_RAISE，/planning/pos_cmd正常，飞行结果+仿真画面全程人工观察。
rosbag:/rflysim/uav1/local/odom /mavros/setpoint_raw/local /mission_command /mission_result /aruco/detected_id /objects /objects_down /balloon_search/status /planning/pos_cmd /move_base_simple/goal。
异常先安全降落再检查日志；回退旧版本：飞机落地且工作区干净后git switch --detach 23f9548，./build.sh，冷启动原脚本；不强制覆盖工作区，不改main/tag。
