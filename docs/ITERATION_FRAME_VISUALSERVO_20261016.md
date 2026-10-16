# 2026-10-16 开发迭代：ArUco动态返程、反穿框一次延长1m、下视窗口、气球撞击控制

> 状态：**GitHub源代码已提交，GPUFree 编译与仿真飞行尚未验证**。旧运行版本回退点 `224d5e01e0d8fb863feefeb7588bcbe11fe5889e`。main与official-demo-baseline不变。

## 本轮修改的任务顺序
1. 首次Ego穿框保存 `planning_end_pos` (完整XYZ) 与 `frame_center`。
2. ArUco识别成功后，`PrecisionMove face_goal={planning_end_pos} yaw_rate=0.35` 动态计算 atan2 航向，在原位慢转；`PrecisionMove goal={planning_end_pos}` 平滑斜向返回。
3. 单个 `PlanningControl goal={frame_center} through_from={planning_end_pos} through_distance=1.0` 动作，反穿框并连续规划到框另一侧1米：`goal_xy=frame_center_xy + 1.0*(frame_center_xy-planning_end_pos_xy)/norm(...)`，`goal_z=frame_center.z`。注意！旧 `offset_z` 直接沿机头yaw，不能保证穿框方向，故新增 `through_from` 专用几何逻辑。旧两次0.5m `PrecisionMove` 已删除。
4. 通过后 `PrecisionMove face_goal={frame_center} yaw_rate=0.35` 原地转向面朝框；随后按原逻辑 `PlanningControl` 去气球A/B/C接近点，并保持原A区坐标；不再触发前一版前视360搜索/失败升高/下视打击分支。
5. `VisualServo`：启动ALIGN要求连续有效对准；APPROACH速度从1.0改0.4m/s、偏移48px以上停前进重对准；距近/像素面积达到阈值且偏移24px内才SPRINT；SPRINT改0.9m/s×0.6s，yaw速率不大于0.08rad/s；冲刺目标丢失或偏差超限立即结束并报告FAILURE，禁止盲冲；冲刺正常结束BRAKING悬停0.5s；旧黑板目标通过时间戳检查新鲜度。这些都是SITL保守测试初值，可能需要回调，不能保证一次撞中。
6. 下视YOLO `/rflysim/sensor2/img_rgb` 窗口 `Down Camera (RGB)`，在 `initially_enabled=false` 时依旧显示原图，且不做YOLO推理；/objects_down不参与现阶段BT视觉伺服。前视窗口行为保持原样。
7. `land` 返航分支完全冻结，继续旧Ego → PrecisionMove → Land。

## 建议服务器部署 (落地disarmed；无任务运行)
```bash
cd ~/catkin_ws
git status --short --branch
git switch dev
git fetch origin
git pull --ff-only origin dev
conda activate demo
python3 -m py_compile src/object_det/scripts/det_color.py
bash -n sim_env_start.sh
set -o pipefail
./build.sh 2>&1 | tee /tmp/uav_reverse_servo_build.log
source devel/setup.bash
ls -lh devel/lib/precision_move.so devel/lib/planning_control.so devel/lib/visual_servo.so
```
冷启动：先`cd ~/PX4PSP && ./SITLRun.sh`，场景完全就绪后`conda activate demo; export ROS_MASTER_URI=http://127.0.0.1:11311; cd ~/catkin_ws; bash ./sim_env_start.sh`。飞行中不要重跑脚本，kill_ros_pid.sh含危险进程清理。检查独立下视窗口和 `rostopic hz /rflysim/sensor2/img_rgb`。

## 核验重点
- 行为树日志动态yaw；反穿框时必须只生成一个Ego目标，且比frame_center多穿过1m；落到出口点后朝frame_center原地转向。
- A区仍为Ego区域规划，旧跟踪/打击路径未改；VisualServo必须先ALIGN才APPROACH；观察A区穿模以及实际是否击中红球。
- 保存完整BT/xterm日志、录制`/rflysim/uav1/local/odom /mavros/setpoint_raw/local /planning/pos_cmd /move_base_simple/goal /objects /mission_command /mission_result /aruco/detected_id`；可额外加下视图像(包很大)。
- 原 `ForceSuccess` 包装仍可能掩盖VisualServo失败，SUCCESS非可靠命中证据。参考用户目视与视频判断。
- Ego局部地图、飞控动态仍不能保证绝对无碰撞；老 `PrecisionMove` 读取的odom twist速度可能恒零，未在本轮修复。首轮必须受控观察。
- 如果编译错误，保存`/tmp/uav_reverse_servo_build.log`；如果过程异常，安全着陆后先回退旧工作版本 `git switch --detach 224d5e0; ./build.sh`，勿强制重置或在空中关闭任务。
