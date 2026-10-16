# ArUco 往返精准移动：未在 GPUFree 编译/飞行验证的开发提交

> 修改目的：限制去 ArUco 和从 ArUco 返回到反向穿框入口时的速度/加速度，减少快速位移、超调和穿模风险。
> 只在 dev 进行代码修改；main、official-demo-baseline 及气球 VisualServo 不改。
> **GitHub 源码提交 ≠ 编译成功 ≠ 实际防穿模已验证。**

## 代码变化
- 新增 `PrecisionMove` BT 插件，直线路径上生成连续位置参考点，具有 ACCELERATE/CRUISE/DECELERATE/SETTLE 状态及实际速度、位置、稳定时长联合到点条件。
- `detect_aruco` 改为 `PrecisionMove(4.5,0.5,1.2) → ArucoDetector`，取消原 Parallel 中 GoTo 与 ArucoDetector 同时发送位置/悬停指令的冲突。
- `return_cross_frame` 中两个大航点 GoTo 和两个机体系 0.5m GoTo 采用 PrecisionMove；**实际穿框的 PlanningControl 保持不变**。
- 原 GoTo、VisualServo、TargetTracker、其他任务分支无改动。
- `llm_mission.launch` 新增 `precision_move/*` 参数。注意：参数只在启动前加载；运行中 rosparam set 不会立即调整已运行的 PrecisionMove。

## 初始参数（待实测）
- `precision_move/max_speed=0.35` m/s：三维直线路径参考速度上限（不等同 PX4 实际速度硬限制）。
- `precision_move/max_accel=0.25` m/s²：参考点速度变化上限。
- `precision_move/max_lead=0.25` m：参考点沿路径最大超前量。
- `precision_move/tolerance=0.12` m；`speed_tolerance=0.10` m/s；`settle_time=0.70` s。
- `precision_move/abort_speed=0.70` m/s：实测速度超过即发悬停指令并报 FAILURE。
- `precision_move/timeout=60` s；`odom_timeout=1.0` s；`yaw_rate=0.5` rad/s。

## 部署（无人机必须已落地，全部脚本未运行）
```bash
cd ~/catkin_ws
git status --short --branch
git switch dev
git fetch origin
git pull --ff-only origin dev
./build.sh
source devel/setup.bash
rospack find mission_pkg
ls -l devel/lib/precision_move.so
```
如果有未提交本地修改或分支不一致，先检查并备份，**不能直接 reset --hard**。

## 分阶段测试
1. 静态审查：`grep -n "PrecisionMove" src/mission_pkg/src/bt_ros.cpp`，`rosparam` 配置在新 launch 中。
2. 在 PX4/RflySim 已启动且飞机初始 disarmed 后，再运行官方 ROS/AI 脚本。检查 bt 插件加载和正式 ROS master 11311。
3. 在完整六步的首次 SITL 飞行中，观察去程、识别、转向、返框各段 `PrecisionMove BEGIN/stage/SUCCESS`，以及 ArUco ID 和最终降落。
4. 录制 `/rflysim/uav1/local/odom`、`/mavros/setpoint_raw/local`、`/mission_result`，比较实测速度峰值、终点速度、超调和障碍穿模情况；不能只看状态 SUCCESS。

## 已知边界
- 新插件产生直线路径，不做绕障；有障碍物时需要规划器或安全航点。
- 强制速度参数限制的是指令参考点进度，不是 PX4 物理速度硬上限；实际越限会触发保护并造成任务失败，须实测整定。
- Return_cross_frame 内的 Ego-Planner / PlanningControl、框中心定位、气球追踪与冲刺保留原样，后续另做优化。
- 原生 `ArucoDetector` 服务不可用时仍可能任务 FAILURE 并悬停，应在后续单独治理安全降落。
- 改动后先单航段及完整流程低风险 SITL 验证，确定无回归后再考虑扩大使用范围。

## 回退
在服务器上检查 Git 工作区干净后，选用**明确的上一个已知可运行 dev commit**进行 `git switch --detach <旧commit>` + 重新编译验证；不能重写 main、官方 tag 或使用强制 push。
