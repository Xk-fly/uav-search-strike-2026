# 2026-10-16 气球结束判定+实际速度稳定返航：实现与验证
> 状态：GitHub dev 已修改；GPUFree 编译和仿真飞行尚待用户验证。稳定回退点：3bc83a2ca5a3f4e62543f4179971ff911f1dbb7e。
> main 与 official-demo-baseline 不修改。

## 本轮变更
1. ArUco: 识别后朝 `planning_end_pos` 转→`PrecisionMove`回到完整XYZ→**新增**朝 `frame_center` 原地转向→Ego一次性反穿框并延长1m→离框再次朝 `frame_center` 转向。其它穿框几何不变。
2. A区：`balloon_zone_A/approach_pos` 和 `goal` 高度从1.00m提高至1.15m，位置XY不变。VisualServo新增绝对Z高度下限0.90m、相对进入位置下降不超过0.15m和垂直伺服速度限幅0.10m/s。实际采用`max(0.90, entry_z-0.15)`为允许最低高度；接近下限时停止平移/视觉速度、发送绝对高度回升设定，4秒回升失败报告FAILURE。
3. 红球布尔事件：仅在已经进入SPRINT且确实在开始时看到目标的前提下，**收到第一帧有效的无目标检测**，发布 `/balloon_attack/target_visible=false`、`/balloon_attack/hit_candidate=true`，立即下发零速度、进入BRAKING，不再继续向前冲。YOLO默认只发布有目标的消息，故前视`detection_color.launch`启用`publish_empty=true`。在`TargetDetectorBase`修正"本轮BT tick没有新ROS消息"不再伪造成负帧的bug：使用预先定义的`handleNoNewData()`。前视检测帧变成空数据时才进入候选命中；无数据超时会安全停止，但不产生命中布尔事件。这个布尔值是按竞赛当前经验定义的业务近似，不是物理破裂、碰撞评分真值。
4. VisualServo的BRAKING和正常Sprint结束时发送零速度而非高速抓取固定位置，降低惯性风险。
5. `land`任务将固定等待1.8s改成`StableHover`：依次 BRAKE(零速度) → 测得速度足够低后HOLD(定点悬停) → 速度估算<=0.16m/s持续0.9秒后SUCCESS → **原有**Ego(0,0,1)→PrecisionMove(0,0,0.5)→Land。最大等待9秒；timeout/odom异常时停止任务、不把Ego强行启动。`StableHover`用odom位置差分(0.2s窗口)计算速度，**不依赖之前实测为0的odom.twist.linear**；旧数据超过0.4s不会给出SUCCESS。
6. Ego全局地图/激光雷达**本次没有配置**；当前只有本地深度点云，无法凭机身模型外观推断ROS确有LiDAR，必须先检查SDK和平台支持。见`docs/LIDAR_RFLYSIM_CHECK_20261016.md`。

## 服务器部署/静态编译检查
**必须先停机落地、确认disarmed，不要飞行中重跑sim_env_start.sh**。
```bash
cd ~/catkin_ws
git status --short --branch
git switch dev
git fetch origin
git pull --ff-only origin dev
git rev-parse --short HEAD
conda activate demo
python3 - <<'PY'
import xml.etree.ElementTree as E
E.parse('src/mission_pkg/launch/llm_mission.launch')
E.parse('src/object_det/launch/detection_color.launch')
print('ROS launch XML OK')
PY
python3 -m py_compile src/object_det/scripts/det_color.py
bash -n sim_env_start.sh
set -o pipefail
./build.sh 2>&1 | tee /tmp/strike_stable_build.log
source devel/setup.bash
ls -lh devel/lib/stable_hover.so devel/lib/visual_servo.so devel/lib/precision_move.so
```
启用ROS主流程前检查新`StableHover`插件是否加载，原ROS前端及模型运行正常。

## 运行中验证
```bash
rostopic info /balloon_attack/target_visible
rostopic info /balloon_attack/hit_candidate
rostopic echo /balloon_attack/hit_candidate
rosparam get /visual_servo/min_world_z
rosparam get /land/stable_speed_mps
rosparam get /balloon_zone_A/approach_pos
```
日志应出现：
- `PrecisionMove face_goal` → 第二次面向 `frame_center` → Ego反向穿框。
- `VisualServo entry z=...`，发生下限时`VisualServo floor protection`，期望最低高度明显高于上次0.33m。
- 冲刺时红球消失：`HIT_CANDIDATE...`，零速度BRAKING；随后`StableHover BEGIN`, `BRAKE->HOLD`, `SUCCESS...`；只有后者SUCCESS才能看到Ego返航新goal。
- 如果命中未触发布尔，但Sprint到0.6s结束，仍启动 BRAKING/StableHover 后续返航，不会继续盲冲。
- 若StableHover超过9s无法稳定，则不启动Ego，需人手安全接管。

rosbag建议新增`/balloon_attack/target_visible`、`/balloon_attack/hit_candidate`，并继续录`/rflysim/uav1/local/odom`、`/mavros/setpoint_raw/local`、`/objects`、`/move_base_simple/goal`、`/planning/pos_cmd`、`/mission_result`。

## 已知风险
- 1帧红球漏检也可能触发命中候选，此为用户明确接受的比赛权衡。
- 高度下限是世界Z而非车顶障碍物几何净空；在移动小车顶部需看实测。
- Zero velocity是期望速度，不是瞬时停止，实际会受控制器与物理模型影响。
- Ego仍使用局部深度点云，若返航起点已在墙内或传感器漏看障碍仍可能撞墙。本次没有实现全局建图。
- 旧PrecisionMove其余航段依赖odom.twist为零的缺陷尚未修正；新StableHover独立用位置差分。
- 原行为树仍保留气球任务`ForceSuccess`包装，整体SUCCESS并不等于物理得分。
