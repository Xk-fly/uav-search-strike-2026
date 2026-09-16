# 项目事实库（PROJECT_FACTS.md）

> 更新时间：2026-09-16  
> 作用：保存已经确认的项目事实、版本、路径、关键接口和状态，减少跨对话幻觉。  
> 规则：这里只记录可追溯的事实；猜测和未来计划放到 `TODO.md`，运行过程放到 `DEBUG_LOG.md`。

## 1. 状态标签

```text
[CONFIRMED]  当前服务器命令、仓库源码或已执行 Git 操作已经确认
[OFFICIAL]   来自赛事官方手册，但当前服务器仍可能需要运行验证
[HISTORICAL] 历史服务器/旧环境曾成立，不能自动代表当前服务器
[PENDING]    尚待当前服务器验证
```

事实冲突优先级：

```text
当前服务器实际输出
> 当前 dev 分支源码/配置
> 赛事官方手册
> 旧服务器历史经验
> 推测
```

## 2. 仓库与版本控制

| 项目 | 值 | 状态 |
|---|---|---|
| GitHub 仓库 | `Xk-fly/uav-search-strike-2026` | [CONFIRMED] |
| 可见性 | Private | [CONFIRMED] |
| Windows 本地路径 | `D:\UAV_Competition\uav-search-strike-2026` | [CONFIRMED] |
| GPUFree 工作空间 | `/home/ubuntu/catkin_ws` | [CONFIRMED] |
| 稳定分支 | `main` | [CONFIRMED] |
| 开发分支 | `dev` | [CONFIRMED] |
| 官方基线标签 | `official-demo-baseline` | [CONFIRMED] |
| 官方基线 commit | `2c0fe57` | [CONFIRMED] |
| 环境路径修复 commit | `3747c37` | [CONFIRMED] |
| 当前服务器最近已知 dev | `6032921` | [CONFIRMED，截至本文件创建前] |

物理备份：

```text
/home/ubuntu/catkin_ws_official_baseline_20260915.tar.gz
约 49M
```

cv_bridge 原嵌套 Git 元数据：

```text
/home/ubuntu/cv_bridge_git_metadata_backup_20260915
```

## 3. 操作系统 / ROS / Python

| 项目 | 值 | 状态 |
|---|---|---|
| OS | Ubuntu 20.04.6 LTS | [CONFIRMED] |
| 运行用户 | `ubuntu` | [CONFIRMED] |
| ROS | ROS 1 Noetic | [CONFIRMED] |
| ROS_MASTER_URI | `http://localhost:11311` | [CONFIRMED] |
| Conda 环境 | `demo` | [CONFIRMED] |
| Python | 3.10.19 | [CONFIRMED] |
| Python 路径 | `/opt/conda/envs/demo/bin/python3` | [CONFIRMED] |
| MAVROS | 1.20.1 | [CONFIRMED] |
| mavros_msgs | 1.20.1 | [CONFIRMED] |
| OpenCV/cv2 | 4.6.0 | [CONFIRMED] |
| NumPy | 1.23.5 | [CONFIRMED] |
| rospy | installed | [CONFIRMED] |

Catkin 加载前：

```text
ROS_PACKAGE_PATH=/opt/ros/noetic/share
```

加载后：

```text
ROS_PACKAGE_PATH=/home/ubuntu/catkin_ws/src:/opt/ros/noetic/share
```

已确认：

```text
mission_pkg  -> /home/ubuntu/catkin_ws/src/mission_pkg
object_det   -> /home/ubuntu/catkin_ws/src/object_det
sensor_pkg   -> /home/ubuntu/catkin_ws/src/sensor_pkg
mavros       -> /opt/ros/noetic/share/mavros
```

## 4. CUDA / PyTorch / GPU

| 项目 | 值 | 状态 |
|---|---|---|
| CUDA Toolkit | 12.8 | [CONFIRMED] |
| nvcc | V12.8.93 | [CONFIRMED] |
| PyTorch | 2.3.1+cu121 | [CONFIRMED] |
| torch CUDA runtime | 12.1 | [CONFIRMED] |
| 当前 GPU | 无卡模式 | [CONFIRMED] |
| `torch.cuda.is_available()` | False | [CONFIRMED] |
| `device_count` | 0 | [CONFIRMED] |
| `/dev/nvidia*` | 当前不存在 | [CONFIRMED] |

正式模型/检测验证时需要有卡模式重新核验。

## 5. PX4 / RflySim

| 项目 | 值 | 状态 |
|---|---|---|
| PX4PSP 根目录 | `/home/ubuntu/PX4PSP` | [CONFIRMED] |
| PX4 目录 | `/home/ubuntu/PX4PSP/Firmware` | [CONFIRMED] |
| PX4 版本 | `v1.12.3` | [CONFIRMED] |
| PX4 commit | `2e8918da66` | [CONFIRMED] |
| RflySim SDK | `/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK` | [CONFIRMED] |
| 仿真启动脚本 | `~/PX4PSP/SITLRun.sh` | [OFFICIAL + 路径已确认] |

`~/PX4PSP` 已确认：

```text
CopterSim/
Firmware/
QGroundControl/
RflySimAPIs/
RflySimUE5/
SITLRun.sh
```

`sim_env_start.sh` 中 RflySim SDK 已修正为：

```bash
RFLYSIM_SDK="${HOME}/PX4PSP/RflySimAPIs/RflySimSDK"
```

并验证：

```text
RflySim SDK path OK
```

## 6. 九格大模型

| 项目 | 值 | 状态 |
|---|---|---|
| 模型根目录 | `/home/ubuntu/model` | [CONFIRMED] |
| 模型目录 | `/home/ubuntu/model/FM9G4B-V` | [CONFIRMED] |
| model 总大小 | 约 8.5G | [CONFIRMED] |
| 主权重 | `FM9G4B-V/pytorch_model.bin` | [CONFIRMED] |
| 主权重大小 | `9,030,068,428 bytes` | [CONFIRMED] |
| 任务脚本 | `/home/ubuntu/model/mission_once.py` | [CONFIRMED] |
| 测试脚本 | `/home/ubuntu/model/test_rflysim.py` | [CONFIRMED] |

提示词：

```text
/home/ubuntu/model/prompts/mission_prompt.txt
/home/ubuntu/model/prompts/basic_cn.txt
/home/ubuntu/model/prompts/system_promt.txt
```

当前模型“实际加载成功”为 [PENDING]。

## 7. 仓库主要模块

```text
src/mission_pkg/      行为树 / 任务控制
src/object_det/       目标检测
src/sensor_pkg/       传感器 / 点云 / 视觉伺服
src/common_msgs/      公共 ROS 消息
src/ego-planner/      路径规划
src/cv_bridge/        ROS 图像桥接依赖
src/behaviortree_cpp/ 行为树依赖
```

## 8. 已确认 launch

`mission_pkg`：

```text
aruco_detect.launch
basic_test.launch
cross_frame.launch
cross_frame_test.launch
full_competition.launch
hit_balloon.launch
llm_mission.launch
```

`object_det`：

```text
detection.launch
detection_color.launch
hsv_detection.launch
```

`sensor_pkg`：

```text
cloud_transform.launch
visual_servo_balloon.launch
```

## 9. 关键 ROS 接口

已知：

```text
/mission_command
/mission_result
/move_base_simple/goal
/planning/pos_cmd
/objects
/mavros/state
/mavros/local_position/odom
/mavros/setpoint_raw/local
/mavros/set_mode
/mavros/cmd/arming
```

传感器相关：

```text
/rflysim/sensor0/Depth_Cloud
/rflysim/sensor1/image_rgb
/rflysim/sensor2/image_rgb
/rflysim/uav1/local/odom
/cloud_registered
```

这些接口“名称存在”与“当前服务器运行时有数据”是两回事，后者尚待验证。

## 10. 官方传感器配置

官方手册当前描述：

```text
SeqID 0：Depth，640×640，FOV 80°，前向，UDP 9999
SeqID 1：前向 RGB，640×480，FOV 90°，UDP 9998
SeqID 2：下视 RGB，640×480，FOV 90°，UDP 9996
```

实际配置文件：

```text
/home/ubuntu/catkin_ws/src/sensor_pkg/Config.json
```

尚未在本轮逐项重新对比。

## 11. Ultralytics 状态

当前环境：

```python
import ultralytics
```

结果：

```text
ModuleNotFoundError
```

仓库存在：

```text
src/object_det/third_party/ultralytics/
```

因此：

```text
环境级 ultralytics 未安装                 [CONFIRMED]
仓库 bundled ultralytics 源码存在          [CONFIRMED]
实际 detection 节点是否正常导入/运行       [PENDING]
```

## 12. 当前运行验证状态

```text
旧服务器官方 Demo 曾跑通                 [HISTORICAL]
当前新服务器完整 Demo                   [PENDING]
当前服务器 Catkin 本轮正式重新编译       [PENDING]
PX4/RflySim 本轮启动                    [PENDING]
MAVROS 在线通信                         [PENDING]
sensor_pkg 运行                         [PENDING]
目标检测运行                            [PENDING]
ArUco 运行                              [PENDING]
Ego-Planner 运行                        [PENDING]
行为树任务运行                          [PENDING]
九格模型加载/任务生成                   [PENDING]
```
