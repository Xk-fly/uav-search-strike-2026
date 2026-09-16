# 运行环境说明（ENVIRONMENT.md）

> 更新时间：2026-09-16  
> 用途：记录 GPUFree 云服务器上的真实运行环境、关键路径、依赖、启动入口与验证规则。  
> 原则：以实际服务器检查结果为准；未验证项不得猜测。

## 1. 当前开发架构

```text
Windows 本地 Codex
D:\UAV_Competition\uav-search-strike-2026
        ↓ commit / push
GitHub Private
Xk-fly/uav-search-strike-2026
        ↓ git pull
GPUFree Ubuntu 云服务器
/home/ubuntu/catkin_ws
        ↓
ROS / PX4 / RflySim / MAVROS / 感知 / 行为树 / 规划 / 大模型
```

- 主要开发分支：`dev`
- 稳定分支：`main`
- 官方基线标签：`official-demo-baseline`

## 2. 系统环境

```text
OS：Ubuntu 20.04.6 LTS (Focal Fossa)
运行用户：ubuntu
ROS：ROS 1 Noetic
ROS_DISTRO=noetic
ROS_MASTER_URI=http://localhost:11311
```

加载 Catkin 前：

```text
ROS_PACKAGE_PATH=/opt/ros/noetic/share
```

执行：

```bash
source ~/catkin_ws/devel/setup.bash
```

后：

```text
ROS_PACKAGE_PATH=/home/ubuntu/catkin_ws/src:/opt/ros/noetic/share
```

已确认可发现：

```text
mission_pkg  -> /home/ubuntu/catkin_ws/src/mission_pkg
object_det   -> /home/ubuntu/catkin_ws/src/object_det
sensor_pkg   -> /home/ubuntu/catkin_ws/src/sensor_pkg
mavros       -> /opt/ros/noetic/share/mavros
```

## 3. Python / Conda / CUDA

```text
Conda 环境：demo
Conda 路径：/opt/conda/envs/demo
Python：3.10.19
Python 路径：/opt/conda/envs/demo/bin/python3
PyTorch：2.3.1+cu121
torch CUDA runtime：12.1
CUDA Toolkit：12.8
nvcc：V12.8.93
```

当前服务器为“无卡模式”：

```text
torch.cuda.is_available() = False
device_count = 0
/dev/nvidia* 不存在
```

正式大模型/目标检测性能验证时，应以有 GPU 的实例重新运行 `nvidia-smi` 和 PyTorch CUDA 检查。

## 4. MAVROS

```text
mavros      = 1.20.1
mavros_msgs = 1.20.1
```

关键话题：

```text
/mavros/state
/mavros/local_position/odom
/mavros/setpoint_raw/local
```

关键服务：

```text
/mavros/set_mode
/mavros/cmd/arming
```

## 5. PX4 / RflySim

根目录：

```text
/home/ubuntu/PX4PSP
```

已确认存在：

```text
CopterSim/
Firmware/
QGroundControl/
RflySimAPIs/
RflySimUE5/
SITLRun.sh
```

PX4：

```text
版本：v1.12.3
Commit：2e8918da66
```

RflySim SDK 真实路径：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
```

当前 shell 的 `PYTHONPATH` 已确认包含 SDK 根目录及 `comm/ctrl/ue/vision/phm/swarm/test/word` 等子目录。

### RflySim SDK 路径修复记录

旧 `sim_env_start.sh` 曾使用：

```bash
RFLYSIM_SDK="/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK"
```

这是旧 WSL 路径，不适用于当前 GPUFree Linux 服务器。

已在 `dev` 修复为：

```bash
RFLYSIM_SDK="${HOME}/PX4PSP/RflySimAPIs/RflySimSDK"
```

服务器 `git pull` 后已经验证：

```text
RflySim SDK path OK
```

因此该问题状态为：**已修复并验证路径存在**。

## 6. 九格大模型

```text
模型根目录：/home/ubuntu/model
模型目录：/home/ubuntu/model/FM9G4B-V
总大小：约 8.5G
主权重：/home/ubuntu/model/FM9G4B-V/pytorch_model.bin
主权重大小：9,030,068,428 bytes
```

主要脚本：

```text
/home/ubuntu/model/mission_once.py
/home/ubuntu/model/test_rflysim.py
```

提示词：

```text
/home/ubuntu/model/prompts/mission_prompt.txt
/home/ubuntu/model/prompts/basic_cn.txt
/home/ubuntu/model/prompts/system_promt.txt
```

当前服务器尚未在本轮完成 GPU 模式下的模型加载验证。

## 7. Python 关键库

```text
cv2 = 4.6.0
numpy = 1.23.5
rospy = installed
ultralytics（环境级 import）= 未安装
```

仓库存在：

```text
src/object_det/third_party/ultralytics/
```

因此当前只能确认“仓库内置 Ultralytics 源码存在”，实际检测节点运行方式需要后续启动验证。

## 8. Catkin 工作空间

```text
/home/ubuntu/catkin_ws
```

构建：

```bash
cd ~/catkin_ws
./build.sh
source devel/setup.bash
```

`build.sh` 当前实际执行：

```bash
catkin_make
```

## 9. 已确认 launch 文件

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

## 10. 官方仿真与完整 Demo 启动入口

RflySim/PX4：

```bash
cd ~/PX4PSP
./SITLRun.sh
```

官方手册要求等待 CopterSim 出现类似：

```text
GPS 3D fixed ...
```

再进入后续 Demo。

完整代码启动入口：

```bash
cd ~/catkin_ws
./sim_env_start.sh
```

当前 `sim_env_start.sh` 会启动/尝试启动：

```text
roscore
九格大模型 mission_once.py
object_det detection_color.launch
sensor_pkg/main.py
AgentContestMap.py
cloud_transform
ArUco
Ego-Planner
llm_mission
setpoint monitor
ROS node resource monitor
```

**重要：当前 dev 脚本中的 MAVROS 启动行处于注释状态。** 因此第一次逐模块复现时应单独确认 MAVROS 的正确启动时机和 UDP FCU URL，不能假设 `sim_env_start.sh` 会自动启动 MAVROS。

脚本中定义：

```text
MAVROS_FCU_URL="udp://:20101@"
```

但实际通信必须以当前服务器运行验证为准。

## 11. 标准验证流程

同步代码：

```bash
cd ~/catkin_ws
git status
git branch --show-current
git pull
```

构建/加载：

```bash
./build.sh
source devel/setup.bash
```

模块验证时根据具体目标使用：

```bash
roslaunch <package> <launch_file>
```

运行完整 Demo 前先保证 PX4/RflySim/MAVROS 等前置环境已确认正常。

常用检查：

```bash
rosnode list
rostopic list
rostopic echo -n 1 <topic>
rostopic hz <topic>
rosservice list
```

## 12. 当前环境验证边界

已确认：

```text
系统版本
ROS/Catkin 环境
Python/Conda/CUDA 软件环境
MAVROS 安装版本
PX4 版本/目录
RflySim SDK 路径
模型目录
关键 launch 文件
Git 同步
```

尚未确认：

```text
本次新服务器 Catkin 全量重新编译
PX4/RflySim 当前实例正常启动
MAVROS 与 PX4 当前实例通信
各传感器实时 topic
目标检测实际运行
ArUco 实际运行
Ego-Planner 实际运行
行为树实际任务
九格模型当前实例加载
完整 Demo
```
