# 运行环境说明（ENVIRONMENT.md）

> 项目：2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景  
> 用途：记录 GPUFree 云服务器上的真实运行环境、关键路径、依赖、启动入口与验证规则。  
> 原则：以实际服务器检查结果为准；未验证项不得猜测。

---

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

- 本地主要开发分支：`dev`
- 稳定分支：`main`
- 官方基线标签：`official-demo-baseline`

---

## 2. 系统环境

### 操作系统

```text
Ubuntu 20.04.6 LTS (Focal Fossa)
```

### 用户

```text
ubuntu
```

### ROS

```text
ROS 1 Noetic
ROS_DISTRO=noetic
ROS_MASTER_URI=http://localhost:11311
```

加载 Catkin 工作空间前：

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

---

## 3. Python / Conda / CUDA

### Conda

```text
当前环境：demo
路径：/opt/conda/envs/demo
```

### Python

```text
Python 3.10.19
/opt/conda/envs/demo/bin/python3
```

### PyTorch

```text
torch = 2.3.1+cu121
torch CUDA runtime = 12.1
```

### CUDA Toolkit

```text
CUDA 12.8
nvcc V12.8.93
```

### 当前 GPU 状态

当前服务器以“无卡模式”启动：

```text
torch.cuda.is_available() = False
device_count = 0
/dev/nvidia* 不存在
```

因此当前 `nvidia-smi` 不显示 GPU 信息属于预期现象。分配 GPU 后应重新检查。

---

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

---

## 5. PX4 / RflySim

根目录：

```text
/home/ubuntu/PX4PSP
```

主要内容：

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

RflySim SDK 实际路径：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
```

当前 shell 的 `PYTHONPATH` 已包含：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/comm
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/ctrl
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/ue
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/vision
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/phm
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/swarm
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/test
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK/word
```

### 已发现并需要修正的问题

当前仓库 `sim_env_start.sh` 中存在旧的 WSL 路径：

```bash
RFLYSIM_SDK="/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK"
```

这与当前 GPUFree Linux 服务器实际路径不一致。

建议改为：

```bash
RFLYSIM_SDK="${HOME}/PX4PSP/RflySimAPIs/RflySimSDK"
```

这样在当前 `ubuntu` 用户下会解析为：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
```

这是当前环境需要修正并纳入版本管理的配置问题。

---

## 6. 九格大模型

目录：

```text
/home/ubuntu/model
```

模型目录：

```text
/home/ubuntu/model/FM9G4B-V
```

模型总大小：

```text
约 8.5 GB
```

主要权重：

```text
/home/ubuntu/model/FM9G4B-V/pytorch_model.bin
约 9.03 GB
```

主要文件：

```text
/home/ubuntu/model/mission_once.py
/home/ubuntu/model/test_rflysim.py
/home/ubuntu/model/prompts/mission_prompt.txt
/home/ubuntu/model/prompts/basic_cn.txt
/home/ubuntu/model/prompts/system_promt.txt
```

---

## 7. Python 关键库

当前 `demo` 环境：

```text
opencv-python / cv2 = 4.6.0
numpy               = 1.23.5
rospy               = installed
ultralytics pip包   = 未安装
```

仓库中自带：

```text
src/object_det/third_party/ultralytics/
```

因此不能仅根据 `pip import ultralytics` 失败判断目标检测不可用。目标检测模块应继续根据实际源码导入路径和启动结果判断。

---

## 8. Catkin 工作空间

```text
/home/ubuntu/catkin_ws
```

主要内容：

```text
build/
devel/
src/
build.sh
kill_ros_pid.sh
sim_env_start.sh
自然语言命令测试.md
```

构建：

```bash
cd ~/catkin_ws
./build.sh
source devel/setup.bash
```

其中 `build.sh` 当前实际执行 `catkin_make`。

---

## 9. 已确认 launch 文件

### mission_pkg

```text
aruco_detect.launch
basic_test.launch
cross_frame.launch
cross_frame_test.launch
full_competition.launch
hit_balloon.launch
llm_mission.launch
```

### object_det

```text
detection.launch
detection_color.launch
hsv_detection.launch
```

### sensor_pkg

```text
cloud_transform.launch
visual_servo_balloon.launch
```

---

## 10. 完整 Demo 启动

主要入口：

```bash
cd ~/catkin_ws
./sim_env_start.sh
```

当前脚本会启动/尝试启动：

```text
roscore
九格大模型
YOLO/颜色检测
sensor_pkg/main.py
AgentContestMap.py
cloud_transform
ArUco
Ego-Planner
llm_mission
setpoint monitor
ROS node resource monitor
```

正式运行前，应先确保 PX4 / RflySim 相关组件已正常启动。

---

## 11. Codex 修改后的标准验证流程

```bash
cd ~/catkin_ws
git status
git branch --show-current
git pull
```

如涉及 C++、message、CMakeLists、package.xml 等：

```bash
./build.sh
source devel/setup.bash
```

然后按修改模块启动：

```bash
roslaunch <package> <launch_file>
```

或运行完整 Demo：

```bash
./sim_env_start.sh
```

必要时检查：

```bash
rosnode list
rostopic list
rostopic echo -n 1 <topic>
rostopic hz <topic>
rosservice list
```

---

## 12. 调试反馈要求

涉及飞行行为、感知、规划、行为树或 MAVROS 的修改后，应记录：

- 修改文件；
- 是否重新编译；
- 实际运行命令；
- ROS topic / service 输出；
- 终端日志；
- 仿真窗口截图；
- 无人机实际行为；
- 修改前后差异。

---

## 13. 重要注意事项

1. 不要把模型权重上传到 GitHub。
2. 不要提交 `build/`、`devel/`、日志、bag、Xterm 运行产物。
3. 当前服务器尚未在本次新实例上完整跑通 Demo，因此所有“可运行”结论应以本服务器实际验证为准。
4. `sim_env_start.sh` 的 RflySim SDK 路径需从旧 WSL 路径修正为当前 Linux 路径。
5. 如果官方手册、旧服务器经验和当前服务器实际环境冲突，以当前服务器实际验证结果为准，并保留差异记录。
