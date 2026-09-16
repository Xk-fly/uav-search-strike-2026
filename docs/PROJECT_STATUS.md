# 项目状态与对话交接（PROJECT_STATUS.md）

> 更新时间：2026-09-16  
> 用途：记录本项目截至当前的完整工作进度，供新的 ChatGPT/Codex 对话直接接手。  
> 重要说明：旧服务器曾经跑通过官方 Demo；**当前这台新 GPUFree 服务器尚未完成完整 Demo 验证**。不要把历史成功误认为当前环境已验证。

## 1. 当前交接状态

目前已经完成 Git/GitHub/Codex 开发闭环和主要环境核验，并修复了一个已确认的 RflySim SDK 路径问题。

当前暂停点：

```text
即将开始“当前新服务器上的官方 Demo 按模块逐步验证”
```

计划的下一步验证顺序：

```text
Catkin 编译
→ ROS Core
→ PX4 / RflySim
→ MAVROS
→ 传感器
→ 点云
→ 目标检测
→ ArUco
→ Ego-Planner
→ 行为树
→ 九格大模型
→ 完整 sim_env_start.sh
```

目前还没有执行这一轮正式模块验证。

## 2. 本次对话已完成：SSH/ROS 环境定位

最初 VS Code Remote SSH 中：

```text
root@gpufree-container
rosversion -d → <unknown>
```

官方远程桌面终端：

```text
ubuntu@gpufree-container
rosversion -d → noetic
```

切换：

```bash
su - ubuntu
```

后与官方环境保持一致。后续比赛 ROS 调试统一以 `ubuntu` 用户为主要运行身份。

## 3. 官方 Demo Git 基线建立

工作空间：

```text
/home/ubuntu/catkin_ws
```

最初检查：

```text
NOT_GIT_REPO
嵌套 Git：src/cv_bridge/.git
项目约 709M
排除 build/devel 后无 >50M 普通源码文件
```

已完成：

### 原始源码物理备份

```text
/home/ubuntu/catkin_ws_official_baseline_20260915.tar.gz
约 49M
```

### cv_bridge Git 元数据备份

```text
/home/ubuntu/cv_bridge_git_metadata_backup_20260915
```

### Git 初始化与 baseline

```text
2c0fe57  baseline: official competition demo
main
official-demo-baseline
```

`.gitignore` 已排除 build/devel/install/log/logs、Python cache、rosbag、模型权重、编辑器文件和 Xterm 运行产物。

## 4. GitHub Private 仓库

```text
Xk-fly/uav-search-strike-2026
Visibility: Private
```

服务器使用专用 Deploy Key，并通过 GitHub SSH 443 访问。

SSH 配置：

```text
Host github-uav
HostName ssh.github.com
User git
Port 443
IdentityFile ~/.ssh/id_ed25519_uav2026
IdentitiesOnly yes
```

认证已成功：

```text
Authentication succeeded (publickey)
```

`main` 与 `official-demo-baseline` 已成功 push。

## 5. dev 分支与 Windows 本地开发

服务器从 baseline 创建：

```text
dev
```

Windows 本地仓库：

```text
D:\UAV_Competition\uav-search-strike-2026
```

本地：

```text
dev  → origin/dev
main → origin/main
```

Codex Desktop 使用该本地仓库作为项目目录。

## 6. Codex 项目规则与知识库

已建立并完善 `AGENTS.md`，明确：

```text
本地 Codex 负责代码分析/修改
完整运行验证在 GPUFree 完成
默认在 dev 开发
保护 main 与 official-demo-baseline
修改飞行/感知/规划逻辑后必须给出服务器验证步骤
信息不足时不得猜测
```

已建立：

```text
docs/ENVIRONMENT.md
```

本次进一步建立：

```text
docs/README.md
docs/PROJECT_OVERVIEW.md
docs/PROJECT_STATUS.md
docs/PROJECT_FACTS.md
docs/DEBUG_LOG.md
docs/TODO.md
```

## 7. 当前服务器环境核验结果

已实际确认：

```text
Ubuntu 20.04.6 LTS
用户 ubuntu
ROS 1 Noetic
MAVROS 1.20.1
Python 3.10.19
Conda 环境 demo
PyTorch 2.3.1+cu121
CUDA Toolkit 12.8
PX4 v1.12.3
PX4 commit 2e8918da66
```

当前实例为“无卡模式”：

```text
torch.cuda.is_available() = False
device_count = 0
/dev/nvidia* 不存在
```

这表示当前未分配 GPU，不等价于永久无 GPU。

加载：

```bash
source ~/catkin_ws/devel/setup.bash
```

后：

```text
ROS_PACKAGE_PATH=/home/ubuntu/catkin_ws/src:/opt/ros/noetic/share
```

以下包可被 `rospack` 正确发现：

```text
mission_pkg
object_det
sensor_pkg
mavros
```

## 8. 九格模型

```text
/home/ubuntu/model
/home/ubuntu/model/FM9G4B-V
```

目录总大小约 `8.5G`。

主权重：

```text
/home/ubuntu/model/FM9G4B-V/pytorch_model.bin
9,030,068,428 bytes
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

## 9. RflySim SDK 路径问题与修复

真实路径：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
```

旧 `sim_env_start.sh` 使用 WSL 路径：

```text
/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK
```

已修改为：

```bash
RFLYSIM_SDK="${HOME}/PX4PSP/RflySimAPIs/RflySimSDK"
```

提交：

```text
3747c37  env: fix RflySim SDK path
```

服务器 `git pull` 后已验证：

```text
RflySim SDK path OK
```

随后服务器已快进到：

```text
6032921
```

当时 `dev` 工作区 clean。

## 10. Python / Ultralytics 现状

已确认：

```text
cv2 = 4.6.0
numpy = 1.23.5
rospy = installed
```

当前 `demo` 环境直接：

```python
import ultralytics
```

返回：

```text
ModuleNotFoundError
```

但仓库存在：

```text
src/object_det/third_party/ultralytics/
```

因此当前只能确认“内置源码存在”，实际检测节点的运行时导入方式尚未通过当前服务器启动验证。

## 11. 已确认 launch 入口

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

重要：`basic_test.launch` 不是纯启动测试，其行为树会执行：

```text
Takeoff → GoTo → Delay → TargetLocator
```

因此不能在 PX4/RflySim/MAVROS 前置条件缺失时直接使用它做“安全检查”。

## 12. 下一位 Agent 接手时不要误判

明确区分：

```text
旧服务器：历史上曾跑通过官方 Demo
当前新服务器：尚未完成完整 Demo 验证
```

当前已经完成：

```text
版本管理
AI 开发闭环
环境核验
RflySim SDK 路径修复
项目知识库建立
```

尚未完成：

```text
当前服务器官方 baseline 的逐模块运行验证
```

下一步从 Catkin 编译开始，不要直接做大范围算法优化。
