# 项目状态与对话交接（PROJECT_STATUS.md）

## 2026-10-29 搜索角速度恢复与自动轻量录包（待云端验证）

- 用户实测反馈：到 A 区后0.5rad/s搜索偏快，转动时漂移穿模。本轮仅将未见球时搜索恢复为 `search_yaw_rate=0.3rad/s`，保留120秒打击时限；接近、冲刺、制动和返航逻辑未改。尚未确认三色制动与Ego返航实飞效果。
- 根目录新增 `record_flight_bag.sh`，`sim_env_start.sh` 在 roscore 就绪后自动启动。默认保存到工作空间 `Bag/`（云端 `/home/ubuntu/catkin_ws/Bag`），LZ4压缩、约64MB分卷；记录真值/MAVROS里程计、飞控状态与指令、检测、规划、任务消息及ROS日志，不录图像或点云。分卷仅限制单文件大小，不限制总录制量。
- 云端：`git pull --ff-only && ./build.sh`，成功后 `source devel/setup.bash`，按原前置流程启动 `./sim_env_start.sh`。检查搜索日志 `yaw_rate=0.30`、`rosnode info /flight_bag_recorder` 及 `ls -lh Bag/`；预期转向减慢，120秒时限保留，录制文件持续增长。
- 结束时在启动终端按Ctrl+C，脚本先结束录包再关闭ROS；也可单独 `rosnode kill /flight_bag_recorder`，仿真继续运行。确认 `.bag.active` 变成 `.bag` 后下载本次所有分卷；重新启动前先正常停止旧脚本，避免旧录包被原有强制清理截断。独立录制：`bash ./record_flight_bag.sh`。
- 本地仅验证脚本语法/模拟启动退出和参数回归，不能代替ROS编译与实飞；异常反馈搜索日志、漂移阶段、录包报错和本次bag。

## 2026-10-29 当前进展：三色共用制动、Ego 返航（待云端编译实飞）

- 用户最新范围：不处理 ArUco 重复识别，不做下视复核/全局建图/换观察点；按经验将“曾看到目标后连续消失”作为命中判断，不增加多级结果分类。
- VisualServo：曾跟踪目标后消失0.25秒即结束，包括冲刺阶段；短暂消失时停止前进，朝最后目标方向转动。未见球时原地搜索0.5rad/s，置信度0.5至0.8的同色候选转向上限0.3rad/s；跟踪门槛0.8→0.5，任务打击时限50→120秒。搜索转向变化为本次用户明确要求，不据此宣称赛事已认可。
- 共用制动复用 PlanningControl 的 `stop_only`：捕获固定位置，MAVROS速度模长≤0.12m/s连续0.5秒才完成，10秒未停稳则保持并失败。该门槛是完成判据，不是飞行速度设定。
- 返航：停稳→GoTo原地面向Home→等待前视更新0.75秒→Ego到Home上方1m→Ego到降落准备点0.5m→原地恢复航向→Land。返航开启规划偏航和新目标匹配，避免沿用上一段轨迹；规划失败/超时不回退直线GoTo。LLM和五份静态XML同步。
- 官方接近/冲刺速度、冲刺最长1.2秒、Ego速度/加速度、膨胀0.30与dist0=0.15、穿框和ArUco逻辑保留；未修改sensor_pkg或Ego地图代码。当前仍为局部地图，尚不承诺三色返航无碰撞。
- 修改文件：`visual_servo.cpp/.h`、`planning_control.cpp/.h`、`goto.cpp`、`common.hpp`、`target_tracker.cpp`、`bt_ros.cpp`、`llm_mission.launch`、五份任务XML。本地已做替身ROS/BT的C++语法检查、实际控制函数的12个编译期行为用例、三色动态/静态XML及参数回归；不能代替ROS编译和实飞。
- 云端：在本分支执行 `git pull --ff-only && ./build.sh`，成功后 `source devel/setup.bash`；任务停止后，按已有前置流程重新启动 `./sim_env_start.sh`。一次一条六步任务，依次红/蓝/紫。
- 预期日志：`Target disappeared after tracking`或`Sprint completed`→`BRAKE: holding fixed position`→`BRAKE complete: actual stop verified`→`ReturnHome`导航完成→降落。搜索耗尽也先制动再返航。检查 `/objects`、`/mavros/local_position/odom`、`/planning/pos_cmd`、`/move_base_simple/goal`、`/mavros/setpoint_raw/local`；异常反馈颜色/阶段、Mission日志与同目录bag，重点记录停稳耗时、是否穿墙及是否卡在新目标等待。

以下为历史记录。

## 2026-10-28 当前进展：停稳补丁已云端编译，飞行测试被 LLM 指令提取问题阻断

- 当前分支：`codex/restore-official-flight`，基于 `88f0098`。该版本单次实测赴 A 无碰撞，反穿框右偏约0.21m且轻微穿模，证据见 `DEBUG_LOG.md`。
- 用户确认 `9e42a24` 云端编译成功；首次启动未起飞。`record.txt` 和 Mission 截图确认任务节点在线且收到消息，但消息只有 `python`/解释文字，生成空 Sequence 后报错，未进入 Takeoff。MAVROS connected=True、armed=False、AUTO.LOITER；停稳补丁的实际飞行效果尚未验证。下一步核对仓库外 `~/model/mission_once.py` 提取逻辑，并以纯六步命令复测任务入口。
- 本轮修改：`planning_control.cpp/.h` 增加可选的接管前、到达后停稳门控；`bt_ros.cpp` 和四份 `full_competition*.xml` 仅在反穿框启用。原有航点、两段机体系 GoTo、官方速度与气球逻辑保留。
- 停稳判据：MAVROS 三维速度模长≤0.12m/s，连续0.5s；0.12是完成判据，不是飞行限速。重复/过期/无效数据不能完成门控；允许最多0.1s的消息时钟领先，用接收时间计算持续时长。10s超时则保持位置并返回FAILURE。
- Ego 膨胀0.30m、`dist0=0.15m` 保持；规划和点云继续使用既有真值坐标系。两路里程计坐标轴不同，不能直接替换整个 odom 话题。Ego 估速、全程 Ego 穿框、地图保留和实际跟踪误差保护仍待后续处理。
- 本地已检查门控函数语法（消息/时间/锁使用替身）及静态/动态 BT XML、源码差异；用户随后确认云端编译成功，实际飞行验证仍待完成。后续涉及 C++ 的修改均须云端重新编译并重启任务节点。
- 云端：确认处于本分支，依次执行 `git pull --ff-only`、`./build.sh`；编译成功后 `source devel/setup.bash`，按已跑通的 PX4/RflySim/MAVROS 前置流程启动 `./sim_env_start.sh`。
- 验收：日志依次出现 `Holding before Ego takeover`、`Actual stop verified`、`Holding endpoint` 和导航完成；反穿前后确有持续悬停，无 `Stop gate timed out`。检查 `/mavros/local_position/odom`、`/planning/pos_cmd`、`/mavros/setpoint_raw/local`，回归首次穿框、ArUco、反穿框与赴 A。仍右偏/擦框、超时或卡住时，反馈阶段、任务日志及包含点云/膨胀地图的 bag；本补丁不代表碰撞已解决。
- 用户文档偏好见 `AGENTS.md` 第 10 节。上次实测摘要见 [OFFICIAL_FLIGHT_ANALYSIS_20261028.md](OFFICIAL_FLIGHT_ANALYSIS_20261028.md)，恢复过程见 [OFFICIAL_FLIGHT_RESTORE_20261028.md](OFFICIAL_FLIGHT_RESTORE_20261028.md)。以下为历史记录。

> **2026-10-16 最新状态提示：本文件主体是 2026-09-16 的历史交接快照。当前 L40 克隆实例上已观察到官方 Demo 六步完整执行。最新验收请优先阅读 [BASELINE_20261016.md](BASELINE_20261016.md)、[HANDOFF_20261016.md](HANDOFF_20261016.md)；多次穿模、气球真实命中未验证，见 [ISSUES_20261016.md](ISSUES_20261016.md)。**


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
