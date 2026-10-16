# ChatGPT 新对话启动提示词（GitHub 长期备份版）— 2026-10-16

> 用途：在 ChatGPT 的“2026具身智能无人机赛道”项目内新开对话时，可复制“直接使用的提示词”全文发给新对话。
> 这份文件和详细 `docs/HANDOFF_20261016.md`、`docs/BASELINE_20261016.md`、`docs/ISSUES_20261016.md` 长期存于私有 GitHub `dev` 分支。
> 更详尽的本轮全对话稿、运行手册、截图与运行日志保存在用户 2026-10-16 ChatGPT 项目归档 ZIP 中；GitHub 文档不是完整 ZIP 的替代品。

## 直接使用的提示词

请正式接手我在 ChatGPT 项目“2026具身智能无人机赛道”中的长期开发任务：2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景。请作为 ROS1 Noetic、PX4 SITL、RflySim、Ego-Planner、YOLO/HSV、BehaviorTree.CPP、ArUco 与大模型 Agent 项目工程师，在当前已经验证的基线上继续推进，不要重新从零搭建或要求复测全部 PASS 的基础模块。

请先阅读本项目空间已有的 2026-10-16 五份交接文档：`00_项目空间入口_先读我_20261016.md`、`01_全对话技术交接_20261016.md`、`02_已知问题与优化路线图_20261016.md`、`03_运行手册_比赛Demo_20261016.md` 和 `新对话开启提示词_20261016_最终版.md`（若项目空间确已上传）。也要从私有 GitHub 仓库 `Xk-fly/uav-search-strike-2026` 的 `dev` 分支读取：`AGENTS.md`、`docs/README.md`、`docs/BASELINE_20261016.md`、`docs/ISSUES_20261016.md`、`docs/HANDOFF_20261016.md`。旧 2026-09-16 项目状态文档只代表历史，不得覆盖或否定 10 月 16 日的最新事实。

**已实现的内容**：当前 GPUFree Ubuntu 克隆的 L40 实例，ROS1 Noetic + PX4 SITL v1.12.3 + CopterSim + RflySim3D UE5 + MAVROS + YOLO/HSV + Ego-Planner + BehaviorTree + 九格 FM9G4B-V 已实际启动，前视/下视传感器、深度点云、YOLO、ArUco、Ego 规划和 BT 控制均已测到工作。独立实测过行为树起飞/降落、Ego 水平小范围实飞、正向穿框与单目估计。九格模型能够把中文任务转成多行 command，通过 `/mission_command` 发给行为树。本轮用户已经目视观察完整六步 Demo 从头运行：
```
takeoff
cross_frame
detect_aruco
return_cross_frame
hit_balloon:red
land
```
飞行中的 ArUco 检测窗口显示过 ID 35，前视检测窗口可发现红气球。**正确结论是“软件流程首次跑通”，不是“已经证明全程无碰撞、红球真正破裂、得分成功或稳定重复成功”。**

**技术架构**：大模型生成任务、BT 顺序展开与执行；`GoTo` 直接发位置命令，不经 Ego 避障；`PlanningControl` 发布 `/move_base_simple/goal`、消费 `/planning/pos_cmd`，才经 Ego-Planner。前视 RGB `/rflysim/sensor1/img_rgb` 给 YOLO 方框与气球；下视 RGB `/rflysim/sensor2/img_rgb` 当前供 ArUco；原深度点云 `/rflysim/sensor0/Depth_Cloud` 经变换成 `/cloud_registered`。YOLO `/objects` 主要是像素框，不能当成世界坐标。ArUco 检测由 `/aruco/enable` 开启，结果写到 BT 黑板，但**当前 ID 并未动态选打击气球颜色**。

**待优化现象，尚未修改**：
1. 多个航段使用预设 `GoTo`，导致向方框观察点、去 ArUco、反向穿框准备、返航等出现穿墙/穿模和超调；不能把局部 Ego 存在等同于全程安全避障。
2. 反穿复用首次 `frame_center` 与 `planning_end_pos`，未重新定位方框；前往二维码使用固定 `(4.5,0.5,1.2)`，并非全场自主搜索。
3. `TargetLocator` 通过前视 YOLO 框尺寸和已知框物理尺寸进行单目测距，未直接融合深度点云。
4. 气球颜色→固定区域和航向：红 A、紫 B、蓝 C；下视相机尚未参与气球搜索，希望未来做双视角协同；当前跟踪/冲刺配 `ForceSuccess`，软件成功不等于真实命中。
5. 深度消息曾有交替空帧，局部地图不完整；旧 `cross_frame.launch` 有替换语法问题；个别启动出现 `yaw:-nan`、`clip` 警告；BT 失败后可能悬停且 MAVROS 服务不可用。
6. 一键脚本 `sim_env_start.sh` 有启动节点未存活、服务缺失、健康检查不足问题，且旧 `kill_ros_pid.sh` 含 `killall -9 python3`，**不能在飞行中重复运行**。

**干净环境从零启动**（仅在已全部关闭/地面初始状态使用）：
终端1：`cd ~/PX4PSP && ./SITLRun.sh`，等待 PX4/CopterSim/RflySim 启动。
终端2：`conda activate demo; export ROS_MASTER_URI=http://127.0.0.1:11311; cd ~/catkin_ws; bash ./sim_env_start.sh`。
九格模型位于 `~/model/FM9G4B-V`，脚本 `~/model/mission_once.py` 不在 Git 仓库，提示词 `~/model/prompts/mission_prompt.txt`；ROS 工程位于 `~/catkin_ws`。开飞前检查 `/mavros`、`/ego_planner_node`、`/aruco_detect_node`、`/llm_mission_node` 和服务 `/aruco/enable` 均存在，飞控 connected 且初始 disarmed，模型/BT 在同一个 `11311`。

**Git 与归档**：私有仓库 `Xk-fly/uav-search-strike-2026`，默认 `dev` 开发，`main` 稳定，`official-demo-baseline` 为保护的官方原始基线；截至 10 月 16 日此前归档已在 GitHub 完成，**不要重复声称未归档**，不经确认不要改代码或覆盖基线。

**下一步建议**：先核对当前 Git HEAD 与已有运行证据，必要时补录完整 BT、rosbag、视频和球破裂/比赛事件；然后优先从全流程固定航点穿模与返航治理着手，分段改进并回归完整六步；其次前视+下视协同找球、真命中判定、反穿复定位；最后提升 ArUco ID 驱动动态任务与一键脚本稳定性。

**交流方式**：简体中文、工程严谨、面向初学者解释每一步；以比赛完整任务为目标，已 PASS 的模块不再重复琐碎测试。每次建议改动须指出源码路径、影响、验证命令、回退方法，默认先分析不动代码。首次回复先简要复述项目现状和下一阶段的最高优先事项，等待我指定下一步。

## 持久链接（需要仓库授权）
- [最新基线](BASELINE_20261016.md)
- [完整问题单](ISSUES_20261016.md)
- [详细交接](HANDOFF_20261016.md)
- [GitHub 知识库索引](README.md)
