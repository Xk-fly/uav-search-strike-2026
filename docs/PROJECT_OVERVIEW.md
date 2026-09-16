# 项目总览（PROJECT_OVERVIEW.md）

> 更新时间：2026-09-16  
> 项目：2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景  
> 作用：让新开的 ChatGPT/Codex 对话快速理解“这个项目要做什么、系统怎么组成、工作如何推进”。

## 1. 项目要做什么

本项目的直接目标是复现并掌握赛事官方 Demo，在当前 GPUFree 云服务器上建立一套可重复、可回退、可由 AI 协助迭代的无人机搜索打击仿真开发环境；在官方 Demo 可靠运行基础上，再逐步优化感知、任务决策、路径规划、视觉伺服和飞行执行等模块。

官方总体闭环：

```text
自然语言指令
→ 场景理解 / 大模型任务生成
→ 行为树决策
→ 感知与路径规划
→ MAVROS / PX4 控制
→ 无人机执行
→ 任务结果反馈
```

当前第一优先级不是立即改算法，而是先确保当前新服务器上的官方基线能够按模块完整跑通。

## 2. 当前系统架构

### 大模型任务层

九格模型位于：

```text
/home/ubuntu/model/FM9G4B-V
```

自然语言任务经模型生成结构化任务指令，并通过 `/mission_command` 交给行为树。

### 行为树 / 任务层

主要代码：

```text
src/mission_pkg/
```

负责起飞、航点、穿框、二维码识别、气球打击、返航/降落等行为，并与规划、检测和 MAVROS 交互。

### 感知层

主要代码：

```text
src/object_det/
src/sensor_pkg/
```

关键感知话题：

```text
/objects
/rflysim/sensor0/Depth_Cloud
/rflysim/sensor1/image_rgb
/rflysim/sensor2/image_rgb
/cloud_registered
```

### 路径规划层

主要模块：

```text
src/ego-planner/
```

关键接口：

```text
/move_base_simple/goal
/planning/pos_cmd
```

### 飞行控制层

ROS 通过 MAVROS 与 PX4 仿真飞控通信。关键接口：

```text
/mavros/state
/mavros/local_position/odom
/mavros/setpoint_raw/local
/mavros/set_mode
/mavros/cmd/arming
```

### 仿真层

```text
/home/ubuntu/PX4PSP
```

官方仿真启动入口：

```bash
cd ~/PX4PSP
./SITLRun.sh
```

完整代码入口：

```bash
cd ~/catkin_ws
./sim_env_start.sh
```

## 3. 代码与运行环境分离

```text
Windows 本地 Codex
D:\UAV_Competition\uav-search-strike-2026
        ↓ commit / push
GitHub Private
Xk-fly/uav-search-strike-2026
        ↓ git pull
GPUFree Ubuntu
/home/ubuntu/catkin_ws
        ↓
ROS / PX4 / RflySim / MAVROS / 模型 / GUI 实际验证
```

GitHub 只管理代码、配置和知识库，不包含完整仿真环境、编译产物和大型模型权重。

## 4. Git / AI 开发策略

```text
main                    稳定版本
dev                     日常开发
official-demo-baseline  官方原始 Demo 永久基线标签
```

项目使用 `AGENTS.md + docs/` 作为跨 ChatGPT/Codex 对话的长期记忆。

## 5. 当前阶段总体工作流

### 阶段 A：开发基础设施 —— 已基本完成

```text
官方 Demo 备份
→ Git 初始化
→ .gitignore
→ baseline commit/tag
→ GitHub Private
→ main/dev
→ Windows clone
→ Codex Project
→ AGENTS.md
→ 环境知识库
```

### 阶段 B：当前新服务器基线复现 —— 即将开始

```text
Catkin 编译
→ ROS Core
→ PX4 / RflySim
→ MAVROS
→ sensor_pkg
→ 点云转换
→ 目标检测
→ ArUco
→ Ego-Planner
→ 行为树
→ 九格大模型
→ 完整 sim_env_start.sh
```

### 阶段 C：比赛功能优化 —— 尚未开始

只有当前服务器官方基线稳定跑通后，再针对感知、行为树、路径规划、视觉伺服、大模型、系统时序和稳定性等专项优化。

具体优化指标应以赛事规则和后续实际基线测试结果为依据，当前不提前虚构评分指标。

## 6. 基线复现成功的基本判据

```text
代码可稳定编译
ROS 包可正常发现
PX4/RflySim 可正常启动
MAVROS 通信正常
传感器话题存在且频率合理
目标检测可运行
ArUco 可运行
Ego-Planner 可运行
行为树能够接收和执行任务
九格模型能够加载并发布任务
完整 Demo 能按预期执行官方测试任务
```

## 7. 跨对话恢复

任何新开的 ChatGPT/Codex 对话应先读取：

```text
AGENTS.md
docs/README.md
docs/PROJECT_OVERVIEW.md
docs/PROJECT_STATUS.md
docs/PROJECT_FACTS.md
docs/ENVIRONMENT.md
```

故障排查时再读取 `DEBUG_LOG.md` 和 `TODO.md`。
