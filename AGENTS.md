# 仓库指南（AGENTS.md）

> 项目：2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景  
> 本文件规定 ChatGPT/Codex/其他 Agent 在本仓库中的工作方式。项目事实、当前进度和环境参数不要只依赖聊天记忆，优先读取仓库知识库。

## 0. 开始任务前的必读顺序

开始任何重要任务前，按以下顺序恢复项目上下文：

1. `docs/README.md` —— 知识库入口、文档职责与事实优先级；
2. `docs/PROJECT_OVERVIEW.md` —— 项目目标、系统架构与总体工作流；
3. `docs/PROJECT_STATUS.md` —— 当前做到哪里、已经完成什么、下一步是什么；
4. `docs/PROJECT_FACTS.md` —— 已确认事实、路径、版本、关键 topic/文件；
5. `docs/ENVIRONMENT.md` —— GPUFree/ROS/PX4/RflySim/Python/模型等运行环境；
6. 与当前任务相关时再读 `docs/DEBUG_LOG.md` 和 `docs/TODO.md`。

如果聊天内容与上述文档冲突，优先按 `docs/README.md` 定义的事实优先级处理，并指出冲突，不要静默猜测。

## 1. 项目定位

本仓库用于 **2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景** 的代码开发、复现、调试与后续优化。

本仓库并不包含完整比赛运行环境。完整仿真环境部署在 GPUFree Ubuntu 云服务器中，包括 ROS 1 Noetic、MAVROS、PX4/RflySim、QGroundControl、九格大模型及其权重、图形显示/Xterm 等。

本地 Codex 的主要职责：

1. 阅读并理解仓库代码结构；
2. 分析 ROS 节点、topic、launch、配置和模块依赖；
3. 修改代码、配置、行为树、感知或规划逻辑；
4. 为每次修改给出明确的云服务器验证步骤；
5. 根据用户反馈的日志、ROS topic、截图和仿真现象继续定位问题；
6. 当产生新的永久事实、阶段性结论或调试结果时，提醒或更新对应知识库文档。

**不要假设本地 Windows 环境可以直接完整运行比赛仿真。**

## 2. Git 分支与基线保护

- `main`：稳定版本，不直接进行日常实验性修改；
- `dev`：主要开发分支，Codex 日常修改默认在此分支进行；
- `official-demo-baseline`：官方 Demo 基线标签，永久回退点。

未经用户明确要求，不要改写或删除 `official-demo-baseline`，不要强制重写 `main` 历史，不要执行 `git push --force`，不要对未知提交执行破坏性的 `git reset --hard`，不要删除稳定分支或标签。

较大实验优先创建独立分支，例如：

```bash
git switch -c feature/balloon-tracking
```

## 3. 项目结构与模块组织

本仓库是 ROS 1 Catkin 工作空间。第一方代码主要位于 `src/`：

- `mission_pkg/`：行为树任务控制、任务逻辑及相关 C++ 节点；
- `object_det/`：Python 目标检测节点与可复用模块；
- `sensor_pkg/`：传感器处理、点云转换、视觉伺服等脚本；
- `common_msgs/`：共享 ROS 消息定义。

其他重要目录包括 `ego-planner/`、`cv_bridge/`、`behaviortree_cpp/`、`object_det/third_party/`。不要对第三方依赖目录进行无关的大范围格式化或重构。

以下内容不应提交：`build/`、`devel/`、`install/`、日志、rosbag、模型权重、Xterm 运行产物。

## 4. 构建、测试与开发命令

在 ROS 1 环境下，从工作空间根目录运行：

```bash
./build.sh
source devel/setup.bash
```

按模块使用对应 `roslaunch`；完整仿真入口为：

```bash
./sim_env_start.sh
```

Catkin 测试：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests
catkin_test_results build
```

**注意：不要把 `basic_test.launch` 当成无副作用的“启动检查”。其行为树会执行起飞/移动/目标定位等任务，必须在 PX4/RflySim/MAVROS 等前置条件满足后再运行。**

## 5. 本地 Codex 与云服务器职责边界

本地 Codex 可以阅读源码、分析调用关系、修改 Python/C++/XML/YAML/launch、增加日志、设计实验、生成验证命令。

除非用户提供实际结果，否则不要假设知道云服务器当前 ROS 节点、`/mavros/state`、PX4 OFFBOARD、RflySim 状态、GPU 分配、模型加载、topic 实际输出或无人机真实飞行表现。

## 6. 每次修改后的强制验证要求

凡涉及 ROS 节点、行为树、任务逻辑、检测、Ego-Planner、MAVROS、视觉伺服、传感器、launch、关键参数或飞行行为，必须同时给出：

1. 修改文件；
2. 是否重新编译；
3. 云服务器执行命令；
4. 启动哪个 launch/脚本；
5. 检查哪些 topic/节点/日志；
6. 正常预期；
7. 异常时用户应反馈什么。

推荐闭环：

```text
本地修改
→ commit + push
→ 云服务器 git pull
→ 必要时重新编译
→ source devel/setup.bash
→ 启动对应模块/完整仿真
→ 检查 topic / 日志 / 窗口
→ 记录实际现象
→ 更新知识库（如适用）
→ 继续分析
```

## 7. 调试与修改原则

优先按：

```text
实际现象 → 功能模块 → topic/service → ROS 节点 → package → launch/config → 源码 → 最小修改 → 重新验证
```

原则：最小改动、一次解决一个问题、不随意改 topic/消息类型/坐标系/第三方版本、不随意删除官方逻辑。信息不足时不要猜测、不要虚构参数或节点状态。

## 8. Commit 规范

提交标题示例：

```text
mission: tune visual servo limits
object_det: adjust balloon confidence threshold
sensor: fix cloud transform frame
planner: tune obstacle clearance
docs: update project status
```

## 9. 多对话 / 多 Agent 协作

不要假设新对话自动知道其他对话内容。跨对话信息必须沉淀到仓库文档。

```text
项目目标/架构变化          → PROJECT_OVERVIEW.md
阶段完成/当前状态变化      → PROJECT_STATUS.md
确认新版本/路径/参数       → PROJECT_FACTS.md 或 ENVIRONMENT.md
一次真实调试/故障处理      → DEBUG_LOG.md
任务完成/新增任务          → TODO.md
Agent 工作规则变化         → AGENTS.md
```

GitHub Private 仓库中的知识库是长期事实来源；聊天记忆只是辅助。

## 10. 文档与分析偏好（用户确认，2026-10-28）

- 分析文档和报告保持简洁，只记录影响比赛流程的结论、必要证据、改动及验证步骤，避免无关内容和重复归档。
- 默认不生成轨迹图、速度图或其他非必要图表；可在内部完成分析，仅在用户明确要求或确有必要时输出。
- 优先完成并验证整个比赛流程，减少无关产物、耗时和额外 token 消耗。
