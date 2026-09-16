# 仓库指南（AGENTS.md）

## 1. 项目定位

本仓库用于 **2026“启元杯”无人系统具身智能算法挑战赛——无人机搜索打击应用场景** 的代码开发与调试。

本仓库并不包含完整比赛运行环境。完整仿真环境仍部署在 GPUFree Ubuntu 云服务器中，包括 ROS 1 Noetic、MAVROS、PX4 / RflySim、QGroundControl、九格大模型及其权重、图形显示与 Xterm 环境等。

因此，本地 Codex 的主要职责是：

1. 阅读并理解仓库代码结构；
2. 分析 ROS 节点、topic、launch、配置和模块依赖；
3. 修改代码、配置、行为树、感知或规划逻辑；
4. 为每次修改给出明确的云服务器验证步骤；
5. 根据用户反馈的日志、ROS topic、截图和仿真现象继续定位问题。

**不要假设本地 Windows 环境可以直接完整运行比赛仿真。**

## 2. Git 分支与基线保护

- `main`：稳定版本，不直接进行日常实验性修改；
- `dev`：主要开发分支，Codex 日常修改默认在此分支进行；
- `official-demo-baseline`：官方 Demo 基线标签，作为永久回退点。

未经用户明确要求：

- 不要改写或删除 `official-demo-baseline`；
- 不要强制重写 `main` 历史；
- 不要执行 `git push --force`、危险的 `git reset --hard` 或删除稳定分支/标签。

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

其他重要目录：

- `ego-planner/`：路径规划模块；
- `cv_bridge/`：ROS 图像桥接依赖；
- `behaviortree_cpp/`：行为树依赖；
- `object_det/third_party/`：目标检测相关第三方依赖。

任务 XML 和检测器 YAML 应放在对应包的 `config/` 中；ROS 入口通常放在 `launch/` 或 `scripts/` 中。

不要对第三方依赖目录进行无关的大范围格式化或重构。

以下内容不应提交：

- `build/`
- `devel/`
- `install/`
- 日志
- rosbag
- 模型权重
- Xterm 运行产物

## 4. 构建、测试与开发命令

在 ROS 1 环境下，从工作空间根目录运行：

```bash
./build.sh
source devel/setup.bash
roslaunch mission_pkg basic_test.launch
roslaunch object_det detection.launch
./sim_env_start.sh
```

Catkin 测试：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests
catkin_test_results build
```

## 5. 本地 Codex 与云服务器职责边界

### 本地 Codex 可以完成

- 阅读仓库源码；
- 搜索函数、类、topic、参数、launch 文件；
- 分析 ROS 节点调用关系；
- 修改 Python / C++ / XML / YAML / launch / 配置文件；
- 增加调试日志；
- 设计实验方案；
- 给出构建、运行和验证命令；
- 根据用户反馈继续修改。

### 本地 Codex 不能直接知道

除非用户提供结果，否则不要假设知道：

- 云服务器当前有哪些 ROS 节点正在运行；
- `/mavros/state` 当前值；
- PX4 当前是否进入 OFFBOARD；
- RflySim 当前场景状态；
- GPU 当前资源占用；
- 模型是否成功加载；
- 某个 ROS topic 当前实际输出；
- 无人机实际飞行表现。

这些必须由用户在 GPUFree 云服务器执行命令后反馈。

## 6. 每次修改后的强制验证要求

凡是涉及 ROS 节点、行为树、任务逻辑、目标检测、二维码检测、Ego-Planner、MAVROS、视觉伺服、传感器、launch 文件、关键参数或飞行行为的修改，Codex 不应只给出代码修改。

必须同时给出：

1. 修改了哪些文件；
2. 是否需要重新编译；
3. 云服务器执行的命令；
4. 启动哪个 launch 或脚本；
5. 需要检查哪些 ROS topic / 节点 / 日志；
6. 正常情况下预期看到什么；
7. 异常情况下用户应反馈哪些信息。

推荐流程：

```text
修改内容
↓
服务器 git pull
↓
重新编译
↓
source devel/setup.bash
↓
启动对应节点/完整仿真
↓
检查 topic / 日志 / 窗口
↓
记录实际现象
↓
反馈给 Codex
```

## 7. 推荐的云服务器验证流程

通常在 GPUFree 云服务器上：

```bash
cd ~/catkin_ws
git status
git pull
```

如修改涉及需要重新编译的 ROS 包：

```bash
./build.sh
source devel/setup.bash
```

再根据模块启动：

```bash
roslaunch <package> <file>.launch
```

或启动完整仿真：

```bash
./sim_env_start.sh
```

需要时检查：

```bash
rosnode list
rostopic list
rostopic echo <topic_name>
rostopic hz <topic_name>
rosservice list
```

Codex 应根据具体修改内容给出更精确的命令，而不是机械重复全部命令。

## 8. 调试问题的分析方式

优先按照以下链路定位：

```text
实际现象
↓
对应功能模块
↓
ROS topic / service / action
↓
对应 ROS 节点
↓
对应 package
↓
对应 launch / config
↓
对应源码
↓
最小修改
↓
重新验证
```

例如“气球检测框正常，但无人机不追踪”时，不要直接修改检测网络，应优先检查检测结果 topic、行为树接收、视觉伺服节点、速度/yaw 指令、MAVROS setpoint 和飞控模式。

## 9. 修改原则

- 最小改动；
- 一次只解决一个明确问题；
- 不进行无关重构；
- 不随意改变 ROS topic 名称；
- 不随意改变消息类型；
- 不随意修改坐标系约定；
- 不随意更换第三方依赖版本；
- 不随意删除官方逻辑；
- 较大改造先解释原因和影响范围。

如果问题可能来自环境而不是源码，应明确指出，不要为了“看起来有修改”而强行改代码。

## 10. 编码风格与命名约定

### Python

- 四空格缩进；
- 模块 / 函数使用 `snake_case`；
- 类使用 `PascalCase`；
- ROS 可执行脚本放在 `scripts/`；
- 可导入逻辑放在 `src/<package>/`。

### C++

- 两空格缩进；
- 大括号单独成行；
- 文件 / 函数使用 `snake_case`；
- 类型使用 `PascalCase`。

### launch / config

使用含义明确的小写蛇形命名，例如 `hit_balloon.launch`。仓库目前没有统一格式化工具，因此应保持 diff 聚焦。

## 11. 测试规范

第一方代码目前没有强制覆盖率要求。引入可复用逻辑时，应尽量增加针对性测试。

修改任务逻辑时，应优先运行最接近的测试场景，例如：

```bash
roslaunch mission_pkg basic_test.launch
roslaunch mission_pkg cross_frame_test.launch
```

涉及飞行行为的修改，应记录 ROS topic、节点状态、终端日志、仿真截图、必要时 rosbag 和无人机实际行为。

默认测试流程不应要求真实飞行硬件。

## 12. Commit 与 Pull Request 规范

当前官方基线提交为：

```text
baseline: official competition demo
```

后续提交标题示例：

```text
mission: tune visual servo limits
object_det: adjust balloon confidence threshold
sensor: fix cloud transform frame
planner: tune obstacle clearance
docs: update simulation workflow
```

不要提交生成文件、编译产物、大型模型权重、无意义日志和临时调试文件。

重要提交应说明修改场景、修改的 package / 文件、修改原因、运行命令、验证结果以及对飞行安全或任务逻辑的影响。

## 13. 多对话 / 多 Agent 协作规则

同一 Codex Project 下可能存在多个独立对话。不要假设新对话自动知道其他对话中的所有内容。

长期有效的信息应写入仓库文档，而不是只存在于聊天中。

建议长期维护：

```text
AGENTS.md
docs/PROJECT_CONTEXT.md
docs/ENVIRONMENT.md
docs/DEBUG_LOG.md
docs/TODO.md
```

其中：

- `AGENTS.md`：所有 Agent 都应遵循的总规则；
- `PROJECT_CONTEXT.md`：比赛架构、模块关系与长期背景；
- `ENVIRONMENT.md`：GPUFree、ROS、PX4、RflySim、模型路径等环境信息；
- `DEBUG_LOG.md`：重要问题、修改、测试现象与结果；
- `TODO.md`：当前任务、优先级和待验证事项。

## 14. 信息不足时的处理原则

如果当前仓库没有足够信息支持某个结论：

- 不要猜测；
- 不要虚构 ROS topic；
- 不要虚构 launch 文件；
- 不要虚构参数；
- 不要假设某个节点已经启动。

应明确告诉用户还需要什么，并给出对应检查命令，例如：

```bash
rostopic list
```

或要求用户提供相关 launch 文件、日志或 topic 输出。

## 15. 当前核心开发闭环

```text
Windows 本地 Codex
        ↓
读取 / 修改 dev 分支代码
        ↓
commit + push
        ↓
GitHub Private
        ↓
GPUFree 云服务器 git pull
        ↓
ROS / PX4 / RflySim 仿真验证
        ↓
用户反馈日志 / topic / 截图 / 飞行现象
        ↓
Codex 继续分析和修改
```

所有较大改动都应围绕这一闭环进行。
