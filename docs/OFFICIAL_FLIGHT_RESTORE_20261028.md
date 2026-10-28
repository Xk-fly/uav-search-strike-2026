# 2026-10-28｜官方飞行逻辑恢复候选与速度核查

## 1. 结论与范围

用户转述赛事方要求“整个仿真飞行速度”不能修改。按此范围，不能只把接近和冲刺的两个参数改回去就宣称恢复完成。

准备独立分支 `codex/restore-official-flight`，以 `dev @ 1dd1dd0` 为父版本，将整个 `src/` 恢复到 `official-demo-baseline @ 2c0fe57`。`sim_env_start.sh` 同样回到基线，只保留已确认需要的 GPUFree SDK 路径修复：`/mnt/c/PX4PSP/...` → `${HOME}/PX4PSP/...`。该路径修改不改变飞行速度。

此版本是“官方源码飞行逻辑 + 环境路径适配”，不是与原始提交逐文件完全相同的仓库。知识库保留开发历史。本分支用于恢复对照；原开发目录的 `dev`、`main` 和 `official-demo-baseline` 标签保留，未切换云服务器版本，也未运行云端仿真。

边界：本地仓库不包含 `~/PX4PSP` 中的飞控参数存储、仿真引擎设置及外部模型脚本。因此本次可以证明仓库源码恢复，不能据此证明云端实际速度、PX4 持久参数或仿真时间倍率已恢复。赛事限制目前依据用户说明；赛事网页和答疑链接本轮未能通过网页工具打开，不编造更具体的规则解释。

## 2. 官方基线与当前 dev 的速度差异

下表比较的是 `dev @ 1dd1dd0`，不是恢复分支。动态 LLM 任务入口为 `llm_mission.launch → bt_ros.cpp → VisualServo`。

| 项目 | 官方基线 | 当前 dev | 判断 |
|---|---|---|---|
| LLM 接近速度参数 `visual_servo/forward_velocity` | 1.0 m/s | 0.4 m/s | 已降低；两版均根据水平偏差进一步缩放指令 |
| LLM 冲刺速度参数 `visual_servo/sprint_velocity` | 1.5 m/s | 0.9 m/s | 已降低 |
| 冲刺时长 `visual_servo/sprint_time` | 1.2 s | 0.6 s | 不是速度值，但改变运动距离和时序 |
| C++ VisualServo 端口缺省前进速度 | 0.35 m/s | 0.35 m/s | 未变；不能用端口缺省值误代替 LLM 实际传入的 1.0 m/s |
| C++ VisualServo `max_vz` | 0.3 m/s | 0.3 m/s | 原端口未变，但 dev 又加入 0.10 m/s 限制，A 区通常令视觉 VZ 为零 |
| 搜索偏航速率 | 0.3 rad/s | A 区 0.12 rad/s | A 区额外调整；普通最大偏航速率仍为 0.5 rad/s |
| 冲刺偏航速率 | 使用普通 PD，限幅 0.5 rad/s | 新增冲刺限幅 0.08 rad/s | 控制方式也变了 |
| 接近速度缩放分母 | 图像宽度 × 0.3，即 192 px | A 区 65 px、其他区默认 48 px | 即使恢复同名速度参数，相同误差下的指令也不同 |
| PrecisionMove | 不存在；相关阶段原先使用 GoTo 等节点 | 参考轨迹 0.35 m/s、加速度 0.25 m/s²；偏航 0.50 rad/s，返穿转向 0.35 rad/s | 新控制路径，没有可直接填回的官方同名速度值 |
| Ego `max_vel / max_acc` | 0.3 m/s / 0.5 m/s² | 0.3 m/s / 0.5 m/s² | 仓库比较未变，不应误改为气球冲刺速度 |
| Land 下降速度 | -0.3 m/s | -0.3 m/s | 该源码未变 |
| 场景小车速度代码 `AgentContestMap.py` | 官方原值 | 文件未变 | 未发现此文件的仓库改动 |
| StableHover 0.16、PrecisionMove 0.10/0.70 | 无这些新增节点 | 稳定判定/异常熔断阈值 | 属于速度判断阈值，不是让无人机按此速度飞行的指令 |

Ego 的障碍膨胀 `0.25 → 0.30 m` 是净空相关配置，不是速度修改；恢复候选将其一并恢复为官方 0.25 m，避免混用两版控制与规划配置。

## 3. 两种方案的取舍

### 方案一：恢复官方飞行逻辑（已准备独立候选）

- 优点：整个 `src/` 可与基线做 Git 树哈希比对，恢复范围明确，避免遗漏硬编码限幅、轨迹生成和其他启动入口。
- 代价：本候选不包含后续 PrecisionMove、动态反穿框、A 区首锁/等待、CONTACT_VERIFY、StableHover、下视后复核和新增 Ego 返航链；官方 Demo 原有的穿模、打击确认不足等问题也会回来。
- 环境例外：只保留 SDK 路径适配，不改变官方飞行源码。
- 适用：当前优先建立“速度和控制均来自官方源码”的对照版本，之后再逐项恢复被允许的逻辑。

### 方案二：保留现有逻辑，恢复速度

能实施，但不能等同于修改两个 launch 参数：

1. 对齐 launch、`bt_ros.cpp` 后备值和各静态 XML 入口，恢复官方接近/冲刺速度；不同入口本来就有不同参数，不能统一全局替换。
2. 处理 VZ、搜索/冲刺偏航速率及接近速度缩放公式；这些属于新控制逻辑与速度的交叉部分。
3. PrecisionMove 没有官方同名节点，不能猜一个数冒充官方值。需要赛事方明确允许怎样的轨迹速度控制，或将这些控制阶段恢复成官方节点；后者已超出“只修改速度值”。
4. 重新评估冲刺时长、追踪边界、制动和复核时序。当前名义冲刺位移约 `0.9×0.6=0.54 m`；官方参数组合约 `1.5×1.2=1.8 m`，均未计实际加减速。即使只恢复冲刺速度而仍用 0.6 s，也约为 0.9 m，已大于 dev 的 0.65 m 冲刺预留量，不能沿用旧的空间假设。
5. 完成编译和全链实测；恢复参数不能保证恢复同样的实际速度曲线。

建议先运行方案一的独立候选，保留 dev 作为功能备份。方案二待速度限制边界明确后逐项恢复，优先保留感知、数据时效与结果记录等不直接改变飞行控制的部分。

## 4. 修改和本地检查

- 修改文件：将 `dev` 相对基线改动过的 `src/` 文件恢复/删除，启动脚本恢复后保留单行 SDK 路径修复；新增本文，并在恢复分支的知识库入口及状态文档注明当前版本。
- 必须重新编译：是。BT 主程序、插件和 CMake 清单均发生恢复，不能只 `git pull` 后直接启动旧 `devel/`。
- 本地检查已通过：暂存的整个 `src/` 与基线无差异（官方源码树 `68e7de059430b8c02ef86bfaff03e3ca9ece22d1`）；启动脚本只差 SDK 路径；4 份恢复涉及的 launch/XML 解析通过，`bash -n sim_env_start.sh` 通过；主目录 dev 与保护引用未变化。
- 本地 Windows 不做 ROS/Catkin 编译或飞行验收。云端结果为 PENDING。

## 5. 云服务器拉取与干净重编译

先在仿真已停止、无人机已落地的状态下操作。使用 `ubuntu` 用户和已验证的编译环境；新开终端，不在仍运行的仿真会话中切换代码。

先检查：

```bash
cd ~/catkin_ws
git status --short --branch
git fetch origin
```

如存在未提交的源码/配置改动，先保存并反馈状态，不执行强制覆盖。新恢复分支尚未在云端创建时：

```bash
git switch -c codex/restore-official-flight --track origin/codex/restore-official-flight
git log -1 --oneline
git diff --exit-code official-demo-baseline HEAD -- src
git diff official-demo-baseline HEAD -- sim_env_start.sh
```

第一条 diff 应无输出，第二条应只有 SDK 路径一行。如果云端已有此分支，使用 `git switch codex/restore-official-flight` 和 `git pull --ff-only`。

保留旧产物作为备份，重新构建，防止旧插件残留（官方 `bt_ros` 会扫描 `devel/lib` 加载插件）：

```bash
cd ~/catkin_ws
UAV_RESTORE_STAMP=$(date +%Y%m%d_%H%M%S)
UAV_RESTORE_BACKUP_DIR="${HOME}/uav_build_backups/${UAV_RESTORE_STAMP}"
mkdir -p "${UAV_RESTORE_BACKUP_DIR}"
if [ -d build ]; then mv build "${UAV_RESTORE_BACKUP_DIR}/build"; fi
if [ -d devel ]; then mv devel "${UAV_RESTORE_BACKUP_DIR}/devel"; fi
source /opt/ros/noetic/setup.bash
./build.sh
```

确认编译退出码为 0 后再继续：

```bash
source ~/catkin_ws/devel/setup.bash
rospack find mission_pkg
rospack find ego_planner
```

备份目录不会被删除，勿将编译产物提交到 Git。不要在失败的编译后启动旧程序。

## 6. 启动和验证

按已跑通的 GPUFree 图形桌面流程，先重启官方 PX4/RflySim：

```bash
cd ~/PX4PSP
./SITLRun.sh
```

在另一个已配置图形显示的 ubuntu 终端，确认 PX4/RflySim/MAVROS 的已知前置条件满足后：

```bash
cd ~/catkin_ws
source devel/setup.bash
./sim_env_start.sh
```

该脚本会执行任务相关启动，不是无副作用的语法检查。应完整关闭上一轮 ROS/仿真，再按此流程启动，以免沿用旧节点、旧参数或仿真时钟。`basic_test.launch` 不作为启动检查。

运行后检查：

```bash
rosnode info /llm_mission_node
rosnode list
rosparam get /visual_servo/forward_velocity
rosparam get /visual_servo/sprint_velocity
rosparam get /visual_servo/sprint_time
rosparam list | grep -E 'manager/max_vel|optimization/max_vel|bspline/limit_vel'
rostopic echo -n 1 /mavros/state
rostopic hz /rflysim/uav1/local/odom
rostopic hz /objects
```

正常预期：三个 VisualServo 参数依次为 `1.0`、`1.5`、`1.2`；Ego 对应速度参数仍为 `0.3`。实际 ROS 节点名称和参数命名空间以 `rosnode list`、`rosparam list` 为准。任务启动日志应包含 `forward_vel=1.00 sprint_vel=1.50 sprint_time=1.2`，不再执行新增的 CONTACT_VERIFY/StableHover/PrecisionMove 处理链。

使用既有自然语言入口提交相同六步任务；不要再额外发布第二套控制任务。记录 `/mission_command`、`/mission_result`、`/objects`、`/mavros/state`、`/mavros/setpoint_raw/local`、`/rflysim/uav1/local/odom`、`/mavros/local_position/odom`、`/planning/pos_cmd` 和 `/rosout`。官方脚本不再自动启用新增的持久录包设置，需另开终端：

```bash
mkdir -p ~/uav_demo_bags
rosbag record -O ~/uav_demo_bags/official_flight_$(date +%Y%m%d_%H%M%S).bag /mission_command /mission_result /objects /mavros/state /mavros/setpoint_raw/local /rflysim/uav1/local/odom /mavros/local_position/odom /planning/pos_cmd /rosout
```

设定速度不等于实际机速：接近阶段有误差缩放，位置模式由 PX4 控制；必须结合 PositionTarget 的 type_mask/coordinate_frame 和里程计解释。实际曲线应与相同 PX4/仿真环境下的官方对照试验比较。

仍需用户核对仓库外的 PX4 参数/仿真配置是否曾手工改变；本次不猜测或覆盖这些值。若曾改动，应从赛事镜像或官方参数快照恢复，并保留差异记录。

异常时反馈：云端 commit、编译日志及退出码、`rosnode list`、上述参数输出、任务节点日志、原始 bag、飞行现象和是否触发官方得分。未得到这些结果前，不将恢复候选标为云端已验收。
