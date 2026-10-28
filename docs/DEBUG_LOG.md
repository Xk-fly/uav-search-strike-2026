# 调试与决策日志（DEBUG_LOG.md）

> **2026-10-16 最新调试记录索引：本轮已观察完整六步 Demo，同时发现穿模、固定 GoTo、ArUco 服务缺席及一键启动行为树缺席等现象。见 [ISSUES_20261016.md](ISSUES_20261016.md) 和 [BASELINE_20261016.md](BASELINE_20261016.md)。下方旧记录保留历史口径。**


> 用途：记录“问题 → 证据 → 处理 → 验证 → 结论”。  
> 这里记录已经真实发生过的调试过程，不把未来计划写成已完成事实。

## 2026-09-15｜VS Code SSH 下 ROS 版本显示 `<unknown>`

### 现象

官方远程桌面：

```text
ubuntu@gpufree-container
rosversion -d → noetic
```

VS Code Remote SSH 初始终端：

```text
root@gpufree-container
rosversion -d → <unknown>
```

### 定位

两个终端运行用户和 Shell 初始化环境不同。

### 处理

```bash
su - ubuntu
```

### 结论

后续 ROS 调试统一以 `ubuntu` 用户为主要运行身份。源码开发采用“Windows 本地 + GitHub + 服务器 pull”，减少远端编辑权限混乱。

---

## 2026-09-15｜官方 Demo Git 基线建立

### 需求

需要让 Codex/AI 可持续修改，同时随时能回退官方原版。

### 检查

```text
catkin_ws 最初 NOT_GIT_REPO
src/cv_bridge/.git 为嵌套 Git
项目约 709M
排除 build/devel 后无 >50M 普通源码文件
```

### 处理

完成：

```text
原始源码 tar 备份
移出 cv_bridge/.git 元数据
建立 .gitignore
git init
baseline commit
main
official-demo-baseline tag
```

官方基线：

```text
2c0fe57 baseline: official competition demo
```

### 结论

后续修改可通过 Git/tag/物理备份回退。

---

## 2026-09-15｜GPUFree 到 GitHub 的访问

### 现象

普通 HTTPS 请求 GitHub 曾长时间无响应。第一次直接：

```bash
ssh -T -p 443 git@ssh.github.com
```

出现 publickey 认证失败。

### 定位

网络到 `ssh.github.com:443` 实际可完成 SSH 握手；失败原因是没有使用项目专用私钥。

### 处理

配置：

```text
Host github-uav
HostName ssh.github.com
User git
Port 443
IdentityFile ~/.ssh/id_ed25519_uav2026
IdentitiesOnly yes
```

执行：

```bash
ssh -vT github-uav
```

结果：

```text
Server accepts key
Authentication succeeded (publickey)
```

### 结论

当前云服务器通过 SSH 443 + Deploy Key 访问 GitHub Private 仓库。

---

## 2026-09-16｜当前新服务器环境核验

已确认：

```text
Ubuntu 20.04.6 LTS
ROS Noetic
Conda demo
Python 3.10.19
PyTorch 2.3.1+cu121
CUDA Toolkit 12.8
MAVROS 1.20.1
PX4 v1.12.3
```

当前为无卡模式：

```text
/dev/nvidia* 不存在
torch.cuda.is_available() = False
```

这表示实例启动模式，不是永久无 GPU 的结论。

---

## 2026-09-16｜RflySim SDK 路径不一致

### 现象

真实 SDK：

```text
/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
```

仓库脚本旧路径：

```text
/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK
```

### 修改

```bash
RFLYSIM_SDK="${HOME}/PX4PSP/RflySimAPIs/RflySimSDK"
```

提交：

```text
3747c37 env: fix RflySim SDK path
```

服务器 `git pull` 后：

```text
RflySim SDK path OK
```

### 结论

路径问题已修复；但只证明路径存在，不代表完整 Demo 已通过。

---

## 2026-09-16｜Ultralytics 环境级 import 不存在

当前 Conda `demo`：

```python
import ultralytics
```

返回：

```text
ModuleNotFoundError
```

仓库同时存在：

```text
src/object_det/third_party/ultralytics/
```

当前结论：

```text
环境级 ultralytics 未安装：确认
bundled 源码存在：确认
真实 detection 节点能否正常运行：PENDING
```

---

## 2026-09-16｜准备首次逐模块基线验证

决定不直接一键启动后混合排错，而按：

```text
Catkin 编译
→ ROS Core
→ PX4 / RflySim
→ MAVROS
→ sensor_pkg
→ 点云
→ 目标检测
→ ArUco
→ Ego-Planner
→ 行为树
→ 九格大模型
→ 完整 sim_env_start.sh
```

逐层验证。

下一条调试记录应从 Catkin 编译结果开始。


## 2026-10-28｜恢复官方速度后的飞行实测与分析

- 用户反馈：恢复分支已完成编译和 rospack 检查；起飞、穿框、ArUco、反穿框正常；赴 A 时桨叶擦框，冲刺后出现穿模。
- 原始证据：`official_flight_20261028_132253.bag`，109.55s、23,106条消息；完整分析见 [OFFICIAL_FLIGHT_ANALYSIS_20261028.md](OFFICIAL_FLIGHT_ANALYSIS_20261028.md)。
- 确认：官方冲刺1.5m/s、约1.2s；结束附近实际速度约1.49m/s，一个悬停命令周期后已切直线GoTo返航；最低高度约0.31m。任务97.86s返回SUCCESS并落地，但未证明官方命中或无碰撞。
- 数据限制：RflySim twist全零且采样重复，已按头时间戳去重再作位姿差分；缺少点云、膨胀地图、官方碰撞和得分事件。
- 决定：先单变量评估Ego膨胀0.25→0.30，再逐项增加实际速度制动门控和避障返航，暂不加PID、不改官方速度。本轮仅分析，没有实施飞行补丁。

## 2026-10-28｜Ego 净空参数调整（待实飞）

- 用户授权：膨胀调到0.30m，并小幅增加轨迹距膨胀障碍的优化间距；问题二本轮暂不改。
- 实施：`FS-J310_Param.xml` 的膨胀0.25→0.30、`dist0` 0.10→0.15；其余飞行源码及速度设定不变。
- 本地 XML/源码差异检查；云端无需重编译，须重启后回归正反穿框及赴 A。实际避障效果尚未验证，步骤见 `PROJECT_STATUS.md`。

## 2026-10-28｜净空调整后实测：赴 A 改善，反穿框右偏

- 证据：`ego_clearance_20261028_141237.bag`，108.50秒，含点云、膨胀图、目标点与控制指令；记录数与 bag 索引一致。用户确认赴 A 无碰撞，反穿框轻微穿模。未录参数快照/官方碰撞事件。
- 反穿框接管（51.43s）时仍在运动：去重位姿差分速度约0.55m/s，MAVROS约0.62m/s；RflySim odom twist始终全零，而 Ego直接使用该速度字段作为初始状态。
- 框平面点云约X=3.3m；实际过平面约55.14s，Y=-0.935；同时规划参考Y=-1.143，右偏约0.21m。该段最大三维跟踪误差约0.65m，最大Y偏差约0.28m；setpoint与规划参考一致，偏移主要发生在实际跟踪。
- 静态框柱点云粗估开口中心Y≈-1.22，复用视觉框心Y≈-1.14，已有约8cm侧偏（非精确标定）。54.7s附近实际位置进入记录的膨胀占据栅格；随后框边从近距点云/膨胀图消失。cloudCallback每帧清空局部缓冲，提前见过的障碍没有持续保留。
- 当前反穿为Ego到框心→两段机体系GoTo；规划只发布位置，未使用规划速度前馈。RViz默认红色显示为B样条控制点/连线，不能当作机体扫掠边界或实际航迹。
- 下步：保持0.30/0.15及官方速度；先补接管前实际停稳、有效估速、框前对中到框后离开的完整Ego穿越；再处理静态框短时地图保留和跟踪偏差保护，回归两次穿框及赴 A。
- 打击仍未接入Ego：本包90.923s仅一帧悬停，90.943s已直线返航，结束速度约1.4m/s。建议Ego负责远距接近/返航，末端视觉伺服保留，并共用障碍走廊检查与持续制动；目标接触范围须与墙/车等障碍区分。仅像素目标不能直接替换成Ego三维航点。
- 本轮只分析并更新简要记录，未改飞行源码，未绘图。

## 2026-10-28｜反穿框停稳交接补丁（待实飞）

- 实施：PlanningControl 增加可选接管前与到达后停稳门控，反穿框启用；实际速度≤0.12m/s连续0.5s才能继续，10s超时保持位置并返回FAILURE。到达后保持原规划目标，避免捕获正在漂移的位置。规划指令和新速度状态加锁。
- 速度模长取 MAVROS；只作停稳判断，不混用其位置或速度向量。bag 中两路位置轴旋转约90°；MAVROS 消息时间戳还可能领先接收时间约32ms，因此使用接收时间计时并检查时间戳有效性/重复/陈旧。
- 范围：仅修改 PlanningControl、LLM 反穿节点和四份完整任务 XML；官方速度、0.30/0.15净空参数、航点/两段 GoTo、气球视觉伺服/冲刺不变。自动审批拒绝了扩大到共享 Ego 估速与整段路线重写的补丁，理由为未实测的碰撞和任务中断风险；本轮收窄到停稳交接。
- 验证：门控函数通过 Clang 语法检查（ROS 时间/消息/锁替身），BT XML 及基线差异检查通过；Windows 未做 ROS 完整编译或实飞，运行边界场景也未完成本地执行。云端须重新编译并回归，命令与反馈要求见 PROJECT_STATUS。
