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
