# 项目知识库入口（docs/README.md）

> **2026-10-29 最新进展：** `codex/restore-official-flight` 增加三色共用丢球结束打击、实际停稳与 Ego 返航；搜索延长至120秒，弱检测放慢转向。官方接近/冲刺参数及0.30/0.15净空保留。需云端重新编译实飞，步骤见 [PROJECT_STATUS.md](PROJECT_STATUS.md)。

> 这套文档是 ChatGPT、Codex 和用户本人共同使用的长期项目记忆。GitHub Private 仓库中的版本是跨对话、跨电脑、跨服务器的主要知识来源。

## 1. 推荐阅读顺序

```text
AGENTS.md
↓
docs/PROJECT_OVERVIEW.md
↓
docs/PROJECT_STATUS.md
↓
docs/PROJECT_FACTS.md
↓
docs/ENVIRONMENT.md
↓
按需读取 DEBUG_LOG.md / TODO.md
```

| 文件 | 作用 |
|---|---|
| `AGENTS.md` | 规定 AI/Agent 应该怎么工作 |
| `PROJECT_OVERVIEW.md` | 项目目标、架构、总体流程 |
| `PROJECT_STATUS.md` | 到目前为止做了什么、当前停在哪里 |
| `PROJECT_FACTS.md` | 已确认版本、路径、接口、状态 |
| `ENVIRONMENT.md` | GPUFree/ROS/PX4/RflySim/Python/模型环境 |
| `DEBUG_LOG.md` | 真实问题、证据、处理、验证、结论 |
| `TODO.md` | 当前任务和后续优先级 |

## 2. 事实优先级

发生冲突时按以下顺序判断：

```text
当前服务器实际命令输出
> 当前 dev 分支实际源码/配置
> 赛事官方手册
> 旧服务器历史经验
> 聊天中的推测
```

仍不确定时标记 `PENDING` 并要求执行验证命令，而不是自行补全。

## 3. 文档更新规则

```text
项目目标/系统架构变化       → PROJECT_OVERVIEW.md
阶段完成/交接状态变化       → PROJECT_STATUS.md
新增确定事实/版本/路径       → PROJECT_FACTS.md
运行环境变化                → ENVIRONMENT.md
真实调试过程                → DEBUG_LOG.md
任务新增/完成               → TODO.md
Agent 行为规范变化           → AGENTS.md
```

避免在多个文件重复维护同一个易变化的数据。

## 4. 对话切换时的使用方法

新开 ChatGPT/Codex 对话时可直接要求：

```text
请先阅读 AGENTS.md 和 docs/ 下的项目知识库，
重点读取 PROJECT_STATUS.md 与 PROJECT_FACTS.md，
在现有状态基础上继续工作，不要假设未验证事项已经完成。
```

## 5. 当前最重要的状态

截至 2026-09-16：

```text
Git/GitHub/Codex 开发闭环已建立
主要环境信息已核验
RflySim SDK 路径问题已修复
当前新服务器尚未完成完整 Demo 验证
下一步是按模块逐层验证官方 baseline
```

## 2026-10-16 基线复现与最新交接（更新提示）

> **最新状态优先于本文 2026-09-16 的历史概述。** 2026-10-16 GPUFree 克隆的 L40 环境已由用户运行完成自然语言→九格大模型→六步行为树→RflySim 官方 Demo 的完整执行流程；ArUco ID35 已观察到，存在穿墙/穿模、气球命中未验证等问题。旧 `PROJECT_STATUS.md` 的“尚未验证 Demo”已过时。

新对话请先读这三个补充文件，再参考原 9 月技术档案：

1. [BASELINE_20261016.md](BASELINE_20261016.md)：已经跑通的部分、环境和证据口径；
2. [ISSUES_20261016.md](ISSUES_20261016.md)：全部主要已知现象、未修复问题和优化顺序；
3. [HANDOFF_20261016.md](HANDOFF_20261016.md)：从零重启、一键脚本、关键参数、交接要求与下一步。

代码基线 `official-demo-baseline` tag 不做覆盖；本次均为 `dev` 文档归档，**没有修改/验证新的飞控或感知算法**。如 ChatGPT 项目空间另有《全对话技术交接》《运行手册》，它们提供更完整的日志与过程细节。

- [CHATGPT_START_PROMPT_20261016.md](CHATGPT_START_PROMPT_20261016.md)：可在新对话直接复制使用的 GitHub 长期备份提示词。
