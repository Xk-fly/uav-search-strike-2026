# 当前任务清单（TODO.md）

## 2026-10-28 当前优先级（官方速度恢复版）

- [x] 云端恢复版编译/包路径检查；读取本轮飞行 bag。
- [x] A1 参数修改：按用户决定，膨胀0.25→0.30、dist0 0.10→0.15；官方速度不变。
- [x] A1 赴 A 单次无碰撞；新 bag 已读取并核对点云/地图。
- [x] 实现反穿框接管前、到达后实际停稳门控；官方速度及原穿框路线保留。
- [ ] 云端重新编译并验证门控：停稳才交接、无超时；回归正反穿框与赴 A，确认右偏/穿模是否改善。
- [ ] 后续按实测评估 Ego 有效估速、完整 Ego 穿框、框边地图保留与实际跟踪误差保护；本轮尚未实施。
- [ ] B1（用户本轮暂缓）：保留官方速度，补实际速度制动稳定门控与避障返航。
- [ ] B2（用户本轮暂缓）：验证冲刺前空间、制动余量及可行接触高度。
- 分析方案见 [OFFICIAL_FLIGHT_ANALYSIS_20261028.md](OFFICIAL_FLIGHT_ANALYSIS_20261028.md)；以下任务为历史清单。


> **2026-10-16 最新优先级：当前官方 Demo 已跑完整个六步流程；优先基线与日志归档→全程 GoTo 穿模治理/返航→反穿定位→前视+下视球搜索与命中确认→ArUco ID 驱动决策→一键脚本健壮性。完整台账见 [ISSUES_20261016.md](ISSUES_20261016.md)。以下为原有历史计划。**


> 更新时间：2026-09-16  
> 完成任务后及时勾选，并同步 `PROJECT_STATUS.md`。

## P0｜当前服务器官方基线逐模块验证

- [ ] 1. 在最新 `dev` 执行 `./build.sh`，确认 Catkin 编译 exit code = 0
- [ ] 2. `source devel/setup.bash` 后检查核心 ROS 包与编译产物
- [ ] 3. 启动/确认 `roscore`
- [ ] 4. 启动 `~/PX4PSP/SITLRun.sh`
- [ ] 5. 确认 CopterSim/PX4/RflySim 初始化，记录 `GPS 3D fixed`
- [ ] 6. 独立启动并验证 MAVROS；确认 FCU URL、`/mavros/state`
- [ ] 7. 验证 `sensor_pkg/main.py` 和比赛场景加载
- [ ] 8. 验证 `/rflysim/sensor*` 相关传感器 topic
- [ ] 9. 验证 `cloud_transform.launch` 与 `/cloud_registered`
- [ ] 10. 在有 GPU 的实例验证 `detection_color.launch`
- [ ] 11. 验证 `/objects` 发布和频率
- [ ] 12. 验证 `aruco_detect.launch`
- [ ] 13. 验证 Ego-Planner
- [ ] 14. 验证行为树任务节点
- [ ] 15. 在有 GPU 的实例验证九格模型加载和 `/mission_command`
- [ ] 16. 最后运行 `./sim_env_start.sh`
- [ ] 17. 完成官方示例自然语言任务闭环
- [ ] 18. 将结果更新到 `PROJECT_STATUS.md`、`PROJECT_FACTS.md`、`DEBUG_LOG.md`

## P0｜验证中必须确认的问题

- [ ] `sim_env_start.sh` 中 MAVROS 启动当前被注释：确认正确启动方式和时序
- [ ] 确认 `MAVROS_FCU_URL="udp://:20101@"` 在当前实例实际正确
- [ ] 确认 bundled `object_det/third_party/ultralytics` 的真实运行导入路径
- [ ] 有 GPU 模式下重新记录 GPU 型号、`nvidia-smi` 和 PyTorch CUDA
- [ ] 对比 `src/sensor_pkg/Config.json` 与官方传感器配置

## P1｜知识库维护

- [ ] 每完成一个验证阶段更新 `PROJECT_STATUS.md`
- [ ] 新确认版本/路径/topic/参数时更新 `PROJECT_FACTS.md` 或 `ENVIRONMENT.md`
- [ ] 每次真实故障处理追加 `DEBUG_LOG.md`
- [ ] TODO 项完成后及时勾选
- [ ] 重要架构变化才更新 `PROJECT_OVERVIEW.md`

## P1｜官方基线稳定后

- [ ] 建立当前服务器“可完整复现”的稳定 commit/tag
- [ ] 将稳定 `dev` 经确认后合并到 `main`
- [ ] 根据比赛任务和规则建立性能基线
- [ ] 再开始检测、视觉伺服、行为树、规划、大模型等专项优化

## 当前禁止提前做的事

```text
在当前新服务器 baseline 尚未跑通前：
不要大范围重构
不要批量升级依赖
不要把环境问题误判成算法问题
不要直接修改 main
不要把旧服务器“曾经跑通”当成当前服务器已经验证
```
