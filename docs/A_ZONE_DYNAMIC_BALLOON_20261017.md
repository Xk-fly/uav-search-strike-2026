# A区动态小车红球：目标丢失保持航向（2026-10-17）

本次修改仅影响动态LLM行为树 `hit_balloon:red` 映射到A区的视觉伺服参数，穿框/反穿框动作不修改，B/C维持旧参数和失去目标后搜索行为。GitHub dev版本不代表服务器已构建或飞行验证。

## 执行顺序
1. `return_cross_frame` 完成后才发起 `hit_balloon:red`：Ego首先到A区 `5.3;1.0;1.00`，接着GoTo在同一高度定向 `yaw=1.5708`；
2. 之后行为树才运行 `TargetTracker(red_balloon)` 和 `VisualServo`，途中偶见红球不会锁定；
3. 首次有效红球出现后启用“曾见过”锁定标志；正常检测时水平校准，连续3帧不同YOLO检测满足40px后进入接近；
4. A区不再要求垂直误差≤55px才能开始前进，纵向速度在A区为0；面积≥0.05、水平≤40px且垂直误差≤130px才允许发起冲刺；
5. 接近中一旦YOLO发送明确无红球帧，或0.40秒没有新有效检测，则停止前进并捕获当时位置及航向定点等待；重现后从ALIGN重新校准，禁止自动旋转重新搜索；
6. 首次发现前只允许绕到达时航向有限角度(+0.60rad)慢速寻找，达到上限就保持航向等待；
7. 冲刺本身仍然保留0.9m/s、0.6s、制动0.5s，冲刺期间目标消失仍只记`/balloon_attack/hit_candidate`（候选，非官方得分确认）；
8. `hit_balloon`的超时不再被ForceSuccess掩盖；A区90秒未完成冲刺会报告FAILURE，原来`land`命令的StableHover→Ego→PrecisionMove→Land保持不变，仅在前面任务成功后执行。失败后须监视并安全接管，不能把FAILURE视为自动返航。

## 安全和验证
- 首先在GPUFree关闭仿真、确认已解除武装；检查本地Config.json已恢复三路正式配置（SeqID 0/1/2 TypeID 7/1/1）。
- `git switch dev && git pull --ff-only`，工作区若有未提交修改先查看git status，勿使用reset --hard；`./build.sh && source devel/setup.bash`。
- 用`./sim_env_start.sh`启动，仅在安全条件下启动`takeoff,cross_frame,detect_aruco,return_cross_frame,hit_balloon:red,land`等正常比赛指令。
- 观察任务日志：到A区前不得出现`A_ZONE_FIRST_TARGET_LOCKED`；首次见球有FIRST_TARGET；短暂丢失为`A_ZONE_WAIT_TARGET`，无 yaw_rate 自动搜索；重现应出现`ALIGN stable -> APPROACHING`，满足接近条件出现`APPROACH -> SPRINT`；`StableHover SUCCESS`后Ego接手。
- 记录`/objects`、`/mavros/local_position/odom`、`/mavros/setpoint_raw/local`、`/balloon_attack/target_visible`、`/balloon_attack/hit_candidate`和`/mission_result`。下视`/objects_down`独立运行，不参与A区撞击。
- 测量冲刺后真实减速、至障碍物最小距离、红球是否真正消失/得分。若场景边界不安全立即停止，不用单次尝试成功冒充真实命中验证。

## 后续工作（未实现）
- 下视YOLO和ArUco的搜索式联动需先做被动检测，避免ArUco BT节点的悬停命令与爬升控制争夺飞控；
- 深度`/Depth_Cloud`经过坐标校验后累积占据地图，先可视化再接入Ego全局返航。


## 2026-10-17 17:27 实测复盘（用户上传 temp.bag，非本仓库数据文件）

本节为新增实测权威记录，以上原始设计参数为历史快照；最新实现见 commit `81651e4`（A区0.90m、首次3帧锁定、冲刺5帧+1.5s、初扫限幅1.20rad、反穿框前退0.5m和出口1.5m）。

- 本次bag约24MB、328.7s；命令 `takeoff/cross_frame/detect_aruco/return_cross_frame/hit_balloon:red/land`；ArUco ID35成功识别。
- A区伺服相对bag开始229.94s时启动，机体实际起始位姿约(5.21,1.02,0.90)m；首次稳定见红球235.37s；此后多次进入APPROACH与WAIT_TARGET。
- **本次没有出现SPRINTING、BRAKING或真正的碰撞后验证**；`/balloon_attack/hit_candidate`只有一条false；约319.95s任务`FAILURE`（A区90s超时），后续land/返航序列并未执行。
- 以`/rflysim/uav1/local/odom`为实际工作空间坐标：A区z范围0.857~0.954m，起点(5.21,1.02,0.90)m，终点(1.18,1.40,0.91)m。连续朝同一方向累计推进约4m，应增加A区航道边界/最大追踪位移，防止追到近墙区域。
- A区194次置信度≥0.80的红球目标消息中，最大检测框面积占比0.03564（**不到当前0.05冲刺阈值**），故冲刺面积条件从未满足；74/194次红球中心纵坐标>370px（现有冲刺垂直误差<=130px时允许中心Y最多370px），39次框碰到图像下边缘。
- 下一轮优先候选：降低A区高度（0.70~0.75m需确认机体和小车净空），同步校验/调整min_world_z下限；其次重评面积触发条件，增加目标接近验证与区域距离边界，避免盲追。
- 本次bag来源原日志 `/tmp/mission_logs/llm_mission__2026-10-17-17-27-29.bag`；仓库后续已把默认录制目录改为 `~/uav_demo_bags`（`sim_env_start.sh`自动mkdir；仍需GPUFree端git pull才生效）。
- `/objects_down`本次未录到有效消息，下视YOLO仍默认关闭，截图能看到下视红球但尚无下视自动验证结果。不能以本次“悬停后红球又出现”解释为撞击后新一轮追球：实际一直处于接近/丢失/重捕状态。


## 2026-10-17 晚间：官方基线高度核实 + A区试验高度/边界保护
- 核对 `official-demo-baseline` (`2c0fe57`): `src/mission_pkg/launch/llm_mission.launch` 给A区红球靠近点为 **1.00m**，B/C区均 **0.70m**；`config/hit_balloon.xml` 单独撞球Demo起始高度为 **1.00m**；`visual_servo.cpp` 原始控制 **VX+VZ+yaw_rate**，会边跟踪边垂直调整。因此这些数值是**初始高度**，不能据此断言最终机体撞球高度；用户目测原版撞到球但缺少原版命中轨迹bag。
- 本轮候选A区改为 **0.75m**（比官方B/C区0.70m高5cm，且不是官方A区的原始数值）；**A区垂直伺服仍关闭**，保持固定高度便于对照。
- A区 `min_world_z` 从0.72变成 **0.60m**：这是仿真试验的高度保护阈值，**不等于经实体安全认证的离地间隙**。入场前确认机体实际底部/车顶间隙。
- 上一轮bag最大红球bbox面积仅3.564%，低于原5%。A区候选面积阈值设 **2.5%**，保留5帧稳定冲刺门槛、1.5s最短锁定跟踪和水平/垂直容差。
- 新增 **A区从视觉伺服开始位置出发最大水平位移3.35m**，冲刺/制动预留0.65m。到边界时停止前进并悬停，预留冲刺距离不足时先禁止冲刺；这是粗略实验约束，无法替代场景深度地图/墙碰撞验证。
- 不修改B/C区、已验证的穿框/反穿框、ArUco和StableHover/Ego正常返航链路。
- 仍需GPUFree本地构建和实测验收。优先记录 `APPROACH -> SPRINT`、`SPRINT -> BRAKING`、`A_ZONE_PURSUIT_LIMIT`、`A_ZONE_PRESERVE_SPRINT_SPACE`；保存rosbag到 `~/uav_demo_bags/`。

## A区撞击后下视反馈（可选且默认开启，仅A区）
- 动态行为树在VisualServo成功（完成冲刺尝试＋BRAKING）后依次执行 `StableHover`（真实速度<0.16m/s稳定0.9s）、`PrecisionMove height=1.25`（单独原地爬升）、`BalloonObserve mode=down timeout=3.0 observe_full_duration=true`。
- 只有真正完成冲刺尝试才执行此检查，前段失败仍不会假成功或自行进入降落流程。
- 使用已存在的下视识别节点，检查时打开`/detect_ctrl_down`，结束后关闭；全过程不允许下视检测写飞行速度。
- 检查结束发布 `/balloon_attack/postcheck_result`：`RED_STILL_VISIBLE`（至少2次连续有效检测）、`RED_NOT_OBSERVED`（有检测帧但未见）、`INCONCLUSIVE_NO_DATA`（无有效图像检测帧）。**下视未观察到红球≠官方得分/气球已破**；任何观测结论均继续原Ego安全返航。
- 关闭此可选链路：将launch参数 `balloon_zone_A/postcheck_enabled` 设为false，保留原有 `hit_balloon→land` 路线。
- 本轮只做了GitHub提交及代码检查，运行编译、ROS bag链路及爬升净空需要GPUFree实测。

## 2026-10-17 19:31 bag 精确分析与 0.55m 测试设置
- 数据源：用户上传 `llm_mission__2026-10-17-19-31-27.bag`，约36.26MB，rosbag2.0无压缩，包内总466.77s。任务开始录制后99.81s，A区视觉伺服启动215.03s、3帧首锁222.11s、251.04s进入SPRINT、251.65s进入BRAKING、252.16s完成制动。
- `/rflysim/uav1/local/odom`：A区持续阶段平均高度0.762m（最小0.725、最大0.802），**冲刺瞬间Z约0.776m**，未看到`VisualServo floor protection`日志，因此不是0.75m高度保护引发的主动回升。
- 冲刺日志`APPROACH -> SPRINT area=0.025 err_x=-15.1 err_y=122.7 stable=5`，水平已大致居中，纵向仍明显偏下。冲刺发出0.90m/s、持续约0.61s；目标是否物理碰撞未有比赛官方得分信息，截图目视从球上方掠过。
- **新故障**：`StableHover`在252.16s启动后始终报告`waiting for fresh odom samples`，261.16s`FAILURE braking timeout`，`/mission_result`只有`FAILURE`。因此自动爬升/下视3秒观测/Ego返航都未执行；bag无`/objects_down`观测消息。
- 根因代码路径：`StableHover::onRunning()`旧版在检查当前里程计头时间戳有无前进**之前**，先用上次存储的last_odom_stamp_判断0.4s超时并直接return，导致超时后即使有新消息也无法更新sample stamp，进入不可恢复等待。bag中`/rflysim/uav1/local/odom`在252~261s持续到达且时间戳前进，支持此代码路径。已将顺序修复，保留当前odom时效、速度差分、制动和稳定阈值门控；仍须GPUFree编译验证。
- 用户指定下轮直接测试A区固定目标高度**0.55m**（原0.75m），同步将A区`min_world_z=0.43m`；当前默认`max_descent_from_entry=0.15`，预计floor=max(0.43,0.55-0.15)=0.43m，保护触发约floor+0.04=0.47m，触发后目标爬升至floor+0.12=0.55m。此值仅是**仿真试验限位**，在机身/车体几何不明确时不得将其视为已认证防撞净空。
- A区`horizontal_only_alignment=true`，`VisualServo`仍在控制vx+yaw但**不主动调节VZ**；先单独验证改变固定航点高度的影响，避免与重启垂直视觉伺服混合改变因素。复测前请确保0.55m沿途以及小车车顶的净空，安全不符应取消本轮低高度测试。
- 其他A区触发/保护（面积2.5%、连续3帧锁定、连续5帧冲刺、3.35m追踪距离、0.65m冲刺余量）保持不变；主备份分支不动。

## 2026-10-17 20:03近距红球疑似击破与去A区Ego间隙优化（待服务器验证）
- 上轮0.55m实验：目视红球从小车消失，但A区视觉伺服始终未满足SPRINT门槛，`APPROACHING`失球只能进入等待，直到任务FAILURE。
- 仅A区启用`CONTACT_VERIFY`：在APPROACH中曾稳定锁球、最近有前进指令、最后目标面积≥6%、横向误差≤120px、最近检测及前进指令都足够新鲜，随后失球立即零速度制动，持续1.8s复核至少5帧新的无红球检测；期间复现红球立即取消并重对准；成功复核只上报`hit_candidate=true`，然后进入原`BRAKING`，再交给已有StableHover、爬升、下视3s观察和Ego返航。若无足够检测数据，则判不确定并回等待，不把掉帧视为成功。
- A区冲刺门槛试调为面积2.0%（原2.5%），横向误差50px（原40px），不同检测帧3（原5），纵向容差150px（原130px），冲刺速度0.9m/s、时长0.6s以及0.5s制动不变；B/C区完全保留之前行为。
- Ego占据地图膨胀半径0.25→0.30m，只改FS-J310 ROS launch参数、非规划核心代码；全局生效，故必须特别回归正反穿框和A区近框转场，必要时通过`obstacles_inflation:=0.25`恢复。**地图全局膨胀并非可测量的机体实际净空保证**。
- 0.55m高度、A区3.35m追踪上限、0.65m冲刺余量、A区已验证多帧航向锁定维持不变。
- 等待GPUFree服务器编译与rosbag验收：`NEAR_CONTACT_START`、`NEAR_CONTACT_VERIFY`、`HIT_CANDIDATE_APPROACH`、`NEAR_CONTACT_CANCEL`、`APPROACH -> SPRINT`、`SPRINT -> BRAKING`、`StableHover SUCCESS`、`A_ZONE_POSTCHECK`以及`/mission_result`。如果视觉候选为真但下视红球仍在，应记录冲突，并非比赛正式命中。
