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
