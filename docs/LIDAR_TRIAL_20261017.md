# RflySim 独立 LiDAR 试验配置（待 GPUFree 实测，非默认开启）

**目的**：核实机顶扫描器能否通过 RflySim UE5 Pro 与现有 VisionCaptureApi 生成真正的独立 LiDAR 点云。暂不接入 Ego，不改当前 `Config.json`、`main.py`、`sim_env_start.sh`。本试验文件只作为可选替换模板，不影响已经完成的比赛 Demo。

## 当前事实与预期
- `Config.json` 只有三路：0=Type7 深度点云、1=前视 Type1 RGB、2=下视 Type1 RGB。
- GPUFree 用户实测的 ROS 话题只有 `/cloud_registered` 和 `/rflysim/sensor0/Depth_Cloud`；`/cloud_registered` 由 `cloud_transform_node` 提供且仅 9.96 Hz，**不是独立 LiDAR**。
- GPUFree 服务器`VisionCaptureApi.py`源码已检索到 TypeID=20 的接收逻辑及 `/rflysim/sensor{SeqID}/vehicle_lidar` / `global_lidar` 发布逻辑。接口存在不代表引擎端一定能正常响应。
- 官方 RflySim 文档 https://rflysim.com/doc/en/4.DevRef/vis-cfg.html 写明 Type20机体系 LiDAR、21世界系 LiDAR、23 Mid-360。类型20可用 DataWidth/Height + otherParams[2..5] 调扫描视场。此次首轮使用较低负载360×16×10Hz，距离15m，水平 -170°～170°，俯仰 -20°～+20°，机体 FRD 位置 [0,0,-0.12]（假设高于飞机中心12cm，**未标定**）。SendProtocol=[0,0,0,0,0,0,0,0] 采用文档示例的本机共享内存，不复用已占用9999/9998/9996端口。
- 预期新 ROS topic: `/rflysim/sensor3/vehicle_lidar` (sensor_msgs/PointCloud2)。一定要核对实际 frame_id 和点云坐标是否符合机体系；**不可直接接入世界系 /cloud_registered**。

## 切换前：飞机已落地、disarmed，退出仿真任务并确保无 ROS/传感器进程
在 GPUFree 终端，确保已拉取包含此配置的 `dev`：

```bash
cd ~/catkin_ws
git status --short --branch
python3 -m json.tool src/sensor_pkg/Config_lidar_trial.json >/dev/null
CONFIG=src/sensor_pkg/Config.json
cp -p "$CONFIG" "${CONFIG}.before_lidar_$(date +%Y%m%d_%H%M%S).bak"
cp src/sensor_pkg/Config_lidar_trial.json "$CONFIG"
python3 -m json.tool "$CONFIG" >/dev/null
python3 - <<'PY'
import json
x=json.load(open('src/sensor_pkg/Config.json'))['VisionSensors']
assert [r['SeqID'] for r in x]==[0,1,2,3]
assert [r['TypeID'] for r in x]==[7,1,1,20]
print('LiDAR trial config installed: 4 sensors, original first three preserved')
PY
```

**注意**：此替换使服务器本地 `Config.json` 变为未提交 Git 改动；后续`git pull`前必须先恢复/保存，不能丢失原版。若系统还在运行不可修改文件并热重启传感器。

## 冷启动试验
按既有运行手册，先启动`~/PX4PSP/SITLRun.sh`，确认模拟器就绪后再用`conda activate demo; cd ~/catkin_ws; bash ./sim_env_start.sh`。**只在无人机落地待机时运行，不用起飞**。

```bash
rostopic list | grep -Ei 'sensor3|lidar|cloud|depth'
rostopic info /rflysim/sensor3/vehicle_lidar
timeout 8 rostopic echo -n 1 /rflysim/sensor3/vehicle_lidar/header
timeout 8 rostopic echo -n 1 /rflysim/sensor3/vehicle_lidar/width
rostopic hz /rflysim/sensor3/vehicle_lidar
rostopic info /rflysim/sensor0/Depth_Cloud
rostopic info /rflysim/sensor1/img_rgb
rostopic info /rflysim/sensor2/img_rgb
```

成功标准：sensor3 节点实际发布 sensor_msgs/PointCloud2，有非零width、周期持续、RViz显示周围墙体且机体坐标与实际方向一致；前面三路不退化。若没有话题，请保留 sensor`main.py`/VisionCaptureApi 启动日志里 `sendReqToUE4` 应答信息、相关 warning/errors、`rostopic list`，核查UE5引擎版本/共享内存支持/授权，不要盲改端口。

## 回退
试验结束、飞机落地、停机后，恢复**原文件**（优先恢复刚才的 .bak）：
```bash
cd ~/catkin_ws
ls -lt src/sensor_pkg/Config.json.before_lidar_*.bak | head
# 将下面 BACKUP 路径替换为实际刚创建的备份文件，不要猜路径
BACKUP=src/sensor_pkg/Config.json.before_lidar_YYYYMMDD_HHMMSS.bak
test -f "$BACKUP" && cp -p "$BACKUP" src/sensor_pkg/Config.json
git diff -- src/sensor_pkg/Config.json
```
`git diff`应为空；若配置是CRLF版本，恢复原始备份更可靠。

## 下一步（数据验证完成之后，另一次迭代）
先在 RViz 独立展示机体系雷达数据，再新建LiDAR->world转换与静态占据地图（使用里程计和时间同步）；之后才做融合/全局安全规划。不要同时让两个节点往 /cloud_registered 发布不同坐标系点云，不要把低质量雷达点云直接接入Ego。
