# RflySim v4.12 / UE5 Pro 机顶雷达：源代码核查与试配方案（仅文档，不自动启用）

## 当前可确认的事实
- 源码`src/sensor_pkg/Config.json`只有3个`VisionSensors`：SeqID=0, TypeID=7（深度转点云）、SeqID=1, TypeID=1（前视RGB）、SeqID=2, TypeID=1（下视RGB），没有TypeID=20~23的LiDAR。
- `src/sensor_pkg/main.py`通过`vis.jsonLoad(1)`读取配置并请求RflySim传感器。当前`cloud_transform_node.py`订阅`/rflysim/sensor0/Depth_Cloud`并发布`/cloud_registered`，供Ego消费；不是独立雷达。
- 场景飞机头顶可见类似LiDAR的模型，但模型不等于已经在RflySimSDK中申请了一个虚拟LiDAR传感器，不能据此声称已提供扫描点云。
- 竞赛场景部分对象在`AgentContestMap.py`中随机偏移、车辆会运动，所以不能把一组固定世界坐标直接当作永远准确的静态全局地图。

## 官方资料
RflySim 官方文档 [Vision sensor config](https://rflysim.com/doc/en/4.DevRef/vis-cfg.html) 与 [multimodal sensor API](https://rflysim.com/doc/en/4.DevRef/vis-cfg.html) 标明：TypeID 7 为深度转点云、TypeID 20~23 为LiDAR；TypeID 20的ROS话题名称为`/rflysim/sensor{SeqID}/vehicle_lidar`，TypeID 21为`/rflysim/sensor{SeqID}/global_lidar`。确切支持受版本/授权/SDK约束；当前运行服务器尚未实测。

## 一阶段：只读核查（不会影响现在比赛六步）
在已启动的ROS系统中：
```bash
rostopic list | grep -Ei 'lidar|laser|scan|cloud|depth'
rostopic info /cloud_registered
rostopic hz /cloud_registered
grep -R 'vehicle_lidar\|global_lidar\|TypeID.*20' -n \
  ~/catkin_ws/src/sensor_pkg \
  ~/PX4PSP/RflySimAPIs/RflySimSDK/vision 2>/dev/null | head -60
```
优先确认服务器SDK代码是否对Type20、21提供ROS桥接，已经存在话题时不要重复启动传感器。

## 二阶段：独立试配（下轮任务，当前不改正式Config.json）
1. 复制正式配置做测试配置，保持SeqID 0/1/2现有数据链，增加SeqID=3、TypeID=20的LiDAR条目（不可覆盖已有编号）。
2. 根据机顶真实安装位置设置FRD系外参SensorPosXYZ和扫描参数。LiDAR配置示例应依据服务器SDK中Type20的例程，确认SendProtocol/端口并使用唯一编号；不要套用当前RGB相机的端口。
3. 先验证点云能否在`/rflysim/sensor3/vehicle_lidar`出现，`rostopic type`期望`sensor_msgs/PointCloud2`，再检查RViz中坐标系与墙体感知；不立即喂进Ego，以免错误点云造成规划误判。
4. 验证相机与LiDAR点云的world TF、实际墙体检测和更新频率后，再另做深度+LiDAR融合、持久化占据地图和全局安全路由。当前`Ego`只在局部图内规划不能保证墙体已被观测。
5. 绝对不要在空中直接修改传感器Config.json或重启`main.py`、`sim_env_start.sh`。
