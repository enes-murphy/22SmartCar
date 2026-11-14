# M3-3｜自主导航与调优

本阶段必须使用 M3-2 产出的地图，并针对阿克曼运动约束配置 Nav2。

## 完成清单

- [x] 解释 `map → odom → base_link` 的来源与含义。
- [ ] 配置 AMCL、代价地图、阿克曼适用的规划器和控制器。
- [ ] 用一条 launch 命令启动仿真、定位与 Nav2。
- [ ] 选择一条大于 5 m 且包含转弯的测试路径。
- [ ] 至少调 3 个参数，保留改前/改后证据。
- [ ] 重复 3 次，统计成功率、耗时和失败原因。

```text
M3-3/
├── config/
├── launch/
├── scripts/
├── results/
└── README.md
```

导航配置将以 M3-2 的真实地图尺寸和窄门宽度为依据，逐步实现和验证。

## 第一步：用已有地图定位

已经确认 ROS Humble 的 `nav2_amcl`、`nav2_controller`、`nav2_planner`、
`nav2_smac_planner` 和 `nav2_regulated_pure_pursuit_controller` 安装完成。
`smart_car_navigation` 先提供定位启动入口，尚未启动自主导航。
构建时它会把 M3-2 的正式 `map.yaml`、`map.pgm` 原样安装到自身 share 目录。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-3
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch smart_car_navigation localization.launch.py gui:=false rviz:=true
```

`gui:=false` 只关闭 Gazebo 画面，Gazebo 的物理仿真仍运行；RViz 显示已保存地图。
先关闭此前运行的 M3-2 `sim_slam.launch.py`，避免两个 Gazebo 或两个 `map→odom` 发布者。

另开终端并加载 `/opt/ros/humble`、M3-1、M3-2、M3-3 四层 ROS 环境，然后检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once --field info
```

前两项应为 `active`，地图分辨率应为 0.05 m。车完全静止时，AMCL 可能
尚未发布 `/amcl_pose`；可使用 M3-2 已验证的脚本安全直行 0.3 m 来触发更新：

```bash
python3 ../M3-2/scripts/drive_route.py --straight-test 0.3 --straight-speed 0.1
ros2 topic echo /amcl_pose --once --field pose.pose.position
ros2 run tf2_ros tf2_echo map base_footprint
```

定位位置应在地图 `(0.3, 0)` 附近；TF 应持续输出变换，可按 Ctrl+C 停止查询。
M3-2 正式 bag 的首次运动时地图位姿约为 `(0.011, 0, 0°)`，所以定位配置
把固定起点初始化为 `(0, 0, 0°)`。后续若改变小车出生位置，须同步调整
AMCL 初始位姿或在 RViz 使用 `2D Pose Estimate`。

### 坐标链由谁提供

`map_server` 只把保存的地图发布给其他节点。AMCL 匹配地图和激光，发布
`map → odom`，修正轮式里程计的累计漂移。阿克曼控制器发布
`odom → base_footprint`；`robot_state_publisher` 发布车模中固定的
`base_footprint → base_link` 和雷达的安装关系。完整链条是
`map → odom → base_footprint → base_link → lidar_link`。

## 第二步：验证导航配置能规划路径

车体约 0.45 × 0.22 m；轮距 0.18 m、轴距 0.28 m，最大转角 0.5236 rad，
按 M3-2 驾驶模型算出的最小车体中心转弯半径约 0.575 m。
Hybrid-A* 规划器使用保守的 0.65 m，且只搜索可前进的 Dubins 路径。
路径跟踪选择 Regulated Pure Pursuit，并关闭原地旋转。

全局与局部代价地图使用 0.05 m/格、带约 2.5 cm 余量的车体多边形和
0.25 m 的膨胀半径。M3-2 测得的 1.15 m 门口已知空闲宽度，扣除
0.26 m 车体宽度与两侧各 0.25 m 的膨胀范围后，名义上还剩约
0.39 m；这只是规划前的几何估计，实际还须检查代价地图和行驶结果。

先关闭第一步的旧 launch，再从 M3-3 目录启动：

```bash
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch smart_car_navigation localization.launch.py gui:=false rviz:=true navigation:=true
```

`navigation:=true` 加载规划器、代价地图、路径跟踪控制器和 Nav2 生命周期管理器。
另开终端，加载同样的四层环境后检查：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
python3 scripts/check_plan.py
```

脚本仅调用 `/compute_path_to_pose` 计算从地图 `(0, 0, 0°)` 到
`(11, 3, 90°)` 的路径，**不会向 `/cmd_vel` 发送指令**。
隔离仿真预检得到 126 个路径点、约 12.52 m，包含第一个左转。
这只是路径存在性的证据，不等于小车已能安全跟随路径。
