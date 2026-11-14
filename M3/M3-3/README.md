# M3-3｜自主导航与调优

本阶段必须使用 M3-2 产出的地图，并针对阿克曼运动约束配置 Nav2。

## 完成清单

- [ ] 解释 `map → odom → base_link` 的来源与含义。
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
