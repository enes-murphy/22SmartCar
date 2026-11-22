# M3-3｜自主导航与调优

本阶段必须使用 M3-2 产出的地图，并针对阿克曼运动约束配置 Nav2。

## 完成清单

- [x] 解释 `map → odom → base_link` 的来源与含义。
- [x] 配置 AMCL、代价地图、阿克曼适用的规划器和控制器。
- [x] 用一条 launch 命令启动仿真、定位与 Nav2。
- [x] 选择一条大于 5 m 且包含转弯的测试路径。
- [x] 至少调 3 个参数，保留改前/改后证据。
- [x] 重复 3 次，统计成功率、耗时和失败原因。

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

## 第三步：验证控制器真的跟车，并试跑转弯路线

最初使用 Nav2 独立进程启动时，局部代价地图的车体轮廓停在仿真初始时刻：
手动前进 0.3 m 后，`/odom` 已更新，代价地图轮廓时间戳和位置却未更新。
这会造成控制器误报到点或 `Failed to make progress`。导航节点改为在
`component_container_mt` 多线程容器中运行后，同样的 0.3 m 手动测试中，
代价地图轮廓跟随到了约 0.45 m；轮廓使用 `base_link`，它比
`base_footprint` 前移 0.14 m。AMCL 和地图服务器仍使用原来的启动方式。

在全新隔离仿真中，`scripts/drive_nav_short.py` 从起点驶向地图
`(2, 0, 0°)`：里程计实际移动 1.64 m，终点 `map → base_link` 为
`(1.81, 0.02)`，距目标 0.19 m，Nav2 返回 `SUCCEEDED`。目标容差是
0.20 m。不能直接用 `/odom` 的 `base_footprint` 坐标与 `base_link` 目标
比较，否则会把固定的 0.14 m 车体偏移误认为导航误差。

接着用 `scripts/run_nav_trial.py` 首跑地图 `(11, 3, 90°)` 目标。
路径包含东向直行和左转，规划长度约 12.84 m。Nav2 返回 `SUCCEEDED`，
动作耗时 84.3 秒，终点 `map → base_link` 距目标 0.18 m。
同步记录在 `results/nav_pilot_01/`：5020 行里程计、2008 行 Gazebo 真值；
全段轨迹长度分别为 12.422 m 和 12.418 m。录制覆盖了约 100.4 秒，
包括动作前后的静止时间。`navigation_baseline.yaml` 保留本次参数，
`summary.json` 保留结果。三项参数调优和固定配置的三次复测见下一节。

## 第四步：三项参数调优与固定配置复测

本次同时修改三项控制参数；基线和新配置的完整 YAML 分别保存在
`results/nav_pilot_01/navigation_baseline.yaml` 与
`results/nav_tuned_pilot_01/navigation_tuned.yaml`。运行时通过
`ros2 param get /controller_server` 读回了新值。

| 参数 | 基线 | 新值 | 预期作用 |
| --- | ---: | ---: | --- |
| `FollowPath.desired_linear_vel` | 0.15 m/s | 0.18 m/s | 提高直线段目标速度 |
| `FollowPath.lookahead_dist` | 0.45 m | 0.40 m | 更贴近转弯路径 |
| `general_goal_checker.xy_goal_tolerance` | 0.20 m | 0.15 m | 缩小到点允许误差 |

同一地图目标 `(11, 3, 90°)` 的首跑对照：基线成功、动作耗时
84.3 s、`map → base_link` 终点误差 0.18 m；新配置成功、耗时
71.1 s、终点误差 0.13 m。耗时减少 13.2 s（约 15.7%）。两组各一次，
且三项参数同时变化，**只能说明整套新配置在这次试跑中更快且终点更近，
不能把变化归因于某一个参数，也不能据此估计基线的波动范围**。

固定新配置后，`scripts/run_repeats.py` 每次启动新的隔离 Gazebo、
AMCL 和 Nav2，从相同出生点运行同一目标，并分别保存启动日志、
动作日志、里程计、Gazebo 真值和 `summary.json`。结果保存在
`results/nav_repeats_tuned/`：

| 运行 | 成功 | 动作耗时 | `map → base_link` 终点误差 | 真值轨迹长度 |
| --- | --- | ---: | ---: | ---: |
| 1 | 是 | 71.1 s | 0.13 m | 12.399 m |
| 2 | 是 | 71.2 s | 0.13 m | 12.417 m |
| 3 | 是 | 71.3 s | 0.12 m | 12.458 m |

成功率 **3/3 = 100%**；成功运行的动作耗时 **71.2 ± 0.1 s**
（均值 ± 样本标准差）；本组三次没有导航失败原因。真值轨迹长度
平均约 **12.425 m**。这里的 100% 是三次观察结果，不能推广为
所有场景下的可靠性保证。轨迹记录覆盖动作前后的短暂静止时间。

若要重做，先关闭其他 Gazebo/导航进程，在 M3-3 目录加载四层环境并构建：

```bash
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
python3 scripts/run_repeats.py --runs 3 --out results/nav_repeats_tuned_02
```

新输出目录必须不存在。脚本自动完成三次启动、录制、导航和退出；
终端显示每次结果及总成功率，不需要手动打开三套终端。

### 关闭阶段的限制

三次动作均已先收到 `Goal succeeded`，保存了 CSV 和结果摘要；
在退出仿真时，当前 ROS Humble 的 `component_container_mt` 仍可能以
`exit code -11` 结束。独立的生命周期关停检查同样重现了这一现象，
因此不能把退出日志中的 `ERROR` 计作导航途中失败。
`run_repeats.py` 先请求导航与定位生命周期管理器关停，再结束 launch；
所有隔离试验进程均已退出。后续升级或更换 Nav2 版本时应复查关闭行为。
Nav2 上游也有[组件容器在生命周期管理器退出时崩溃的报告](https://github.com/ros-navigation/navigation2/issues/6690)；
该报告使用 Jazzy，不能直接断定这里的 Humble 崩溃有相同根因。
