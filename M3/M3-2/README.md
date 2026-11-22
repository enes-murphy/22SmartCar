# M3-2 激光 SLAM 建图

本题要自建含长走廊、回环和一个难点的 Gazebo 世界，用激光 SLAM 走完整圈，保存标准 `map.pgm` 与 `map.yaml`，并用墙体直线度、回环一致性和导航可用性三组证据评价地图。Gazebo 真值只用于离线评价，不参与建图。

## 场景和实现

`worlds/scene.world` 的外墙内边界为 x=±7 m、y=±5 m；中央障碍区边界为 x=±5 m、y=±3 m。环形走廊宽约 2 m，上下直段约 10 m，绕中央障碍一圈后返回起点形成回环。右侧自行设计的窄门理论净宽 1.2 m，用来检验地图栅格、墙体厚度和导航膨胀后是否仍有通道。

`scripts/drive_route.py` 使车沿走廊巡场，直线段用两侧激光墙距纠偏。`smart_car_slam` 使用 `slam_toolbox`；参数在 `config/slam_params.yaml`。地图分辨率 0.05 m/格，1.2 m 门约占 24 格；雷达 360 线、10 Hz、有效距离 0.12–8 m，因此 `max_laser_range` 不超过 8 m。正式地图在 `maps/map.pgm` 和 `maps/map.yaml`，原始录包在 `bags/formal_02/`。

## 地图质量证据

`results/quality_report.md` 基于正式地图和 rosbag：四段中央墙的拟合 RMS 偏差约 0.004–0.008 m；检查区域内未见明显分离的双层墙，横截面最大占据跨度 0.10 m；窄门中央已知空闲宽度至少 23 格，即 1.15 m。墙体跨度只是最终地图重影的代理指标，**不能单凭这张图证明 slam_toolbox 内部的回环约束已经触发**。门宽还需在 M3-3 中扣除车体和膨胀区。

`results/quality_regions.png` 标出取样位置。`results/loop_closure_compare/maps_side_by_side.png` 和同目录的 `metrics.json` 是相同 rosbag 重放、仅切换回环参数的对比；两张图须连同参数差一起解释。正式地图与比较地图不是同一趟现场新驾驶。

## 实际工作过程与各程序的作用

1. **检查输入链。** M3-1 的车模和控制器提供 `/odom`、`/tf` 与 `lidar_link` 上的 `/scan`。Slam Toolbox 用激光扫描、里程计和 TF 估计车在地图上的位置，并持续发布 `/map`；Gazebo 提供物理场景。我们曾用 `ros2 topic list`、`ros2 topic echo /scan --once --field header.frame_id` 和 `ros2 topic echo /map --once --field info` 确认输入输出确实连通，扫描坐标系为 `lidar_link`，地图分辨率是 0.05 m/格。
2. **设计可检验的世界。** 自建 [`worlds/scene.world`](worlds/scene.world) 含环形走廊、中央障碍和右侧窄门。长直墙便于量化直线度；绕中央障碍回到起点可检验闭合一致性；窄门检验地图是否保留可通行空间。地图的 `map` 坐标原点由 SLAM 估计，不能把 Gazebo 世界坐标、`odom` 和 `map` 的数值直接当成同一坐标。
3. **先发现驾驶问题，再用传感器修正。** 起初按固定时长开环直行，第一段偏向墙体；另一次 3 m、0.10 m/s 的直行从真值约 `(-4.995,-4.002)` 到 `(-1.994,-3.668)`，横向偏离约 0.33 m。通过 [`scripts/inspect_scan.py`](scripts/inspect_scan.py) 检查前、左、右激光距离，再在直线段用左右墙距差给小角速度修正。3 m 的居中测试结束在约 `(-1.996,-3.999)`，较快的 0.20 m/s 测试也在约 `(-1.992,-4.004)`。这证明了该直段的反馈效果，不能推论所有场景都会居中成功。
4. **完整巡场并保存原始证据。** 车沿环形走廊走完一圈，`ros2 bag` 录下 `/clock`、`/cmd_vel`、`/odom`、`/scan`、`/tf`、`/tf_static`、`/map`、`/map_metadata` 和 `/gazebo/model_states`。录包结束日志显示 `Recording stopped` 后，再用 `map_saver_cli` 从 `/map` 保存标准 PGM 图像与 YAML 元数据。最终交付的是 [`maps/map.pgm`](maps/map.pgm) 和 [`maps/map.yaml`](maps/map.yaml)；`pilot_01`、`wall_follow_pilot` 是排查过程中的早期地图，不应当作正式结果。原包约 50 MiB，保留本机而未提交 Git。
5. **离线评价与单变量试验。** [`scripts/eval_map.py`](scripts/eval_map.py) 对正式地图和 bag 做墙体、闭合与窄门评价。然后 [`scripts/prepare_slam_replay.py`](scripts/prepare_slam_replay.py) 从同一个 bag 制作可重放输入，排除旧 `/map`、Gazebo 真值，并从 `/tf` 移除原有的 `map→odom`，避免新建图受旧结果污染；两次重放仅将 `do_loop_closing` 从 `true` 改成 `false`。结果、配置和详细局限见 [`report.md`](report.md)。

## 三类质量指标分别回答什么

| 指标 | 正式地图的结果 | 能说明什么；不能说明什么 |
| --- | --- | --- |
| 墙体直线度 | 中央北、南、西、东墙拟合 RMS 分别约 0.007、0.004、0.008、0.007 m | 这些是选定墙段的栅格点相对拟合直线的离散程度；不是地图绝对定位误差。 |
| 回到起点的一致性 | 真值首尾距离 0.087 m，地图坐标下融合位姿首尾距离 0.084 m；所查墙段最大占据跨度 0.10 m | 可描述这次路线的首尾闭合和最终地图重影；不能单凭它证明内部发生了回环图优化。 |
| 窄门导航空间 | 中央连续已知空闲至少 23 格，即 1.15 m | 表示原始地图在门口留下空闲栅格；还须考虑车身宽度、定位误差和代价地图膨胀。 |

同一包的回环开/关对比中，四墙加权 RMS 为 **0.0079 / 0.0089 m**，所查墙段最大占据跨度为 **0.10 / 0.20 m**，门口已知空闲宽为 **1.15 / 1.10 m**。斜杠前为开启、后为关闭。此配对支持“这一次重放开启配置在这些综合指标上较好”；有的单段墙恰好相反，且每组只重放一次、没有直接记录回环事件。不要把参数开关本身说成回环事件已触发的证据，也不要把这组地图说成两次独立实车驾驶。

## 按展示指南现场执行

**考前｜终端 A。** 在 M3-2 目录构建，确认笔记本离线仍能启动 Gazebo；预热一次后按 Ctrl+C 退出。准备 `bags/formal_02/`、地图和提前录好的建图过程录屏。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch smart_car_slam sim_slam.launch.py
```

**1 看自建世界｜终端 A；文本编辑器或 Gazebo 窗口。**

```bash
gedit worlds/scene.world
```

指给考官看：约 10 m 的长直段、绕中央障碍的回环、右侧 1.2 m 窄门；说明窄门预期使激光测量和占据栅格边缘出现厚度或伪影，实际评价见质量报告。

**2 起 SLAM 仅验证可运行｜终端 A，保持运行。** 关闭可能存在的旧仿真，执行：

```bash
ros2 launch smart_car_slam sim_slam.launch.py
```

**终端 B** 新开后加载环境，再检查话题：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source install/setup.bash
ros2 topic list | grep -E '^/(scan|odom|map|tf)$'
```

展示提前录好的操作录屏和 `ros2 bag info bags/formal_02`；录屏是建图过程证据，不替代**现场加载保存地图**。验证后在 A 按 Ctrl+C，等 SLAM 和 Gazebo 完全退出，避免两个 `/map` 发布者。

**3 现场从保存文件加载地图｜终端 A，保持运行。** 在 M3-2 目录执行指南指定的 `map_server` 方式，文件名用本机绝对路径：

```bash
ros2 run nav2_map_server map_server --ros-args -p "yaml_filename:=$PWD/maps/map.yaml"
```

它是生命周期节点，单独启动后还需在 **终端 B** 执行：

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo /map --once --field info
```

应看到地图分辨率约 0.05 m。**终端 C** 新开 Ubuntu 终端，加载 ROS 后打开 RViz：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
rviz2 -d config/mapping.rviz
```

在 RViz 中确认 Fixed Frame 为 `map`、Map 话题为 `/map`，指给考官看长墙、回环区域和窄门。此处显示的是从磁盘 `map.pgm`/`map.yaml` 加载的地图，并非重新建图。

**4 看定量质量和单变量对比｜终端 B；图像窗口。**

```bash
cat results/quality_report.md
cat results/loop_closure_compare/metrics.json
xdg-open results/quality_regions.png
xdg-open results/loop_closure_compare/maps_side_by_side.png
```

按“墙直度 → 是否双墙/回环一致性 → 窄门已知空闲宽度”讲三组数据，并说清对比实验只改变回环开关。结束时先关 RViz，在 A 按 Ctrl+C 关闭地图服务器。
