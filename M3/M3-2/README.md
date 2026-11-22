# M3-2 激光 SLAM 建图

本题要自建含长走廊、回环和一个难点的 Gazebo 世界，用激光 SLAM 走完整圈，保存标准 `map.pgm` 与 `map.yaml`，并用墙体直线度、回环一致性和导航可用性三组证据评价地图。Gazebo 真值只用于离线评价，不参与建图。

## 场景和实现

`worlds/scene.world` 的外墙内边界为 x=±7 m、y=±5 m；中央障碍区边界为 x=±5 m、y=±3 m。环形走廊宽约 2 m，上下直段约 10 m，绕中央障碍一圈后返回起点形成回环。右侧自行设计的窄门理论净宽 1.2 m，用来检验地图栅格、墙体厚度和导航膨胀后是否仍有通道。

`scripts/drive_route.py` 使车沿走廊巡场，直线段用两侧激光墙距纠偏。`smart_car_slam` 使用 `slam_toolbox`；参数在 `config/slam_params.yaml`。地图分辨率 0.05 m/格，1.2 m 门约占 24 格；雷达 360 线、10 Hz、有效距离 0.12–8 m，因此 `max_laser_range` 不超过 8 m。正式地图在 `maps/map.pgm` 和 `maps/map.yaml`，原始录包在 `bags/formal_02/`。

## 地图质量证据

`results/quality_report.md` 基于正式地图和 rosbag：四段中央墙的拟合 RMS 偏差约 0.004–0.008 m；检查区域内未见明显分离的双层墙，横截面最大占据跨度 0.10 m；窄门中央已知空闲宽度至少 23 格，即 1.15 m。墙体跨度只是最终地图重影的代理指标，**不能单凭这张图证明 slam_toolbox 内部的回环约束已经触发**。门宽还需在 M3-3 中扣除车体和膨胀区。

`results/quality_regions.png` 标出取样位置。`results/loop_closure_compare/maps_side_by_side.png` 和同目录的 `metrics.json` 是相同 rosbag 重放、仅切换回环参数的对比；两张图须连同参数差一起解释。正式地图与比较地图不是同一趟现场新驾驶。

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
