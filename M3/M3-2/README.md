# M3-2｜SLAM 建图：让地图能用于导航

本阶段复用 M3-1 中已经验证的阿克曼小车、`/scan`、`/odom` 与 TF，使用
`slam_toolbox` 在自建 Gazebo 世界中生成标准占据栅格地图，并用数据评价地图质量。

## 本题最终要证明什么

1. 自建世界同时包含长走廊、回环路径和一个有明确目的的难点。
2. SLAM 只使用激光、里程计和 TF，能够生成 `map → odom` 并保存标准地图。
3. 地图质量有墙体直线度、回环一致性和导航可用性三组定量证据。
4. 参数选择与雷达特性、车辆速度和场景几何有对应关系。

## 当前进度

- [x] 读取原题并拆解硬性要求。
- [x] 安装并验证 ROS 2 Humble、Gazebo、RViz、rosbag2、`slam_toolbox` 和地图保存工具。
- [x] 设计满足尺寸约束的环形世界及 1.2 m 窄门难点。
- [x] 创建 `smart_car_slam` 包、SLAM 参数与统一启动文件。
- [x] 在 Gazebo 中实测世界、小车、控制器、雷达和 `map → odom`。
- [x] 编写并试走可复现巡场路线。
- [ ] 录制 rosbag，完成正式建图并保存 `map.pgm`、`map.yaml`。
- [ ] 自动评价墙体直线度、回环一致性和导航可用性。
- [ ] 完成单变量对比实验和最终报告。

## 世界设计

世界平面由外墙和中央矩形障碍区组成：

```text
       外墙内边界 y = +5 m
  ┌────────────────────────────┐
  │                            │
  │   ┌────────────────────┐   │
  │   │                    │   │
  │   │   中央障碍区       │ 窄│  右侧走廊
  │   │    10 m × 6 m      │ 门│  宽 2 m
  │   │                    │   │
  │   └────────────────────┘   │
  │                            │
  └────────────────────────────┘
       外墙内边界 y = -5 m
```

- 外墙内边界：`x = ±7 m`、`y = ±5 m`。
- 中央障碍区边界：`x = ±5 m`、`y = ±3 m`。
- 环形走廊宽度：`2.0 m`。
- 上下长直段：约 `10 m`，满足原题“长走廊长度 ≥ 8 m”。
- 计划路线中心线：水平段 `y = ±4 m`，竖直段 `x = ±6 m`。
- 圆角半径：`1.0 m`，高于 M3-1 算出的车辆理论下限约 `0.575 m`。
- 出生点：`(-5, -4)`，朝向东，走一圈后回到同一位置和朝向。

### 自定难点：1.2 m 窄门

右侧 2 m 宽走廊中加入两块相向挡墙，把局部净宽缩小为 `1.2 m`。这个难点用于检查：

1. 5 cm/格的地图能否清楚保留窄门；理论宽度为 `1.2 / 0.05 = 24` 格。
2. 激光噪声和地图墙体厚度是否在门口形成伪障碍。
3. 加入车辆轮廓与安全膨胀后，M3-3 是否仍有可导航通道。

它不是普通障碍物，而是有明确数值验收条件的导航瓶颈。

## SLAM 参数依据

配置位于 `config/slam_params.yaml`，当前初始值如下：

| 参数 | 初始值 | 依据 |
| :-- | --: | :-- |
| `resolution` | 0.05 m | 1.2 m 窄门对应 24 格，精度与文件规模平衡 |
| `min_laser_range` | 0.12 m | 与车模雷达有效下限一致 |
| `max_laser_range` | 8.0 m | 不超过车模雷达最大量程 |
| `minimum_time_interval` | 0.10 s | 雷达为 10 Hz，不人为降到默认的 2 Hz |
| `minimum_travel_distance` | 0.10 m | 直线约每 10 cm 加一个位姿节点 |
| `minimum_travel_heading` | 0.05 rad | 转向约每 2.9° 加一个位姿节点 |
| `loop_search_maximum_distance` | 4.0 m | 返回起点附近时允许搜索回环 |

这些是有依据的起始配置，不预先宣称是最优值。正式路线与地图数据出来后再调参，
并且一次只改变一个变量。

## 工程结构

```text
M3-2/
├── worlds/scene.world
├── config/slam_params.yaml
├── src/smart_car_slam/
│   ├── launch/sim_slam.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
├── maps/
├── bags/                  # 原始大文件不提交 Git
├── scripts/
├── results/
└── README.md
```

M3-2 是单独的 colcon 工作空间，但运行时依赖 M3-1 已安装的
`smart_car_description`。因此构建顺序必须是系统 ROS → M3-1 → M3-2。

## 当前阶段的构建与首次验证

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
colcon build --base-paths src --packages-select smart_car_slam
source install/setup.bash
ros2 launch smart_car_slam sim_slam.launch.py
```

第一次启动默认不打开 RViz，先单独验证 Gazebo 和 ROS 数据链。后续可用
`rviz:=true` 启动可视化；工程自己的 `config/mapping.rviz` 已设置俯视视角与
`/map` 显示。已有仿真运行时，可在另一终端单独执行：

```bash
rviz2 -d config/mapping.rviz --ros-args -p use_sim_time:=true
```

实测前不把“构建成功”当成“建图成功”。

## 最终交付物

```text
worlds/scene.world
config/slam_params.yaml
maps/map.pgm
maps/map.yaml
scripts/drive_route.py
scripts/eval_map.py
results/quality_report.md
results/*.png
README.md
```

正式实验还会记录 `/odom`、`/scan`、`/tf`、`/tf_static` 与 `/map`。禁止使用
`/gazebo/model_states` 修正 SLAM；Gazebo 真值最多只能作为独立的事后评价依据。

## 巡场路线预览

`scripts/drive_route.py` 复用 M3-1 已验证的 ROS 速度发布和仿真时钟逻辑。
从 `(-5, -4)` 朝东出发，依次走 `10、6、10、6 m` 四段直线，每段后以
`1.0 m` 半径左转 90°。直线初始速度 `0.20 m/s`，弯道 `0.10 m/s`，
一圈理论耗时约 `222.8` 仿真秒。先用下面的命令只查看分段，不驱动车辆：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/drive_route.py --dry-run
```

脚本采用按时间控制的开环路线，实际位置会受轮式里程计和物理接触误差影响。
正式运行前需检查起点、控制器状态；首圈观察是否擦墙或偏离窄门中心。

### 首圈试走（pilot_01）

8 段驾驶指令均完成，Slam Toolbox 生成的地图已保存为 `maps/pilot_01.pgm`
和 `maps/pilot_01.yaml`（283 × 202 格，0.05 m/格）。Gazebo 模型真值的
终点为 `(-4.955176, -4.048592)`、航向 `0.010994 rad`；相对规划起点
`(-5, -4, 0)` 的位置误差约 `0.0661 m`，航向误差约 `0.63°`。
这是路线的终点闭合误差，不等同于 SLAM 回环一致性；试走期间没有录制 rosbag。
