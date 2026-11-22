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
- [x] 录制 rosbag，完成正式建图并保存 `map.pgm`、`map.yaml`。
- [x] 自动评价墙体直线度、回环一致性和窄门几何可用性。
- [x] 完成同录包的回环开关单变量对比实验和最终报告。

最终报告见 [report.md](report.md)：同一份正式录包重放两次，只改变
`do_loop_closing`，并比较墙体直线度、墙体跨度和窄门空闲宽度。
`results/loop_closure_compare/` 保留两张地图、配置快照、原始指标与并排图。

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

## 从零复现一次成功建图（三个 Ubuntu 终端）

下面是 2026-11-22 实际跑通的流程：启动 Gazebo 和 Slam Toolbox，用激光居中路线绕场地一圈，同时录制 rosbag，最后把实时 `/map` 保存成 PGM 和 YAML。**终端 A、B、C 按用途命名，不要求按字母顺序打开。** 每个终端都要自己 `source` 环境；命令都在 Ubuntu 中运行，不需要 Windows。请先结束旧的 M3-1/M3-2 Gazebo 仿真，避免两个世界或两台同名小车同时运行；已经运行过一圈的世界也要重启，才能让车回到 `(-5, -4)` 起点。

### 终端 A：构建并启动仿真，整圈期间保持运行

第一次运行或切换过 Git 版本后，按顺序构建车模包和建图包：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select smart_car_description
source install/setup.bash
cd ../M3-2
colcon build --base-paths src --packages-select smart_car_slam
source install/setup.bash
ros2 launch smart_car_slam sim_slam.launch.py rviz:=true
```

`colcon` 将源码安装到工作区的 `install/`，`source` 让当前终端找到刚构建的 ROS 包。最后一行同时启动 Gazebo（物理仿真）、小车及阿克曼控制器、Slam Toolbox（用激光和里程计建图）与 RViz（显示 `/map`）。A 不返回 `$` 是正常的。等日志出现 `Successfully spawned entity [smart_car]`、两个控制器的 `Configured and activated`，再继续。Gazebo 中的小车外观很小；可以展开左侧 **Models** 查找 `smart_car`，不必只凭画面中是否看见车判断启动成功。

### 终端 B：核对数据源，再开始录包，保持运行

新开一个 Ubuntu 终端 B，输入：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source install/setup.bash
ros2 topic list | grep -E '^/(cmd_vel|gazebo/model_states|odom|scan|map|tf|tf_static)$'
ros2 topic echo /gazebo/model_states --once --field name
ros2 topic info /map
```

应看到 `smart_car` 出现在模型名称列表，并且 `/map` 的 `Publisher count: 1`。其中 `/scan` 是激光数据，`/odom` 是轮式里程计，`/tf` 是坐标变换，`/map` 是 SLAM 生成的实时地图。若没有小车或 `/map`，回 A 检查启动日志，先不要驾驶。确认后，**仍在 B** 输入：

```bash
RUN_TAG="reproduce_$(date +%Y%m%d_%H%M%S)"
echo "$RUN_TAG"
ros2 bag record --use-sim-time -o "bags/$RUN_TAG" /clock /cmd_vel /odom /scan /tf /tf_static /map /map_metadata /gazebo/model_states
```

`ros2 bag` 相当于 ROS 数据录音机；带时间后缀的目录避免覆盖旧试验。看到 `Recording...`、`Sim time /clock found`，以及主要话题的 `Subscribed to topic` 后，**让 B 一直录制，不要在 B 输入驾驶命令**。`/cmd_vel` 在驾驶脚本启动后才会出现，录包程序随后会自动订阅。关于 `/tf` 的 durability 警告不妨碍其继续录制；`/gazebo/model_states` 只作为事后真值对照，不输入 SLAM。

### 终端 C：开车走一圈，等程序自行停车

再开一个 Ubuntu 终端 C，输入：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source install/setup.bash
python3 scripts/drive_route.py --wall-follow --laps 1
```

`--wall-follow` 用左右激光墙距修正直行方向；转弯仍按既定半径和角速度执行。路线是四段直线与四次左转，理论约 **222.8 仿真秒**。窄门处左右测距会接近 `0.6 m`。等待 `Lap 1, edge 4: finished left turn` 和 `Run complete; zero velocity command sent` 出现，C 回到 `$` 后再进入下一步。运行中不要暂停 Gazebo，也不要再启动第二个驾驶脚本。若出现碰撞或安全停车，保留日志并先排查，不要把该次当成完整地图；下一次试跑从重启 A 的仿真开始。

### 回到终端 B：停止录包并保存地图

先在**录包的 B** 按一次 `Ctrl+C`，等待 `Writing remaining messages from cache` 和 `Recording stopped`，B 重新出现 `$`。然后在**同一个 B** 输入：

```bash
ros2 run nav2_map_server map_saver_cli -f "maps/$RUN_TAG" --ros-args -p use_sim_time:=true
ls -lh "maps/$RUN_TAG.pgm" "maps/$RUN_TAG.yaml"
ros2 bag info "bags/$RUN_TAG"
```

`map_saver_cli` 将当前 `/map` 保存成占据栅格图 `.pgm` 和描述分辨率、原点的 `.yaml`；`ros2 bag info` 只检查录包的时长、消息数和话题，不会播放它。看到 `Map saved successfully`、两个地图文件存在，且 bag 中有 `/scan`、`/odom`、`/map`，才算完成这次复现。本次实测得到 **282 × 202 格、0.05 m/格**的地图和约 **245 秒、77,608 条消息**的 bag；每次运行的具体数字会略有差异。

此时 A 仍在运行，可继续看 Gazebo 和 RViz。实时建图阶段由 Slam Toolbox 发布 `/map`，**不需要另外启动 `map_server`**。结束展示时，到 A 按一次 `Ctrl+C`，等仿真和图形窗口退出。下次重跑要重新启动 A，让小车回到起点。时间后缀地图和 bag 是新试验记录；原有 `maps/map.yaml`、`maps/map.pgm` 与 `bags/formal_02/` 不会被覆盖。

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

## 正式试次 02

使用激光居中路线完成一圈。原始录包 `bags/formal_02/` 保留在本机，
约 50.2 MiB、105533 条消息，含 `/odom`、`/scan`、`/tf`、`/tf_static`、
`/map`、`/clock`、`/cmd_vel` 和仅供事后评价的 `/gazebo/model_states`。
保存 `maps/map.pgm` 与 `maps/map.yaml`，尺寸 282 × 203 格，分辨率 0.05 m/格。
根据 bag 中驾驶首尾附近的 Gazebo 真值，起点约 `(-4.997671, -3.979444)`，
终点约 `(-4.911313, -3.985065)`，位置闭合误差约 `0.08654 m`；
航向从约 `-1.482°` 到等效 `+6.288°`，相差约 `7.77°`。
这些是车辆真实轨迹的闭合指标，不是地图墙体直线度或 SLAM 回环指标。

运行离线评价（不需要 Gazebo）：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/eval_map.py
```

脚本读取地图和本地 rosbag，生成 `results/quality_report.md`。它对中央四条墙边
避开角点拟合直线，东边还避开自建窄门，报告 RMS 和 95% 残差；
取墙体横截面跨度检查明显双层重影；对门中央连续已知空闲格计数；
合成 `map→odom` 与 `/odom` 位姿计算地图坐标系下的首尾闭合距离。
RMS 小于单格尺寸只是多个栅格点拟合的残差，不代表绝对位置有毫米精度。
最终地图未显双墙、首尾接近，也不能单凭它们证明 SLAM 内部的回环约束触发。
完整数值和局限见 `results/quality_report.md`。

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

### 正式试次 01：第一段碰撞，保留诊断数据

原始录包保存在本地 `bags/formal_01/`（约 49 MiB，不提交 Git），包含
`/clock`、`/cmd_vel`、`/odom`、`/scan`、`/tf`、`/tf_static`、`/map` 和
`/map_metadata`。脚本在第一段直线发送 `0.20 m/s`、`0 rad/s`；里程计在
第 20 秒约前进 4 m，第 30 秒仅到约 4.8 m，表明车开始受阻。
Gazebo 终点真值 `(-0.441707, -3.200778, 0.261938 rad)`，相对起点
侧移约 `+0.799 m`、朝向偏离约 `+15.0°`，车身接触中央障碍区南侧。
同时里程计报告 `(5.714836, -0.003172, -0.037366 rad)`，未反映实际侧移。
这说明需要先诊断物理行驶与轮式里程计的差异；本试次不作为最终建图结果。

### 低速 3 m 直行诊断

`straight_slow_02_start` 的 Gazebo 真值为
`(-4.994644, -4.001787, 0.043661 rad)`；以 `0.10 m/s`、零转向命令
直行 30 秒后，`straight_slow_02_end` 的真值为
`(-1.993726, -3.667769, 0.107505 rad)`。真实侧移 `+0.334 m`，
航向增加 `0.063844 rad ≈ 3.66°`，但里程计横向几乎为零。
因此低速开环仍会向中央障碍区偏移；下一步需用雷达测量两侧墙距，
为直行段提供闭环纠偏。`straight_slow_01_start` 是旧仿真碰撞后的状态，
不可用作新试次起点。

### 激光居中 3 m 对照试验

`scripts/drive_centered_test.py` 以左右 90° 雷达墙距之差为反馈量，
直行时用 `angular.z = clamp(0.30 × (left - right), -0.12, 0.12)` 微调转向；
侧墙测距失效或前方障碍小于 0.5 m 时停车。以 `0.10 m/s` 行驶 3 m，
Gazebo 真值从 `(-4.998929, -4.004224, 0.020575 rad)` 变为
`(-1.995881, -3.998545, 0.006345 rad)`：横向偏移 `+0.005679 m`，
对比同速开环的 `+0.334 m` 约减少 98.3%。这是一次试验，尚不能代表
多次重复实验的均值或稳定性；正式路线速度 `0.20 m/s` 尚待验证。

随后以 `0.20 m/s` 重做 3 m 居中直行：真值从
`(-4.999718, -4.001918, 0.009025 rad)` 到
`(-1.992433, -4.003812, -0.000883 rad)`，横向偏移 `-0.001894 m`。
这验证了目标直线速度下的短距离效果。整圈试走可用
`python3 scripts/drive_route.py --wall-follow`：直行段依据左右墙距纠偏，
转弯段仍使用原定角速度；拐角暂时缺少一侧墙时保留名义直行命令。
整圈碰撞安全性与最终地图质量仍需实测。

### 第一处转角试走

`python3 scripts/drive_route.py --wall-follow --first-corner` 完成 10 m 直线
和首个 90° 左转。Gazebo 真值终点为 `(5.925619, -2.922714, 1.645839 rad)`，
距计划 `(6, -3)` 约 `0.107 m`，朝向约 `94.3°`（计划 `90°`）。
车辆位于右侧走廊、窄门下方。这个检查验证首个转角可通行，
尚未验证窄门和剩余三个转角。

### 激光居中整圈试走

`python3 scripts/drive_route.py --wall-follow --laps 1` 完成四条直线和
四次左转，右侧窄门处测得左右距离约 `0.60 m` 和 `0.63 m`。
Gazebo 真实终点为 `(-4.889141, -3.982283, 0.068129 rad)`，相对规划起点
的位置闭合误差约 `0.1123 m`，航向误差约 `3.90°`。地图保存为
`maps/wall_follow_pilot.pgm` 和 `.yaml`（292 × 231 格，0.05 m/格）。
地图中走廊和窄门可辨认，但长墙边缘存在厚度和局部重影，仍需正式录包后
定量评价墙体直线度与回环一致性。
