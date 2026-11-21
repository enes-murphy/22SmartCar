# M3-2 最终报告：激光 SLAM 建图与单变量对比

## 目标与交付

本题在 M3-1 的阿克曼小车基础上，设计一个含长走廊、可回环通道和窄门的 Gazebo 场景；使用 `/scan`、里程计和 TF 由 Slam Toolbox 建图，保存标准占据栅格地图，并用墙体、闭合和窄门指标评价地图。正式地图是 [maps/map.yaml](maps/map.yaml) 与 [maps/map.pgm](maps/map.pgm)，原始录包留在本机 `bags/formal_02/`，详细的正式地图指标在 [results/quality_report.md](results/quality_report.md)。

世界的外墙内边界为 `x=±7 m、y=±5 m`，中央岛为 `x=±5 m、y=±3 m`；上下直走廊长约 10 m，窄门设计净宽 1.2 m。地图分辨率为 0.05 m/格，设计窄门约占 24 格。小车由 `(-5,-4)` 出发，沿环形通道巡场一圈。第一段开环驾驶曾擦到中央障碍；改用雷达左右墙距反馈居中后，完成正式巡场。`/gazebo/model_states` 只在录包后用于独立评价，没有用于校正 SLAM。

## 正式地图质量

正式地图大小为 282 × 203 格，分辨率 0.05 m/格。中央四段墙在固定区域内拟合直线，RMS 偏差分别为北墙 0.007 m、南墙 0.004 m、西墙 0.008 m、避开窄门的东墙 0.007 m；这些是栅格墙点相对拟合线的离散程度，不是地图绝对定位精度。驾驶首尾 Gazebo 真值间距 0.087 m、航向差 7.77°；同期 `map→odom` 与 `/odom` 合成的地图位姿首尾间距 0.084 m、航向差 6.50°。地图中门口中央连续已知空闲宽度至少 23 格，即 1.15 m。取样长墙最大占据跨度 2 格，即 0.10 m，未见明显分离的双层墙。这些指标不能单独证明 Slam Toolbox 内部触发了回环约束；门口宽度也尚未扣除车体和代价地图膨胀。

## 单变量对比设计

为避免两次驾驶路线不同，先从同一份 `formal_02` rosbag 中制作只含 `/clock`、`/scan`、`/odom`、`/tf`、`/tf_static` 的重放包。处理时从 `/tf` 中移除原建图运行发布的 `map→odom`，避免把旧 SLAM 结果喂给新试验；`/gazebo/model_states` 和原 `/map` 均不重放。重放包生成脚本为 [scripts/prepare_slam_replay.py](scripts/prepare_slam_replay.py)。

两轮均以 3 倍速重放同一输入包，用同一版本 Slam Toolbox 的 `async_slam_toolbox_node`，其余参数相同，分别在隔离的 ROS 域中运行。**唯一有意改变的 SLAM 参数**是 `do_loop_closing`：开启见 [config/slam_params.yaml](config/slam_params.yaml)，关闭见 [config/slam_params_no_loop.yaml](config/slam_params_no_loop.yaml)。逐轮配置快照、哈希、地图及日志保存在 `results/loop_closure_compare/replay_on/` 和 `replay_off/`。可用 `diff -u` 核对两个 YAML 只有这一行不同。运行脚本是 [scripts/run_slam_replay_trial.py](scripts/run_slam_replay_trial.py)。

使用 [scripts/compare_slam_maps.py](scripts/compare_slam_maps.py) 对地图比较。该脚本先根据地图原点和尺寸，把正式地图上预先定义的四段墙及窄门取样区域平移到两张重放地图的对应世界位置，再计算墙体直线 RMS、墙体横截面最大跨度、门口连续已知空闲宽度。两个版本都使用相同的评价方法和取样区域；原始数值见 [metrics.json](results/loop_closure_compare/metrics.json)，地图并排图见 [maps_side_by_side.png](results/loop_closure_compare/maps_side_by_side.png)。

| 指标 | 开启回环 | 关闭回环 | 本次观察 |
| --- | ---: | ---: | --- |
| 北墙 RMS | 0.0079 m | 0.0074 m | 关闭略小 |
| 南墙 RMS | 0.0074 m | 0.0119 m | 开启较小 |
| 西墙 RMS | 0.0080 m | 0.0083 m | 接近 |
| 东墙 RMS，避开窄门 | 0.0089 m | 0.0000 m | 关闭较小；栅格线恰好平直，不代表整体更准 |
| 四墙 538 个采样点加权 RMS | 0.0079 m | 0.0089 m | 开启较小 |
| 取样墙体最大横截面跨度 | 0.10 m | 0.20 m | 开启的重影代理指标较小 |
| 窄门中央连续已知空闲宽度 | 1.15 m | 1.10 m | 开启多 0.05 m，即一格 |

两张地图分辨率均为 0.05 m/格；开启回环图为 282 × 203 格，关闭回环图为 283 × 203 格。这一对重放中，开启回环的地图在综合墙体 RMS、最大墙体跨度和门口已知空闲宽度上表现较好，但并非每段墙都更好。图像中的白色为已观测空闲区，黑色为占据区，灰色为未知区。关回环地图的左下边缘更厚，与最大墙体跨度指标一致。

## 结果解释与局限

此实验完成了“同数据、只改一个配置值”的比较。它支持“在这一次重放中，开启回环配置得到的地图上述若干指标较好”的结论；**不能**直接证明内部一定发生了回环闭合事件，因为没有保存明确的回环检测事件或图优化前后轨迹。它也不能把两张地图的全部差异都归因于该开关：SLAM 是异步处理，线程调度及重放速度可能影响处理时序；每组只重放一次，缺少组内波动估计。关闭回环版本的东墙残差恰为零，说明单项直线度容易受栅格取样影响，不能用一项替代整体地图质量。

正式地图的地图坐标闭合距离与车辆真值闭合距离接近，但两者不是逐点轨迹误差，也不证明回环优化事件触发。窄门 1.15 m 是原始已知空闲宽度；M3-3 后续还使用车体 footprint 与膨胀层验证规划和实际通行。M3-3 在这张正式地图上完成超过 5 m、含转弯的自主导航，为地图可用于该固定路线提供了独立的下游证据。

## 复现步骤

在 M3-2 目录中运行，先加载 ROS 2 Humble。原始 `bags/formal_02/` 需要在本机；因体积约 50 MiB，Git 中只保留代码、地图、报告及评价结果，不提交原始录包。每个输出目录必须不存在；重做时换新的目录名，并在 `compare_slam_maps.py` 中通过 `--on`、`--off` 指向新地图。

```bash
source /opt/ros/humble/setup.bash
python3 scripts/prepare_slam_replay.py --bag bags/formal_02 --out bags/slam_replay_inputs_02
```

上面会创建一个新的重放包；两轮都通过 `--bag` 指向它，确保输入相同：

```bash
python3 scripts/run_slam_replay_trial.py --bag bags/slam_replay_inputs_02 \
  --params config/slam_params.yaml \
  --out results/loop_closure_compare/replay_on_02 --domain 173 --rate 3
python3 scripts/run_slam_replay_trial.py --bag bags/slam_replay_inputs_02 \
  --params config/slam_params_no_loop.yaml \
  --out results/loop_closure_compare/replay_off_02 --domain 174 --rate 3
python3 scripts/compare_slam_maps.py \
  --on results/loop_closure_compare/replay_on_02/map.yaml \
  --off results/loop_closure_compare/replay_off_02/map.yaml \
  --out results/loop_closure_compare/metrics_02.json
```

运行脚本将 ROS 日志目录放在各自输出目录内，并使用独立 ROS 域及本机 DDS 通信，避免干扰正在运行的仿真。实验保存完成后，它会正常结束 SLAM 节点。
