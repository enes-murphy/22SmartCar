# M3-4：导航中出现障碍物

M3-3 已能沿 M3-2 保存的地图导航。本题在车驶向目标时，把箱子放进前方走廊，观察规划器多久改道、控制器能否跟上，以及参数改变会带来什么代价。数值结果见 [report.md](report.md)，逐次记录见 `results/`。

## 场景与比较方法

每次都从 Gazebo 世界 `(-5, -4)` 朝东出生，使用 M3-2 的地图和 AMCL 定位，驶向地图坐标 `(8.0, 0.4, 0°)`。导航目标接受 **8 个仿真秒**后，脚本在世界坐标 `(-1.0, -3.35)` 放入静止箱体，长宽各 `0.30 m`，高 `0.50 m`。箱体偏向走廊北侧，靠近正常行驶带并进入代价地图安全区域；成功轨迹通常从它南侧通过。世界坐标与地图坐标原点不同；箱体在地图上约为 `(4.0, 0.65)`。

全局规划器沿用 M3-3 的 Hybrid-A*，局部控制器沿用 Regulated Pure Pursuit。两套正式配置只改变 `FollowPath.lookahead_dist`：短前视 `0.30 m`，长前视 `0.80 m`；目标速度同为 `0.18 m/s`。完整参数在 `config/`。这比较的是**同一个控制器的两套前视参数**，不是两种插件。前视距离决定沿规划路径向前取多远的跟踪点；太短可能频繁修正，太长可能较晚贴合改道后的路径。哪种表现更好由重复试验决定。

三组各 10 次的结果分别为 **8/10、7/10、7/10 成功**。将较差的长前视组从 `0.80 m` 调至 `0.50 m` 后，成功率未提高。完整数值和解释见 [report.md](report.md)。

## 重做实验

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic，以及已构建的 M3-1、M3-2、M3-3 工作空间。先关闭其他 Gazebo、RViz 和 Nav2 会话，再执行：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-4
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
source ../M3-3/install/setup.bash
python3 scripts/run_experiment.py --config short --runs 10 --out results/short_10 --first-domain 10
python3 scripts/run_experiment.py --config long --runs 10 --out results/long_10 --first-domain 10
python3 scripts/run_experiment.py --config long_tuned --runs 10 --out results/long_tuned_10 --first-domain 10
```

三组命令依次运行。每次试验单独启动 Gazebo、AMCL 和 Nav2，并使用隔离的 ROS domain 与 Gazebo 端口。先让小车慢行 `0.3 m`，建立实时 `map→odom` 后再启动 Nav2；随后再慢行 `0.3 m`，检查局部和全局代价地图的车体轮廓是否跟着移动。轮廓冻结时，该次在发出导航目标**之前**被拒绝，日志留在 `attempts/`，程序重新启动。统计分母只包含 `run_01` 到 `run_10`；连续 20 次预检不过则报错停止，不把环境故障算作避障失败。

输出目录须事先不存在。可用 `--world-x`、`--world-y`、`--size-x`、`--size-y`、`--delay` 修改障碍位置、尺寸和出现延迟；改动后属于另一实验场景，不应混入同条件对比。每次保存 `summary.json`、`trial.log`、参数读回、Gazebo 真值轨迹、里程计、速度指令、激光距离、代价地图诊断和每条 `/plan` 路径。Gazebo 真值仅供**事后评价**，导航器本身只使用地图、AMCL、激光与里程计。

离线重算指标无需启动仿真：

```bash
python3 scripts/analyze_experiment.py results/short_10
python3 scripts/analyze_experiment.py results/long_10
python3 scripts/analyze_experiment.py results/long_tuned_10
python3 scripts/plot_results.py results/short_10 results/long_10 results/long_tuned_10 --out results/trajectory_comparison.png
```

结果写入各组的 `analysis.json`。每组的 `navigation.yaml` 是运行时配置快照，`parameter_readback.txt` 是节点实际读到的值。

## 指标定义

| 指标 | 计算方法 |
| --- | --- |
| 成功率 | 10 次通过预检的试验中，箱体确实生成、Nav2 返回 `SUCCEEDED`，且终点 `map→base_link` 距目标不超过 `0.25 m` 的比例。 |
| 耗时 | 从目标被接受到动作返回的仿真时间；不包括启动和预热。墙钟时间另记，供排障。 |
| 额外路径 | Gazebo 真值轨迹长度减去同次起终点的直线距离，表示绕行与跟踪偏差多走的路。 |
| 平滑度 | 真值轨迹按约 `0.1 m` 间距取样，计算相邻曲率变化的均方根；越小表示转弯变化越缓。 |
| 首条新路径延迟 | 箱体生成到下一条 `/plan` 发布的仿真时间。它只证明又发布了路径。 |
| 稳定绕行延迟 | 箱体附近路径相对出现前横移超过 `0.10 m`，连续两条路径都满足时取第一条的时间；没有达到条件就记缺失。 |

组均值中的耗时、额外路径和平滑度只统计成功运行；失败次数与原因单列，避免中途停止形成虚假的“短路径优势”。

```text
M3-4/
├── config/          # 完整导航配置
├── scripts/         # 障碍注入、独立重复试验、离线分析
├── results/         # 正式试验与预检诊断
├── report.md        # 数据解释、局限与失效边界
└── README.md
```
