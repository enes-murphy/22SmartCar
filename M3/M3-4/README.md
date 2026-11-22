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

## 现场演示速用流程

以下命令在**一个新 Ubuntu 终端**依次执行。Gazebo 是第二个窗口，用来观察车和箱子，不需要在那里手动放模型。`run_experiment.py` 自动启动 Gazebo、定位和导航；`--gui` 显示 Gazebo。每次给 `--out` 一个不存在的新目录。演示前先关闭其他 Gazebo/Nav2 会话，避免混用仿真。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-4
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
source ../M3-3/install/setup.bash
DEMO="results/live_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config short --runs 1 --delay 3 --world-x -1 --world-y -3.35 --size-x .30 --size-y .30 --gui --max-preflight-attempts 4 --first-domain 180 --out "$DEMO"
cat "$DEMO/summary.json"
cat "$DEMO/run_01/trial.log"
```

按 Enter 后，小车**不会立即开动**：程序先启动 Gazebo、AMCL 定位和 Nav2，再做短距离预热。待 `trial.log` 出现 `Goal accepted at sim ...`，导航才开始；小车应开始向目标行驶。随后日志出现 `Obstacle inserted at sim ...`，Gazebo 中应看到新箱子。等终端重新出现 `$`，再运行上面的两条 `cat` 查看结果。`summary.json` 的 `successes` 为 1 表示到点；为 0 则查看 `run_01/summary.json` 中的 `outcome` 和 `trial.log`。运行中先不要重复输入同一命令或手动发第二个目标。

Gazebo 窗口展示接近、出现障碍、改道、通过，现场截图或录屏保存。脚本目前从**目标被接受**起计 3 个仿真秒，模型服务实际完成会晚少许；口述时按 `trial.log` 中的两条时间戳说明。2026-11-22 彩排中，目标在仿真 27.110 秒被接受，箱子在 30.250 秒生成，相差 3.14 秒，车成功到点；记录在 `results/live_rehearsal_20261122/`。历史 `results/short_10`、`long_10`、`long_tuned_10` 是 **8 秒**条件，不要说成现场的 3 秒结果。另有 3 秒条件的图在 `results/guide2_*`。

考官指定新位置时，先要到 **Gazebo 世界坐标**，在同一终端输入（下面的 `BOX_X`、`BOX_Y` 换成考官给的数；两次保持条件相同）：

```bash
BOX_X=-1.3
BOX_Y=-3.45
RANDOM_OUT="results/random_live_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config short --runs 2 --delay 3 --world-x "$BOX_X" --world-y "$BOX_Y" --size-x .30 --size-y .30 --gui --max-preflight-attempts 4 --first-domain 190 --out "$RANDOM_OUT"
cat "$RANDOM_OUT/summary.json"
cat "$RANDOM_OUT/run_01/summary.json"
cat "$RANDOM_OUT/run_02/summary.json"
```

`--runs 2` 会依次启动两次独立仿真；每次结束后才进入下一次，不要另外开终端发车。2026-11-22 用示例坐标彩排时，第一次成功、第二次失败；这仅验证现场输入与记录流程，不能代替考官指定的随机坐标。若考官允许第一次后调整参数，应改用两个 `--runs 1` 的新输出目录，并分别保存两次配置，不把它们当作同配置重复试验。

展示事先完成的两方案十次数据、调优十次数据和四指标图：

```bash
cat results/short_10/summary.json results/long_10/summary.json results/long_tuned_10/summary.json
xdg-open results/guide2_four_metrics.png
xdg-open results/guide2_trajectory_comparison.png
```

三份 `summary.json` 是原先的 **8 秒**批次；两张 `guide2_` 图片是之后的 **3 秒**批次。讲解时先指出图与表属于不同条件，不要拼成同一组统计。两方案采用同一 RPP 控制器的 `0.30 m` 和 `0.80 m` 前视距离；原题允许这种“同一控制器、两套极端参数”的比较。成功率用各组全部十次；平均耗时、额外路径、平滑度只用成功轨迹。额外路径的题目定义是实际轨迹长度减**无障碍实际路线**长度，不能用起终点直线距离冒充。

最后现场复现已观察过的较难条件。世界 `(-1,-3.6)`、`0.35 m` 箱体在 2026-11-22 彩排中使目标失败；独立重试有随机性，若成功也要照实展示。

```bash
FAIL_OUT="results/failure_live_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config short --runs 1 --delay 3 --world-x -1 --world-y -3.6 --size-x .35 --size-y .35 --gui --max-preflight-attempts 4 --first-domain 200 --out "$FAIL_OUT"
cat "$FAIL_OUT/run_01/summary.json"
cat "$FAIL_OUT/run_01/trial.log"
```

脚本在有效导航失败时可能以退出码 1 结束；`run_01/summary.json` 和 `trial.log` 才是判读依据。起跑前预检失败会保留在 `attempts/` 并重试，不计为避障试次。
