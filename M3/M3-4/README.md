# M3-4 动态障碍下的避障

本题要求车出发后才出现可参数化的障碍，公平比较两种局部方案各 10 次，报告成功率、成功运行耗时、绕行额外路径和平滑度；对较差方案调优再跑 10 次，并解释失效边界和路径重规划。

## 已完成的实验及其适用范围

沿用 M3-2 地图和 M3-3 的 AMCL、Hybrid-A* 全局规划及 Regulated Pure Pursuit 局部控制。现有正式三组均从同一出生点去地图 `(8.0, 0.4, 0°)`，目标接受 **8 个仿真秒后**在 Gazebo 世界 `(-1.0, -3.35)` 加入 `0.30×0.30×0.50 m` 箱体。箱体在正常行驶带北侧附近，没有完全堵住通道。两套比较方案是**同一控制器的两套极端前视参数**，这符合原题可选方式 B；不能称为两种不同插件。

| 前视距离 | 成功数 | 成功运行耗时 | 额外路径 | 曲率变化 RMS |
| --- | ---: | ---: | ---: | ---: |
| 0.30 m | 8/10 | 43.24 ± 1.54 s | 0.0222 ± 0.0024 m | 0.259 ± 0.039 1/m |
| 0.80 m | 7/10 | 42.33 ± 1.04 s | 0.0214 ± 0.0023 m | 0.086 ± 0.007 1/m |
| 0.50 m 调优 | 7/10 | 42.03 ± 0.51 s | 0.0245 ± 0.0024 m | 0.164 ± 0.007 1/m |

每组均值仅取成功运行；失败仍计入十次分母。0.50 m 是对 0.80 m 的单变量调整，未提高成功数。逐次原始记录在 `results/{short_10,long_10,long_tuned_10}/`，四问分析在 `report.md`。当前“额外路径”是**真值轨迹长度减同次起终点直线距离**，不是原题提出的“实际路径减无障碍实际路线”；两种口径不能混称。首条新 `/plan` 的延迟也不等于稳定绕行延迟，详见报告。

## 展示指南补充试跑与待现场完成的部分

展示指南要求现场 **3 秒后**注入、考官随机指定位置跑 **2 次**、展示至少四帧过程、四指标对比图并现场复现失效。现有 8 秒固定位置的 30 次数据不能冒充这些现场项目；随机位置也不能保证成功。指南自查表写“两种规划器”，而原题允许同一局部控制器的两套极端参数；本实现采用后者。2026-11-22 已用 `guide_short` 配置验证目标接受约 3 秒后注入：`results/guide_pilot_offset_20261122/run_01/` 成功，到达耗时 42.86 s；`results/guide_pilot_odom_20261122/run_01/` 的 0.35 m 箱体有效触发失败。前一条成功记录生成了 `process_frames/` 四张按真值和规划路径重建的分时图；它们是**数据可视化，不是 Gazebo 现场截图**，考核还应现场截屏。

原 8 秒三组数据的四指标对比图在 `results/four_metric_comparison.png`。其中第三项是“实际路径减同次起终点直线距离”的冗余量，**不能称为相对无障碍路线的绕行额外路径**。为此虽试算了一条无障碍参考记录，但它与这三组的路径长度不相容，未用来替换正式指标。另已启动 3 秒十次批次 `results/guide_short_10_v2_20261122/`，目前只完成第 1、2 次，其余未计为已完成；后续要用 `--resume` 跑满。考官随机给位置的两次测试必须现场执行。

## 按展示指南现场执行

**考前｜终端 A。** 确认 M3-1、M3-2、M3-3 已构建；关闭其他手工启动的 Gazebo/Nav2。M3-4 是脚本目录，无须再运行 `colcon build`。预热 Gazebo 后关掉。正式原始数据、`report.md`、配置及失败日志都准备在本机，断网可读。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-4
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
source ../M3-3/install/setup.bash
```

`scripts/run_experiment.py` 会自动启动仿真、定位和导航；现场不要再手动起另一套。`--gui` 会显示 Gazebo 供考官观察；不加则后台运行。每次输出目录必须是新目录。

**1 发目标、3 秒后注入｜终端 A；Gazebo 窗口。**

```bash
DEMO="results/live_3s_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config guide_short --runs 1 --delay 3 --world-x -1.0 --world-y -3.35 --size-x 0.30 --size-y 0.30 --gui --out "$DEMO" --first-domain 200
```

脚本在预检通过后发送目标，目标被接受后计 **3 个仿真秒**再调用 Gazebo 生成模型服务；这是“目标接受后 3 秒”的具体定义，不把它说成精确“车轮开始转后 3 秒”。Gazebo 中依次展示接近、检测、绕行、通过；用系统截屏功能保存至少四帧，并保留本次 `trial.log`、`summary.json`、`data/plans.jsonl`。箱体接近路径但未完全堵死，演示不保证一定成功。

**2 考官随机指定位置，跑两次｜仍在终端 A。** 请考官明确给 **Gazebo 世界坐标** x、y，以及箱体长宽；若只在 RViz 地图上点出位置，本场景约有 `map x = world x + 5`、`map y = world y + 4` 的偏移，先核对再转换。两次使用同一位置、尺寸、目标和 3 秒延迟，避免临时改变条件后把结果合算。逐行输入：

```bash
read -r -p '考官指定的 Gazebo world x: ' BOX_X
read -r -p '考官指定的 Gazebo world y: ' BOX_Y
read -r -p '箱体长宽（米，例如 0.5）: ' BOX_SIZE
RANDOM_OUT="results/random_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config guide_short --runs 2 --delay 3 --world-x "$BOX_X" --world-y "$BOX_Y" --size-x "$BOX_SIZE" --size-y "$BOX_SIZE" --gui --out "$RANDOM_OUT" --first-domain 180
cat "$RANDOM_OUT/summary.json"
```

若考官允许两次间调参，应保留第一次配置与日志；现有 runner 对一个两次批次固定配置。需要改参数时分别使用 `--runs 1 --config short` 和 `--runs 1 --config long`、两个不同输出目录，并清楚说明两次条件不同，不能当同配置重复统计。

**3 展示提前完成的对比数据｜终端 A；图像查看器。**

```bash
cat results/short_10/summary.json
cat results/long_10/summary.json
cat results/long_tuned_10/summary.json
cat report.md
xdg-open results/four_metric_comparison.png
xdg-open results/guide_pilot_offset_20261122/run_01/process_frames/01_approach.png
xdg-open results/guide_pilot_offset_20261122/run_01/process_frames/02_detection.png
xdg-open results/guide_pilot_offset_20261122/run_01/process_frames/03_detour.png
xdg-open results/guide_pilot_offset_20261122/run_01/process_frames/04_after_obstacle.png
```

先报两套参数各十次的成功数和四指标，再报 0.50 m 调优十次。明确指出这是 **8 秒、固定近路径箱体**的历史数据，与现场 3 秒随机位置不是同一组。四指标图的路径项采用端点直线基线，不能说成无障碍参考路线。四张过程图是实测数据重建，现场 Gazebo 截图另拍。

**4 现场复现已观察过的一条失效｜终端 A；Gazebo 窗口。** 历史 `results/pilot_on_path_short/` 在世界 `(-1.0,-3.6)`、0.35 m 箱体、8 秒延迟曾失败；先给考官看原记录，再用相同条件独立重试：

```bash
cat results/pilot_on_path_short/run_01/summary.json
FAIL_OUT="results/failure_live_$(date +%Y%m%d_%H%M%S)"
python3 scripts/run_experiment.py --config guide_short --runs 1 --delay 3 --world-x -1.0 --world-y -3.6 --size-x 0.35 --size-y 0.35 --gui --out "$FAIL_OUT" --first-domain 200
cat "$FAIL_OUT/run_01/summary.json"
```

仿真有波动，历史一次失败不能保证这次必失败；如结果不同，应如实说明。程序完整试跑但目标失败时可能以退出码 1 结束，需查看逐次记录，而不是只凭退出码判断脚本崩溃。

**5 若需按指南新条件重做正式统计｜终端 A，考前顺序运行。** 以下是待执行的**新实验**，不要覆盖已有 8 秒数据。先固定一个真正处于规划行驶带且留有可行侧向空间的箱体位置，试跑确认安全与难度，再固定位置、尺寸、目标和 3 秒延迟；两组仅改前视距离。示例命令的箱体位置是候选值，不能在未试跑前称为可通过的正式条件：

```bash
python3 scripts/run_experiment.py --config guide_short --runs 10 --delay 3 --world-x -1.0 --world-y -3.35 --size-x 0.30 --size-y 0.30 --max-preflight-attempts 4 --first-domain 20 --out results/guide_short_10_v2_20261122 --resume
python3 scripts/run_experiment.py --config guide_long --runs 10 --delay 3 --world-x -1.0 --world-y -3.35 --size-x 0.30 --size-y 0.30 --max-preflight-attempts 4 --first-domain 70 --out results/guide_long_10_new
python3 scripts/analyze_experiment.py results/guide_short_10_v2_20261122
python3 scripts/analyze_experiment.py results/guide_long_10_new
```

两组各十次后才选较差组、只改一个参数再跑十次。指南按无障碍路线定义额外路径，还需用同一目标和速度补无障碍参考运行，再以其成功轨迹长度作为统一基线重算；不可直接拿现有“超出起终点直线”的数据冒充。四指标图、四帧截图和失效触发记录须与这组新场景对应。每次完整运行的失败都计入分母；预检时尚未发目标的拒绝启动单列，不计入十次。
