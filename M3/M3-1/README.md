# M3-1｜坐标系与里程计

## 最终结论速览

最终车模在 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 中完成了 10 圈 2 m 外框的圆角方形实验。每圈单独采集 `/odom` 与 Gazebo 真值并对齐到同一个后轴地面参考点。**里程计终点相对真值的漂移为 3.37 ± 0.46 cm（10 圈均值 ± 样本标准差）**；逐圈结果见 [`per_run.csv`](results/frame_aligned_repeats_01/per_run.csv)，轨迹叠加见 [`trajectories.png`](results/frame_aligned_repeats_01/trajectories.png)。小车实际未回到起点的闭环距离另为 **1.40 ± 0.36 cm**，不能与里程计漂移混为一谈。

最终车模的 TF 主链为 `odom → base_footprint → base_link → lidar_link`。`odom` 来自轮里程计，`base_link` 比后轴地面点前移 0.14 m、上移 0.05 m，雷达再上移 0.085 m；本阶段未运行地图定位，因此没有 `map`。分段分析表明残余航向差主要在 4 次转弯中累积（合计约 −1.70°），直线段距离基本吻合。历史轮名顺序错误已修正；剩余转弯偏差尚不能只凭位姿数据唯一归因于打滑或某个标定参数。详见下文的实验、代码和局限性说明。

## 本阶段要学会什么

- 理解 `odom`、`base_footprint`、`base_link`、`lidar_link` 的物理含义。
- 把给定 Xacro 和控制器参数组装成可构建的 ROS2 包。
- 控制阿克曼小车走 2 m 正方形闭环。
- 同时记录里程计和 Gazebo 真值，重复 10 次并计算均值与标准差。
- 画等比例轨迹对比图，并解释误差来源。

## 完成路线

- [x] 第 1 步：读车模，核对轴距、轮距、轮半径、最大转向角。
- [x] 第 2 步：计算最小转弯半径，设计圆角正方形转弯方法，并完成仿真验证。
- [x] 第 3 步：创建并构建 `smart_car_description` ROS2 包；Xacro 已成功展开。
- [x] 第 4 步：创建空地 Gazebo 世界和启动文件，小车与雷达已在桌面仿真显示。
- [x] 第 5 步：确认 `/cmd_vel`、`/odom`、`/tf` 和 `/gazebo/model_states`；完成低速直行冒烟测试。
- [x] 第 6 步：实现参数化圆角方形驾驶脚本，并在 Gazebo 中实测。
- [x] 第 7 步：实现两路轨迹记录，完成 10 次实验。
- [x] 第 8 步：计算闭环误差并绘图。
- [x] 第 9 步：生成、标注并解释 TF 树；核对里程计与车模的物理参考点。
- [x] 第 10 步：用参考点修正后的最终车模重复 10 圈并更新正式统计。

## 计划目录

```text
M3-1/
├── worlds/                     # square.world
├── src/smart_car_description/  # ROS2 包：urdf/config/launch/package.xml
├── scripts/                    # 驾驶、记录、分析脚本
├── logs/                       # 10 次原始 CSV
├── results/                    # 统计、轨迹图、TF 树
└── README.md
```

## 第一课：车模参数和过弯方案

原始文件取自[上游 M3-1 题目目录](https://github.com/Blake-John/22SmartCar/tree/main/round1/M3/M3-1)，暂时保留原始参数，后续通过仿真检验它们是否能正确驱动车辆。

| 参数 | Xacro 车模 | 控制器 YAML | 作用 |
| :-- | :-- | :-- | :-- |
| 轴距 | `wheelbase = 0.28 m` | `wheelbase = 0.28 m` | 影响转弯半径 |
| 前后轮距 | `0.18 m` | `0.18 m` | 左右轮转向和速度差 |
| 轮半径 | `0.05 m` | `0.05 m` | 轮速换算和里程计 |
| 转向限位 | 前轮关节约 `±0.5236 rad` | `±0.5236 rad` | 最大约 30° |

`base_footprint` 在地面；修正后的车模将车体参考点 `base_link` 放在它前方半个轴距（0.14 m）、上方一个轮半径（0.05 m）。这段变换由车模中的 fixed joint 定义。下文已采集的 10 圈和第一次 TF 图保留了**修正前** x=0 的配置，便于比较。

先用简化单轨模型估算。轴距 `L = 0.28 m`、最大等效转向角 `δ ≈ 30°`：

```text
R_min ≈ L / tan(δ) = 0.28 / tan(30°) ≈ 0.485 m
90° 圆弧长度 ≈ πR/2 ≈ 0.762 m
```

方案：始终前进，直线与四段 90° 圆弧交替，形成外框约 2 m × 2 m 的圆角正方形轨迹。单轨公式得到的 `0.485 m` 只是车体中心的理论下界：转弯时内侧前轮转角更大，还要加半个前轮距 `0.09 m`，所以中心线半径至少约 `0.575 m`。第一次试跑选 `R = 0.65 m`，每边直线段为 `2−2R = 0.70 m`。角点不会是数学意义的尖角。车轮打滑、控制器跟踪延迟等仍可能让实测半径偏离命令值，必须以 Gazebo 轨迹检查。

后续实验统一以 `/gazebo/model_states` 记录的起点和终点计算真实闭环偏差；`/odom` 只用于与真值比较，不用真值修正控制器的里程计。

## 当前环境依赖

本机已有 ROS 2 Humble 和 `colcon`；Gazebo ROS、`gazebo_ros2_control`、阿克曼控制器及 `xacro` 已安装。若在新机器上复现，需要有管理员权限的终端执行：

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ackermann-steering-controller ros-humble-xacro
```

安装后用 `ros2 pkg prefix gazebo_ros`、`ros2 pkg prefix gazebo_ros2_control`、`ros2 pkg prefix ackermann_steering_controller` 和 `ros2 pkg prefix xacro` 逐一确认。若实际仿真运行在另一台机器或 WSL，应在那套 Ubuntu 环境执行这些命令。

## 将车模组装成 ROS 2 包

`package.xml` 是包的身份及依赖清单；`CMakeLists.txt` 告诉构建工具把 `urdf/`、`config/`、`launch/` 放入安装目录。因此 Xacro 中的 `$(find smart_car_description)` 才能找到控制器 YAML。

在本目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select smart_car_description
source install/setup.bash
ros2 pkg prefix smart_car_description
xacro src/smart_car_description/urdf/smart_car.urdf.xacro -o /tmp/smart_car_m3_1.urdf
```

构建成功后应显示 `1 package finished`，`ros2 pkg prefix` 应指向 `M3-1/install/smart_car_description`。生成的 URDF 中应有 11 个 link、10 个 joint；这只证明模型可解析，**不代表仿真控制与里程计已经跑通**。

## 启动最小仿真

`worlds/square.world` 定义了 20 m × 20 m 的平地、灯光和物理引擎；`launch/sim.launch.py` 启动 Gazebo、`robot_state_publisher`、生成车模，然后依次加载关节状态广播器和阿克曼控制器。世界插件将 Gazebo 真值话题映射到题目约定的 `/gazebo/model_states`。

```bash
cd M3/M3-1
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select smart_car_description
source install/setup.bash
ros2 launch smart_car_description sim.launch.py
```

现场首次验证：Gazebo 出现小车和雷达，`/scan`、`/tf`、`/model_states` 已发布，**但 `/odom` 缺失**。原因是启动文件当时尚未加载阿克曼控制器。第二次验证发现系统未安装 `ros-humble-joint-state-broadcaster`，补装后两个控制器均显示 `Configured and activated`。后续话题列表含 `/odom`、`/gazebo/model_states`、`/scan`、`/tf`、`/tf_static`；`/odom` 单条消息实测 `header.frame_id: odom`、`child_frame_id: base_footprint`。下文记录了随后完成的 TF 查询与行驶实验。

旧版终端缺少 `ros2 control` CLI 插件，诊断时可直接调用控制器管理服务：

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

期待看到 `joint_state_broadcaster` 与 `ackermann_steering_controller` 均为 `active`，并在 `ros2 topic list` 中看到 `/odom` 与 `/gazebo/model_states`。如果失败，请保留 spawner 的日志，不要把控制器失败误判为 Gazebo 没安装。

### 现场验证记录

- 两个控制器均显示 `Configured and activated`；`/cmd_vel` 的类型为 `geometry_msgs/msg/Twist`，订阅者是阿克曼控制器。
- TF 实测：`odom → base_footprint` 随时间更新；`base_link → lidar_link` 固定平移 `[0, 0, 0.085] m`，旋转为零。
- 以 `0.1 m/s`、10 Hz 发布 30 次直行命令后，`/odom` 的 x 从约 `-0.0456 m` 变为 `0.3436 m`，差约 `0.389 m`；y 基本不变。这证明控制链路可用，**尚不能作为漂移误差**，因为本次没有同步采集前后 Gazebo 真值。
- 下一步采集 `/odom` 与 `/gazebo/model_states` 的同步轨迹，并统一时间与起点；只有两者比较才可评估里程计误差。

## 两路轨迹记录器：第一次练习

`scripts/record_traj.py` 同时订阅 `/odom` 和 `/gazebo/model_states`，在后者的 `name` 数组里定位 `smart_car`，按同一仿真时钟输出 `odom.csv`、`truth.csv`，表头均为 `t,x,y,theta`。`theta` 是弧度，规范到 `(-π, π]`。里程计使用消息自带的仿真时间戳；真值消息没有自身时间戳，所以使用收到消息时的 `/clock` 时间。两路采样点不必恰好同一时刻，后续分析要按时间插值或近邻配对。

第一次静止实验 5 秒内得到 250 行 `/odom` 与 100 行真值，分别约为 50 Hz、20 Hz。但当时 Gazebo `/clock` 默认只有 10 Hz，时间列出现重复（多条数据同为 `t=0.000000`）；现已增加 `config/gazebo_clock.yaml` 将 `/clock` 设为 100 Hz，并修正里程计取自身时间戳。旧 `stationary_01` 保留作排错证据，不用作定量统计。

重启后的 `stationary_02` 同样取得 250 行里程计和 100 行真值；时间列严格递增，里程计间隔 0.02 s、真值间隔 0.05 s。这验证了记录器和仿真时间配置，但车辆未走闭环，不能拿静止样本计算题目的闭环漂移。两次小样本都保存在 `logs/`，后续 10 次正式实验另存新文件。

先保持小车不动，仅用 5 秒验证两路文件都能写出：

```bash
python3 scripts/record_traj.py --duration 5 --out logs/stationary_01
head -n 4 logs/stationary_01/odom.csv
head -n 4 logs/stationary_01/truth.csv
```

Gazebo 保持运行，先在一个新终端执行（以下 `cd` 命令从仓库根目录开始）：

```bash
cd M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/record_traj.py --duration 12 --out logs/straight_01
```

在记录器运行期间，另一终端发送低速直行命令：

```bash
ros2 topic pub -r 10 -t 30 --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

最后检查 `head -n 4 logs/straight_01/odom.csv` 与 `head -n 4 logs/straight_01/truth.csv`。这只是记录功能试验，尚未完成 2 m 正方形和 10 次统计。

## 下一课：第一圈圆角方形试跑

`scripts/drive_square.py` 以 20 Hz 向 `/cmd_vel` 发送速度命令。直线时 `linear.x = 0.1 m/s`、`angular.z = 0`；转弯时保持前进，用 `angular.z = v/R` 指定转向速度。每段的持续时间按 `距离÷速度` 计算，**不读取 `/odom` 作为停止条件**。这样试跑可以揭示命令、里程计与 Gazebo 真值之间的真实差异，避免里程计自己决定何时“走够一圈”。脚本用 Gazebo 仿真时间计段长，仿真暂停时不会偷偷走完下一段。

车模的轴距、轮距和转向限位决定了半径下界。默认 `--side 2.0 --radius 0.65 --speed 0.1 --laps 1` 的一圈含四段 `0.70 m` 直线和四段 `90°` 圆弧，理论约 69 秒仿真时间。`--side` 指中心线轨迹的外框宽度，不是每段直线的长度。

先重新启动仿真（终端 A，保持运行）：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smart_car_description sim.launch.py
```

打开第二个 Ubuntu 终端（终端 B），先开始记录。`--duration 150` 留出切换终端和观察时间；驾驶结束后可按 `Ctrl+C` 提前保存：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/record_traj.py --duration 150 --out logs/square_trial_01
```

打开第三个 Ubuntu 终端（终端 C），开始第一圈：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/drive_square.py --side 2.0 --radius 0.65 --speed 0.1 --laps 1
```

看到 `Run complete` 后回终端 B 按 `Ctrl+C`，确认出现 `Saved ... odom and ... truth rows`。终端 A 的 Gazebo 可以暂时保持运行。此时只检查第一圈的实际运动形状和两路文件是否完整；正式的 10 次统计需另做，试跑数据不计入结果。

### 第一圈试跑记录（调试数据，不计入正式统计）

现场完成四段直线和四段左转，`drive_square.py` 输出 `Run complete`。记录器保存 4756 行里程计与 1902 行 Gazebo 真值，两个 CSV 的时间列均严格递增，范围约 95 秒。按 `Ctrl+C` 后曾在 `rclpy.shutdown()` 报重复关闭错误；文件已经在报错前关闭并保存。记录器和驾驶脚本现都改用 `rclpy.try_shutdown()` 处理这种退出路径。

两条轨迹分别平移并旋转到自身起点后，在 [`results/square_trial_01.png`](results/square_trial_01.png) 中按等比例坐标与理想指令轨迹叠加。终点数据如下；角度是整圈展开后的总转角，不能只看原始 `theta` 的末值（它会在 ±π 处折返）。

| 指标 | `/odom` | Gazebo 真值 |
| :-- | --: | --: |
| 终点距各自起点 | 0.620 m | 0.345 m |
| 累积转角 | 327.2° | 380.8° |

两路起点对齐后的终点相距约 **0.925 m**。这里的 0.345 m 是车实际没有回到起点的闭环偏差；0.925 m 才是本次里程计与真值在终点的估计差异。按试跑开始时间粗略分段，四次转弯每次都约为里程计 82°、真值 95°，而指令期望 90°。重复出现的方向性偏差提示优先检查转向几何、控制器里程计计算和轮地接触；单凭这一圈还不能断言是打滑或某个参数错误。正式实验前要先诊断转弯偏差，再明确 10 次试验的统一起止条件。

## 转弯诊断：同时看前轮和车身

圆弧命令取 `v=0.1 m/s`、`R=0.65 m`，所以目标角速度 `ω=v/R≈0.154 rad/s`，持续 `πR/(2v)≈10.2 s` 后期望转过 90°。依据轴距 `0.28 m`、前轮距 `0.18 m`，理想左转的内侧前轮角约 `atan(0.28/(0.65−0.09))≈26.6°`，外侧前轮约 `atan(0.28/(0.65+0.09))≈20.7°`。这只是几何预测，`/joint_states` 才能告诉我们仿真里前轮实际转到了多少。

如果 Gazebo 已经关闭，先按上文“终端 A”命令启动。保持 Gazebo 运行，在终端 B 开始记录：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/record_traj.py --duration 90 --joints --out logs/turn_diag_01
```

终端 C 执行一次左转：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/drive_square.py --radius 0.65 --speed 0.1 --turn-only
```

终端 C 显示 `Run complete` 后，回终端 B 按 `Ctrl+C` 保存。期待 `logs/turn_diag_01/` 出现 `odom.csv`、`truth.csv`、`joints.csv`。其中 `joints.csv` 的前两列关节位置是弧度，代表左右前轮转向角；后两列是后轮旋转角，不是车身航向。比较三者可区分转向命令未执行、车身运动偏离车轮几何，以及里程计计算偏差。

### 修正前的诊断结果

`turn_diag_01` 实际保存 2934 条里程计、1173 条真值和 2935 条关节状态。根据前轮转角大于 `0.05 rad` 的时段识别转弯，约为记录时间 `11.4–21.6 s`；稳定段用中位数求得：

| 量 | 理想几何预测 | 仿真实际 |
| :-- | --: | --: |
| 左前轮转角 | 26.6° | 20.7° |
| 右前轮转角 | 20.7° | 26.6° |
| 车身转过角度 | 90° | Gazebo 真值 95.9°；`/odom` 81.5° |

图见 [`results/turn_diag_01.png`](results/turn_diag_01.png)。前轮角度恰好左右对调，证据指向控制器轮名数组的顺序，而不是前轮没有转动。[ROS 2 Humble 的控制器源码](https://raw.githubusercontent.com/ros-controls/ros2_controllers/humble/steering_controllers_library/src/steering_odometry.cpp)在 Ackermann 模式输出 `{右轮, 左轮}` 的牵引速度和转向角，里程计也按这个顺序解释反馈。原始 YAML 的两个数组都是 `{左轮, 右轮}`，所以现改成 `{右轮, 左轮}`。这解释了明确的配置错误，但是否能消除 95.9° 与 81.5° 的差距，需要下一次相同命令实测。

### 修正后复测

由于控制器启动时读取 YAML，先到运行 Gazebo 的终端 A 按 `Ctrl+C`，等它退出。ROS 2 包已重新构建，安装目录中的 YAML 已确认是右轮在前；随后在终端 A 执行：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smart_car_description sim.launch.py
```

控制器启动成功后，终端 B 记录到**新目录**，保留修正前数据以便比较：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/record_traj.py --duration 90 --joints --out logs/turn_diag_02
```

终端 C 再执行同一个转弯命令：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/drive_square.py --radius 0.65 --speed 0.1 --turn-only
```

看到 `Run complete` 后，在终端 B 按 `Ctrl+C` 保存。复测要比较左右前轮转角是否恢复为约 `26.6° / 20.7°`，以及两种车身转角是否更接近命令的 90°。诊断数据不计入 10 次正式闭环统计。

### 修正后复测结果

`turn_diag_02` 保存了 3563 条里程计、1425 条真值和 3563 条关节状态，三份 CSV 的时间列均严格递增。使用与修正前相同的方法：前轮角大于 `0.05 rad` 识别转弯区间，排除开始和结束各 1 秒后取前轮角中位数；车身转角使用记录起止航向之差，并按角度环绕计算。对比图是 [`results/turn_diag_comparison.png`](results/turn_diag_comparison.png)。

| 指标 | 修正前 | 修正后 | 命令或几何预测 |
| :-- | --: | --: | --: |
| 左前轮转角 | 20.73° | 26.56° | 26.57° |
| 右前轮转角 | 26.57° | 20.72° | 20.73° |
| Gazebo 车身转角 | 95.91° | 90.42° | 90° |
| `/odom` 车身转角 | 81.49° | 89.06° | 90° |
| 两种车身转角之差 | 14.43° | 1.36° | 0° |

单次转弯的里程计与真值角度差下降约 90.6%。这强烈支持原先轮名顺序是主要的系统性误差来源；仍有约 1.36° 差异，不能据此断言车轮完全不打滑。下一步先复测**完整一圈**的起终点误差，再设计 10 次重复试验及均值 ± 标准差。

## 修正后第一圈与位姿参考点

`square_trial_02` 复测同一条 2 m 外框圆角方形命令，保存 4793 条里程计、1918 条 Gazebo 真值、4795 条关节数据。三份 CSV 时间严格递增。车体转角分别为 `/odom` **357.02°** 和 Gazebo 真值 **358.53°**，与完整一圈的 360° 很接近。

直接叠加 `/odom` 和 `/gazebo/model_states` 时，途中仍可能看到约十几厘米的间隔。这部分不能立即算作里程计漂移：控制器积分的是**后驱动轴中点**的轨迹，而模型位姿在现有车模中表现为**车体中心**，两者沿车体前后方向相距约半个轴距 `0.28/2=0.14 m`。转弯时车体中心与后轴中点画出的圆弧不同。`scripts/analyze_trial.py` 保留原始 CSV，并在分析时按 `后轴真值 = 模型位姿 − 0.14·(cos θ, sin θ)` 转换到相同的物理点；然后分别对齐起点和初始朝向，在相同时刻插值比较。这个 0.14 m 转换与轨迹形状吻合，但后续 TF 检查还应核对模型坐标与车体坐标的关系。[ROS 2 控制器运动学说明](https://control.ros.org/iron/doc/ros2_controllers/doc/mobile_robot_kinematics.html)给出了后轴速度与车身角速度的关系。

```bash
python3 scripts/analyze_trial.py --input logs/square_trial_02 --output-prefix results/square_trial_02
```

脚本输出 [`results/square_trial_02_trajectory.png`](results/square_trial_02_trajectory.png) 和 [`results/square_trial_02_analysis.json`](results/square_trial_02_analysis.json)。使用同一后轴参考点，修正前后试跑如下：

| 指标 | 轮名修正前 `square_trial_01` | 修正后 `square_trial_02` |
| :-- | --: | --: |
| Gazebo 真值终点距起点 | 0.358 m | **0.0238 m** |
| `/odom` 终点距起点 | 0.620 m | **0.0780 m** |
| 两路终点位置之差 | 0.954 m | **0.0545 m** |
| 行驶与静止期间逐时刻位置差的平均值 | 0.423 m | **0.0275 m** |

最后一行是**这一圈中多个时间采样点的平均差**，不是题目要求的“10 次运行的均值 ± 标准差”。正式统计必须以每次完整运行分别算一个误差，再对 10 个误差值求均值和标准差。

## 重复实验：先试运行，再采集 10 圈

`scripts/run_trials.py` 把“开始记录 → 静止 1 秒 → 驾驶一圈 → 停车后继续记录 2 秒”合成一次运行，并为每圈创建独立的 `run_01/`、`run_02/` 等目录。每个目录包含原始 `odom.csv`、`truth.csv` 和表示完整性的 `metadata.json`。驾驶仍然只按预设速度与仿真时间执行，Gazebo 真值只用于分析，不反馈给驾驶器。每圈约 69 秒仿真时间；10 圈约需 12 分钟，加上 Gazebo 加载时间。

这里的 10 次是**同一 Gazebo 会话中连续的 10 圈**，每圈起点和初始朝向分别对齐；小车在两圈之间停约 2 秒，仿真世界不会重置。因此计算出的标准差描述此连续试验的圈间变化，不能解释为完全独立的 10 次开机试验。

先保持 Gazebo 和控制器运行，用一个新终端试采 **1 圈**，验证单命令流程：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_trials.py --runs 1 --out logs/repeat_pilot
```

成功应看到 `Run 1/1: complete`，并有 `logs/repeat_pilot/run_01/metadata.json`。正式运行时使用**新的**输出目录，避免覆盖试采：

```bash
python3 scripts/run_trials.py --runs 10 --out logs/square_repeats_01
```

全部显示 `complete` 后，计算每圈误差和 10 圈的**样本标准差**（分母 `n−1`）：

```bash
python3 scripts/analyze_repeats.py --runs-dir logs/square_repeats_01 --out results/square_repeats_01
```

`per_run.csv` 每圈一行；`summary.json` 对各指标给出均值与标准差；`trajectories.png` 以相同坐标比例叠加 10 圈。最核心的漂移指标是**起点对齐后的里程计终点与后轴 Gazebo 真值终点之差**；Gazebo 真值自身的闭环偏差单列，表示小车对计划路线的执行误差。分析时仍将原始模型位姿后移 `0.14 m` 到后轴中点，原始 CSV 不改写。分析脚本会拒绝缺失或不完整的运行目录，不会把半圈误当正式样本。

### 10 圈实测结果

单圈试采 `logs/repeat_pilot` 有 3624 条里程计和 1450 条真值，后轴参考点的终点差为 0.03516 m。正式数据 `logs/square_repeats_01` 的 10 圈均显示 `complete`；每圈约 3622–3628 条里程计和 1448–1454 条真值。所有时间戳都没有倒退。第 3 圈真值在开始处有 5 条相同时间戳：`/gazebo/model_states` 自身没有消息时间戳，记录器采用最近一次 `/clock` 时间，同一时钟刻度可能收到多条状态。分析时仅对这 5 条相同时间戳保留最后一条以便插值，原始 CSV 仍完整保留。

| 每圈各算一个值，再统计 10 圈 | 均值 ± 样本标准差 |
| :-- | --: |
| 里程计与 Gazebo 后轴真值的终点距离（主要漂移指标） | **0.03377 ± 0.00442 m**（3.38 ± 0.44 cm） |
| Gazebo 后轴真值自身的闭环距离 | 0.01357 ± 0.00475 m（1.36 ± 0.48 cm） |
| 里程计自身的闭环距离 | 0.04115 ± 0.00782 m（4.12 ± 0.78 cm） |
| 全程同一时刻的平均轨迹距离 | 0.01537 ± 0.00282 m（1.54 ± 0.28 cm） |
| Gazebo 真值相对 360° 的航向误差 | −0.598 ± 0.273° |
| 里程计相对 360° 的航向误差 | −2.346 ± 0.265° |

10 圈终点差范围为 0.02771–0.04435 m。样本标准差用 `s = sqrt(Σ(eᵢ−ē)²/(10−1))`，这里的 `eᵢ` 是第 `i` 圈的**一个**终点差，而不是 CSV 的几千行采样点。`results/square_repeats_01/per_run.csv` 可逐圈复算，`summary.json` 保留精确值，`trajectories.png` 是等比例轨迹图。路程按命令几何约 6.884 m，平均终点差约为其 0.49%；该比值仅辅助理解，不代表沿途每处都有同样百分比的误差。

10 圈里程计航向平均比 Gazebo 真值少约 1.75°，且每圈同向，说明尚有稳定的转弯偏差；终点差的均值也明显大于圈间标准差。此前把控制器车轮数组从左先改成右先，让单次 90° 转弯的里程计与真值差由约 14.43° 降到约 1.36°，证明**轮名顺序错误**确实是先前主要误差源。剩余偏差可能来自轮半径、轮距、轴距、轮地接触或控制器运动学近似；现有两路位姿和这 10 圈数据尚不能把它唯一归因为“打滑”。若要进一步区分，需要固定其他变量、记录各车轮关节状态，并做直线与单次转弯的对照实验。

这 10 圈在同一 Gazebo 会话中连续运行、每圈独立对齐起点。结果表征当时仿真配置下的圈间变化，**不是** 10 次独立重启或换地面的泛化误差。后续 TF 检查与最终车模实验记录在下文。

## TF 坐标系现场检查

在 Gazebo 运行时执行 `ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true`，得到 [`results/tf_tree.pdf`](results/tf_tree.pdf)、[`results/tf_tree.gv`](results/tf_tree.gv) 和便于预览的 [`results/tf_tree.png`](results/tf_tree.png)。这张图来自**参考点修正前**的运行，实测主链为：

```text
odom ──动态，约 50 Hz──> base_footprint ──固定──> base_link ──固定──> lidar_link
```

`base_link` 还连接了相机和四轮。`view_frames` 给固定变换显示的 `10000 Hz`、时间 `0.0` 是其静态变换显示方式，不能理解成传感器每秒发布一万次；图中的 `default_authority` 也不足以单凭这张图识别发布节点。结合本项目配置，`odom → base_footprint` 来自阿克曼控制器（YAML 中 `enable_odom_tf: true`，frame ID 设为 `odom` 和 `base_footprint`）；车体、雷达等关节的固定变换来自 `robot_state_publisher` 展开的 Xacro；车轮关节随 `/joint_states` 更新。旧版实测 `base_footprint → base_link` 是 `[0, 0, 0.05] m`，`base_link → lidar_link` 是 `[0, 0, 0.085] m`，均无转角。

| frame | 本项目中的含义 |
| :-- | :-- |
| `map` | 地图参考系，通常由建图或定位系统提供长期全局参照；本次没有启动这类节点，因此树中**没有** `map`，不属于 Gazebo 启动失败。 |
| `odom` | 局部连续参照；轮速积分让它相对车身平滑，但长期会漂移。 |
| `base_footprint` | 车体基座在地面的二维参考点；修正后位于后轴中点。旧版将它放在 `base_link` 正下方。 |
| `base_link` | 车体本体的参考坐标系；跟随小车平移、转动。 |
| `lidar_link` | 雷达安装位置；相对车体固定，`/scan` 应使用它作为 `header.frame_id`。 |

[ROS REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)规定了 `map`、`odom`、`base_link` 的用途和层级；本项目在 `odom` 与 `base_link` 之间增加地面投影 `base_footprint`。将来接入定位时通常由定位系统提供 `map → odom`，而轮里程计继续提供 `odom → base_footprint`。

**参考点审计：** 旧版 Xacro 的 `base_footprint → base_link` 在 x 方向为 0，前后轴相对 `base_link` 分别在 `+0.14 m` 和 `−0.14 m`；按模型定义，旧版 `base_footprint` 因而位于车体中心正下方。但本项目以两个后轮和前轮转角计算的阿克曼里程计运动学，结合已录轨迹，表明 `/odom` 的积分轨迹更接近**后轴中点**。在单圈试采中，不换算参考点的全程平均轨迹距离约 0.181 m，后移 Gazebo 模型真值 0.14 m 后降为约 0.018 m。原始 10 圈分析仍按旧版模型将 Gazebo 真值后移 0.14 m，结果有效且保留原始 CSV。此问题属于坐标定义一致性，不能简单算作轮子打滑。[ROS 2 Humble 的车式运动学说明](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html)给出了后轴速度与车身角速度的计算关系。

已将源码中的 `base_footprint_joint` 改为 `base_link` 相对后轴地面点前移 0.14 m、上移 0.05 m，使 TF 名称与控制器的积分参考点一致。重新构建并重启 Gazebo 后，`tf2_echo base_footprint base_link` 实测平移 `[0.140, 0.000, 0.050] m`、旋转为零。新采集目录的 `metadata.json` 写入 `truth_forward_offset_m: 0.0`，分析器按每次采集的元数据选择换算距离；旧目录没有此字段时仍按旧版 0.14 m 分析。

修正后单圈试采 `logs/frame_aligned_pilot` 完成 3623 条里程计与 1449 条真值。无需再后移 Gazebo 真值，两路轨迹的全程平均距离为 **0.01757 m**、终点距离为 **0.02679 m**；Gazebo 自身闭环距离为 0.01966 m，里程计自身为 0.04634 m。如果错误地继续后移 0.14 m，平均轨迹距离反而变为 **0.15476 m**。这验证了新模型位姿的参考点已发生预期变化，且驾驶链路仍可工作。单圈试采本身不能给出均值与标准差；最终车模的正式 10 圈结果见下一节。旧版 10 圈与 TF 图继续作为改动前对照，不能冒充新版验证。

## 最终车模的正式 10 圈结果

最终数据位于 `logs/frame_aligned_repeats_01/run_01` 至 `run_10`。10 圈均由自动驾驶脚本正常结束、元数据均标记 `complete` 和 `truth_forward_offset_m: 0.0`，每圈约 3623–3624 条里程计、1449–1450 条真值；所有 CSV 时间戳严格递增。`scripts/analyze_repeats.py` 输出逐圈指标 [`results/frame_aligned_repeats_01/per_run.csv`](results/frame_aligned_repeats_01/per_run.csv)、汇总 [`summary.json`](results/frame_aligned_repeats_01/summary.json) 和等比例叠加图 [`trajectories.png`](results/frame_aligned_repeats_01/trajectories.png)。

| 最终配置的指标 | 10 圈均值 ± 样本标准差 |
| :-- | --: |
| **里程计与 Gazebo 后轴真值的终点距离** | **0.03371 ± 0.00455 m（3.37 ± 0.46 cm）** |
| Gazebo 真值自身闭环距离 | 0.01401 ± 0.00357 m（1.40 ± 0.36 cm） |
| 里程计自身闭环距离 | 0.04235 ± 0.00491 m（4.24 ± 0.49 cm） |
| 全程同一时刻的平均轨迹距离 | 0.01642 ± 0.00292 m（1.64 ± 0.29 cm） |
| Gazebo 真值相对 360° 的航向误差 | −0.601 ± 0.195° |
| 里程计相对 360° 的航向误差 | −2.388 ± 0.215° |

每圈的主要漂移指标先算 `eᵢ = ||p_odom,end − p_truth,end||`，两个终点都以各自起点和初始朝向归零，且现在无需前后平移真值。然后对 10 个 `eᵢ` 求算术平均和样本标准差：`s = sqrt(Σ(eᵢ−ē)²/(10−1))`。最终 10 个终点距离范围为 **0.02650–0.04091 m**。Gazebo 自身闭环距离另列，因为小车没精准走回起点是**路径执行偏差**，与里程计相对真值的估计偏差不是同一个量。旧版（分析时需后移真值 0.14 m）的终点差为 3.38 ± 0.44 cm，与新版数值相近；调整 TF 参考点主要是修正坐标语义，不会自动消除车轮里程计的剩余漂移。

### 漂移主要在哪里产生

`scripts/analyze_segments.py` 按 `metadata.json` 保存的驾驶开始时刻，把每圈切为 4 段直线和 4 段左转，在计划段边界对两路轨迹插值。输出 [`per_segment.csv`](results/frame_aligned_repeats_01/per_segment.csv)、[`per_run_segments.csv`](results/frame_aligned_repeats_01/per_run_segments.csv) 与 [`segment_summary.json`](results/frame_aligned_repeats_01/segment_summary.json)。以下的均值和样本标准差都先**每圈汇总**，再跨 10 圈统计；段边界依据计划仿真时间，实际 `/cmd_vel` 传递可能有一个 20 Hz 发送周期的延迟。

| 航向差：里程计减 Gazebo 真值 | 10 圈均值 ± 样本标准差 |
| :-- | --: |
| 4 段直线合计 | −0.050 ± 0.069° |
| 4 段转弯合计 | **−1.699 ± 0.176°** |
| 平均每次转弯 | **−0.425 ± 0.044°** |

直线段的距离差（每圈先取 4 段平均）为 **0.000036 ± 0.000625 m**，即均值约 0.04 mm；这说明当前设置下，直线里程尺度与 Gazebo 真值相当接近。最终 10 圈整体航向差平均约 **−1.787°**，分段结果说明主要的**航向差**在四次转弯中累积，而非直线行驶。早期单转弯实验已证实车轮左右数组顺序错误会造成大幅转向偏差，修正后角度差明显缩小。现在留下约每次 −0.425° 的方向性偏差；直线距离基本吻合使“单纯轮半径尺度错误”不太像主要原因，但轮地接触、转向几何标定和控制器运动学近似仍可能贡献误差。仅凭位姿轨迹不能唯一证明打滑；若要区分这些机制，需要在最终配置下重复记录 `/joint_states`，并进行不同转弯半径或不同地面摩擦条件的受控对照。

这些 10 圈是在同一 Gazebo 会话里连续完成、没有逐圈重置世界；统计量描述**这套仿真配置**的圈间变化，不可外推成真实车辆或其他地面的性能。`map` 没有出现在当前 TF 树中，因为本阶段没有运行地图定位；将来定位器应负责连接 `map → odom`，而当前阿克曼控制器继续负责 `odom → base_footprint`。
