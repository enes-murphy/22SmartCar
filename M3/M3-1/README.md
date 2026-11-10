# M3-1｜坐标系与里程计

## 本阶段要学会什么

- 理解 `odom`、`base_footprint`、`base_link`、`lidar_link` 的物理含义。
- 把给定 Xacro 和控制器参数组装成可构建的 ROS2 包。
- 控制阿克曼小车走 2 m 正方形闭环。
- 同时记录里程计和 Gazebo 真值，重复 10 次并计算均值与标准差。
- 画等比例轨迹对比图，并解释误差来源。

## 完成路线

- [x] 第 1 步：读车模，核对轴距、轮距、轮半径、最大转向角。
- [x] 第 2 步：计算最小转弯半径，设计正方形转弯方法（理论方案，待仿真验证）。
- [x] 第 3 步：创建并构建 `smart_car_description` ROS2 包；Xacro 已成功展开。
- [x] 第 4 步：创建空地 Gazebo 世界和启动文件，小车与雷达已在桌面仿真显示。
- [x] 第 5 步：确认 `/cmd_vel`、`/odom`、`/tf` 和 `/gazebo/model_states`；完成低速直行冒烟测试。
- [ ] 第 6 步：实现参数化正方形驾驶脚本。
- [ ] 第 7 步：实现两路轨迹记录，完成 10 次实验。
- [ ] 第 8 步：计算闭环误差并绘图。
- [ ] 第 9 步：生成、标注并解释 TF 树。

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

`base_footprint` 在地面，`base_link` 比它高一个轮半径（0.05 m）；这段变换由车模中的 fixed joint 定义。

先用简化单轨模型估算。轴距 `L = 0.28 m`、最大等效转向角 `δ ≈ 30°`：

```text
R_min ≈ L / tan(δ) = 0.28 / tan(30°) ≈ 0.485 m
90° 圆弧长度 ≈ πR/2 ≈ 0.762 m
```

方案：始终前进，直线与四段 90° 圆弧交替，形成外框约 2 m × 2 m 的圆角正方形轨迹；若取半径 0.485 m，每边可留 `2−2R ≈ 1.03 m` 直线。角点不会是数学意义的尖角。轴距公式只是初值：两前轮实际转角不同、控制器接口可能有约束，必须以 Gazebo 实测轨迹校准。

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

现场首次验证：Gazebo 出现小车和雷达，`/scan`、`/tf`、`/model_states` 已发布，**但 `/odom` 缺失**。原因是启动文件当时尚未加载阿克曼控制器。第二次验证发现系统未安装 `ros-humble-joint-state-broadcaster`，补装后两个控制器均显示 `Configured and activated`。最新话题列表含 `/odom`、`/gazebo/model_states`、`/scan`、`/tf`、`/tf_static`；`/odom` 单条消息实测 `header.frame_id: odom`、`child_frame_id: base_footprint`。尚需实际查询 TF 两端和控制小车运动。

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
