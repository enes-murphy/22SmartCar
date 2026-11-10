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
- [ ] 第 4 步：创建空地 Gazebo 世界和启动文件。
- [ ] 第 5 步：确认 `/cmd_vel`、`/odom`、`/tf` 和 `/gazebo/model_states`。
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
