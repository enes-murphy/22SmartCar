# M3-1｜坐标系与里程计

## 本阶段要学会什么

- 理解 `odom`、`base_footprint`、`base_link`、`lidar_link` 的物理含义。
- 把给定 Xacro 和控制器参数组装成可构建的 ROS2 包。
- 控制阿克曼小车走 2 m 正方形闭环。
- 同时记录里程计和 Gazebo 真值，重复 10 次并计算均值与标准差。
- 画等比例轨迹对比图，并解释误差来源。

## 完成路线

- [ ] 第 1 步：读车模，核对轴距、轮距、轮半径、最大转向角。
- [ ] 第 2 步：计算最小转弯半径，设计正方形转弯方法。
- [ ] 第 3 步：创建 `smart_car_description` ROS2 包。
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

## 第一课前置问题

已知轴距 `L = 0.28 m`、最大前轮转角 `δ = 30°`，请先独立计算：

```text
R = L / tan(δ)
```

然后思考：这个半径能否让车沿 2 m 正方形的四个角连续前进转弯？不要先写控制脚本。
