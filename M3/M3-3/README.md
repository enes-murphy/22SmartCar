# M3-3 自主导航与调优

本题必须沿用 M3-2 的地图，用一条 launch 命令启动仿真、AMCL 和 Nav2，让阿克曼车走一条超过 5 m、至少有一个转弯的路线。考核既看现场给的新目标，也看三项参数的前后证据、三次运行和代价地图及速度诊断。

## 实现与已有证据

`smart_car_navigation` 的 `localization.launch.py` 在 `navigation:=true` 时一键启动全栈；使用 M3-2 的 `map.yaml`。AMCL 发布 `map → odom`，阿克曼控制器发布 `odom → base_footprint`，车模发布固定的 `base_footprint → base_link → lidar_link`。全局规划器是适合车式运动约束的 Hybrid-A*，局部控制器是 Regulated Pure Pursuit；车不能原地转。全局和局部代价地图包含障碍层、膨胀层及车身多边形。窄门已知空闲宽度约 1.15 m；膨胀半径、车宽和转弯半径都必须一起考虑。

现有固定目标地图坐标 `(11, 3, 90°)`，规划路径约 12.8 m，含转弯。基线一次成功，耗时 84.3 s、终点误差 0.18 m；三项参数一起修改后的试跑成功，耗时 71.1 s、终点误差 0.13 m。固定新配置的三次独立仿真均成功，耗时约 71.1、71.2、71.3 s。见 `results/nav_pilot_01/`、`results/nav_tuned_pilot_01/`、`results/nav_repeats_tuned/`。这些历史固定目标试验不能代替考官现场给的新目标。

| 参数 | 基线 → 新值 | 理由与代价 |
| --- | --- | --- |
| `FollowPath.desired_linear_vel` | 0.15 → 0.18 m/s | 提高目标速度；可能缩短反应时间和安全余量。 |
| `FollowPath.lookahead_dist` | 0.45 → 0.40 m | 更贴转弯路径；可能增加转向修正。 |
| `general_goal_checker.xy_goal_tolerance` | 0.20 → 0.15 m | 要求停得更近；可能增加末段调整或失败。 |

三项一起改，不能把 13.2 s 耗时变化归因于其中任意一个。`src/smart_car_navigation/config/navigation.yaml` 是实际 Nav2 配置；基线和调优快照在上述结果目录。历史运行日志中也记录了曾发生的代价地图冻结、控制器进展失败和退出阶段容器报错，不能删去失败历史。

2026-11-22 已在这台机器补录三张独立诊断图：`results/diagnostics_correct_20261122/{global_costmap,local_costmap,cmd_vel_angular}.png`，另存有 `cmd_vel.csv`。它们由 ROS 实际话题采集，不是手画示意图。起初发现组件模式下局部代价地图误读默认参数，故一键 launch 改成普通节点模式；重新采集前实际读回 `rolling_window=true`、局部宽度 `3 m`、全局膨胀半径 `0.25 m`。这次诊断路线到地图 `(10.19,1.02)` 时 Nav2 中止，目标误差 `2.14 m`；因此这组三图是**一次失败运行的诊断证据**，不能说成新增成功记录。图中的局部地图只覆盖随车滚动的 3 m 区域，正适合解释窄处的代价约束。

## 按展示指南现场执行

**考前｜终端 A。** 先在 M3-3 目录构建并预热一次 Gazebo，确认不依赖网络；预热后 Ctrl+C 关闭，不要让旧仿真和正式演示并行。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-3
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch smart_car_navigation localization.launch.py gui:=true rviz:=true navigation:=true
```

**1 一条命令起全栈｜终端 A，保持运行。** 预热已结束后，重新执行最后一条 `ros2 launch`。Gazebo 与 RViz 出现，等待地图、AMCL、规划器和控制器启动；不要在额外终端分别手动起三套服务。

**2 检查状态｜新开终端 B。**

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-3
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
source install/setup.bash
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic info /cmd_vel
```

前三项应为 `active [3]`。若 AMCL 静止时暂未产生定位更新，可先在 B 低速直行 0.3 m 后等地图变换稳定：

```bash
python3 ../M3-2/scripts/drive_route.py --straight-test 0.3 --straight-speed 0.1
ros2 topic echo /amcl_pose --once --field pose.pose.position
```

**3 考官现场给目标｜RViz 窗口。** 在地图上请考官选择距离当前车位 **大于 5 m**、包含至少一个转弯的可行目标；点击 `2D Goal Pose`，在地图点目标并拖动确定终点朝向。不要用事先固定的 `(11,3)` 冒充随机目标。观察全局路径、车体运动、到达或失败状态；在终端 B 可查 `/plan` 与 `/cmd_vel`：

```bash
ros2 topic info /plan
ros2 topic echo /cmd_vel --once
```

若考官给的是数值坐标、希望终端明确打印耗时和终点误差，改由 B **单独**发送目标（不要同时在 RViz 再点一次）：

```bash
read -r -p '考官给的地图 X: ' GOAL_X
read -r -p '考官给的地图 Y: ' GOAL_Y
read -r -p '考官给的终点朝向（度）: ' GOAL_YAW
python3 scripts/run_nav_trial.py --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --goal-yaw-deg "$GOAL_YAW" --min-travel 5
```

依提示输入考官给的数字。脚本要求车靠近起点；第二次现场跑目标须先在 A Ctrl+C 停止旧仿真，再重新一键启动，并重新核对定位。日志中记下每次成功/失败、动作耗时与失败原因；仅看 `SUCCEEDED` 不足以代替终点坐标检查。

**4 展示调参和诊断｜终端 B；RViz 窗口。**

```bash
gedit src/smart_car_navigation/config/navigation.yaml
cat results/nav_pilot_01/summary.json
cat results/nav_tuned_pilot_01/summary.json
cat results/nav_repeats_tuned/summary.json
```

对照上表解释三个参数的前后值、理由和已测结果。先在 B 检查实际发布的话题及类型：

```bash
ros2 topic list | grep -E 'costmap|^/plan$|^/cmd_vel$'
ros2 topic type /global_costmap/costmap
ros2 topic type /local_costmap/costmap
```

在 RViz 的 `Displays → Add` 中添加两个 `Map` 显示，Topic 分别选 `/global_costmap/costmap` 与 `/local_costmap/costmap`；再添加 `Path`，Topic 选 `/plan`。若实际话题类型不支持 `Map`，先依据上面的 `ros2 topic type` 结果选对应的 Nav2 Costmap 显示插件，不要凭空指定不存在的话题。车运动时，在**终端 C** 执行：

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_plot rqt_plot /cmd_vel/angular/z
```

先打开已经录好的三张图，考官要求现场重录时，在终端 C 执行下面的采集命令，再让终端 B 发送目标；采集器在约 100 秒后保存三张图和速度 CSV。若导航提前结束，仍等采集器结束或按 Ctrl+C 并检查输出。

```bash
xdg-open results/diagnostics_correct_20261122/global_costmap.png
xdg-open results/diagnostics_correct_20261122/local_costmap.png
xdg-open results/diagnostics_correct_20261122/cmd_vel_angular.png
python3 scripts/capture_nav_diagnostics.py --out "results/live_diag_$(date +%Y%m%d_%H%M%S)" --seconds 100
```

角速度曲线也可以用 `rqt_plot` 现场观察。展示时说清楚：全局图覆盖整张地图，局部图是车旁的滚动窗口，曲线显示控制器输出；这三张现存图来自一次中途失败的诊断运行。结束时在 A 按 Ctrl+C，等仿真关闭。
