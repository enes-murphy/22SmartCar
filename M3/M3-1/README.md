# M3-1 坐标系与里程计

本题要把给定阿克曼车模装成可运行的 `smart_car_description` 包，使它订阅 `/cmd_vel`、发布 `/odom`，走边长 2 m 的圆角正方形；每次同时记录里程计和 Gazebo 真值，重复 10 次，计算位置及角度闭环偏差的均值与样本标准差，并说明 TF 由谁发布。

## 实现与证据

- 车模和控制器在 `src/smart_car_description/`，空地世界在 `worlds/square.world`。轴距 0.28 m，最大转向角约 30°；选择半径 0.65 m 的四个 90° 圆弧转角，每边直线长 `2 - 2×0.65 = 0.70 m`。0.10 m/s 时，一圈理论约 68.8 仿真秒。阿克曼车不能原地旋转。
- `scripts/square_driver.py` 同时驾驶和录制，支持 `--laps`、`--side`、`--out`；`scripts/analyze.py` 生成统计和等比例轨迹图。正式 10 圈原始 CSV 在 `logs/frame_aligned_repeats_01/`，每份 CSV 表头为 `t,x,y,theta`。
- 最终起点和参考点对齐后，里程计终点与 Gazebo 真值终点相差 **3.37 ± 0.46 cm**（10 圈均值 ± 样本标准差）；真值自身闭环偏差 **1.40 ± 0.36 cm**。这两个数含义不同。逐圈表在 `results/frame_aligned_repeats_01/per_run.csv`，汇总在 `results/closed_loop_stats.md`，等比例红绿轨迹、起终点及误差标注在 `results/trajectory_compare.png`。
- `results/tf_tree.pdf` 标出主链 `odom → base_footprint → base_link → lidar_link`：控制器动态发布第一段，`robot_state_publisher` 根据固定关节发布后两段；相机安装变换也为静态。M3-1 尚无 SLAM/定位，所以没有 `map → odom`。
- Gazebo 真值只做离线对照，不参与里程计修正。转弯残余误差不能仅凭位姿轨迹断言为打滑。10 圈是在同一仿真会话中连续运行，不能当成 10 次独立重启。

## 按展示指南现场执行

**考前｜终端 A。** 在 M3-1 目录构建并预热 Gazebo；`colcon build` 要成功。预热后按 Ctrl+C 退出，正式展示时再启动。保留原始 10 圈 CSV 在本机。

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch smart_car_description sim.launch.py
```

**1 起仿真｜终端 A，保持运行。** 如已结束预热，在同一终端再次运行 `ros2 launch smart_car_description sim.launch.py`。等 Gazebo 出现且控制器激活；此终端暂时不会回到提示符。

**2 查里程计｜另开终端 B。** 每个新终端都要自行加载环境：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /odom
```

看到频率后按 Ctrl+C，继续在 B 执行：

```bash
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

向考官指出 `frame_id: odom`、`child_frame_id: base_footprint`；`/cmd_vel` 有订阅者。`/gazebo/model_states` 是仿真真值。

**3 现场跑一圈并录两路｜终端 B；A 继续运行。**

```bash
DEMO_LOGS="logs/live_$(date +%Y%m%d_%H%M%S)"
python3 scripts/square_driver.py --laps 1 --side 2.0 --radius 0.65 --speed 0.1 --out "$DEMO_LOGS"
head -n 2 "$DEMO_LOGS/run_01_odom.csv"
head -n 2 "$DEMO_LOGS/run_01_truth.csv"
```

等 `Run 1/1: complete`；两份 CSV 应为 `t,x,y,theta`。现场的一圈只证明可运行，不能用它计算十圈标准差。若考官要求从零跑满十圈，仍在 B 用**新目录**执行：

```bash
TEN_LOGS="logs/exam10_$(date +%Y%m%d_%H%M%S)"
TEN_RESULTS="results/$(basename "$TEN_LOGS")"
python3 scripts/square_driver.py --laps 10 --side 2.0 --radius 0.65 --speed 0.1 --out "$TEN_LOGS"
python3 scripts/analyze.py --logs "$TEN_LOGS" --expected-runs 10 --out "$TEN_RESULTS"
```

十圈驾驶理论就约 11.5 分钟，会超过指南给的约十分钟展示时长，短场考核先展示预录的十圈数据。

**4 看统计与图｜终端 B；图像查看器窗口。**

```bash
head -n 11 results/frame_aligned_repeats_01/per_run.csv
cat results/closed_loop_stats.md
xdg-open results/trajectory_compare.png
```

图上指出红色 `/odom`、绿色真值、等比例坐标、起终点及闭环误差。十圈位置、角度偏差都要报均值 ± 标准差。

**5 看 TF｜终端 B；PDF 查看器窗口。**

```bash
xdg-open results/tf_tree.pdf
```

逐边解释动态或静态、发布者，并明确说 M3-1 **不应有** `map → odom`，那是后续 SLAM/定位的职责。结束时先关图，再在终端 A 按 Ctrl+C，等 Gazebo 退出。
