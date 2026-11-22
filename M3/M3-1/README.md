# M3-1 坐标系与里程计

本题要把给定阿克曼车模装成可运行的 `smart_car_description` 包，使它订阅 `/cmd_vel`、发布 `/odom`，走边长 2 m 的圆角正方形；每次同时记录里程计和 Gazebo 真值，重复 10 次，计算位置及角度闭环偏差的均值与样本标准差，并说明 TF 由谁发布。

## 实现与证据

- 车模和控制器在 `src/smart_car_description/`，空地世界在 `worlds/square.world`。轴距 0.28 m，最大转向角约 30°；选择半径 0.65 m 的四个 90° 圆弧转角，每边直线长 `2 - 2×0.65 = 0.70 m`。0.10 m/s 时，一圈理论约 68.8 仿真秒。阿克曼车不能原地旋转。
- `scripts/square_driver.py` 同时驾驶和录制，支持 `--laps`、`--side`、`--out`；`scripts/analyze.py` 生成统计和等比例轨迹图。正式 10 圈原始 CSV 在 `logs/frame_aligned_repeats_01/`，每份 CSV 表头为 `t,x,y,theta`。
- 最终起点和参考点对齐后，里程计终点与 Gazebo 真值终点相差 **3.37 ± 0.46 cm**（10 圈均值 ± 样本标准差）；真值自身闭环偏差 **1.40 ± 0.36 cm**。这两个数含义不同。逐圈表在 `results/frame_aligned_repeats_01/per_run.csv`，汇总在 `results/closed_loop_stats.md`，等比例红绿轨迹、起终点及误差标注在 `results/trajectory_compare.png`。
- `results/tf_tree.pdf` 标出主链 `odom → base_footprint → base_link → lidar_link`：控制器动态发布第一段，`robot_state_publisher` 根据固定关节发布后两段；相机安装变换也为静态。M3-1 尚无 SLAM/定位，所以没有 `map → odom`。
- Gazebo 真值只做离线对照，不参与里程计修正。转弯残余误差不能仅凭位姿轨迹断言为打滑。10 圈是在同一仿真会话中连续运行，不能当成 10 次独立重启。

## 从搭车到得出结果：我们怎样做的

1. **先让车能动。** `src/smart_car_description/` 定义车体、车轮、转向关节、激光和相机，并用 Gazebo 插件接入 `ros2_control`。`ackermann_steering_controller` 消费 `geometry_msgs/Twist` 类型的 `/cmd_vel`，同时给出 `/odom`；`joint_state_broadcaster` 发布车轮和转向关节状态。控制器日志出现 `Configured and activated` 才说明真正可用，仅看 Gazebo 里出现车还不够。
2. **再核对坐标。** `/odom` 消息的 `header.frame_id=odom`、`child_frame_id=base_footprint`；动态 `odom→base_footprint` 由控制器负责，车身和传感器的固定关系由 `robot_state_publisher` 负责。最初 `tf2_echo` 报“frame does not exist”只是它尚未收到变换；随后持续输出才是有效证据。M3-1 不负责全局地图，因此此时没有 `map→odom` 是正常的。
3. **单独验证转向和记录。** 先用短直行、单次左转检查阿克曼车是否按设定方向行驶；用 `record_traj.py` 同时采 `/odom`、`/gazebo/model_states` 和必要时的 `/joint_states`。CSV 的 `t,x,y,theta` 分别是相对记录起点的时间、平面位置和航向角。`odom.csv` 是控制器估计，`truth.csv` 是 Gazebo 中模型的仿真真值；两者不能直接把未经对齐的坐标相减。记录脚本曾因重复声明 `use_sim_time` 报错，修正后才产生有效 CSV。
4. **按车辆几何设计路线。** 半径 0.65 m 的四段四分之一圆连接四段 0.70 m 直线；路线理论总长 `4×0.70 + 2π×0.65 ≈ 6.88 m`，以 0.10 m/s 行驶约 68.8 个仿真秒。先试一圈，再用 `run_trials.py` 连续跑十圈。由于转弯受控制器和车辆实际响应影响，日志打印“完成四段转弯”只表示指令时长执行完，闭环精度仍须通过轨迹检验。
5. **统一参考点后分析。** 里程计以 `base_footprint` 为参考，Gazebo 模型位姿的参考点不同；比较前应换算为同一后轴地面点，并分别减去各圈的初始位姿和朝向。`scripts/analyze.py` 负责统计和等比例轨迹图；逐圈数据见 [`results/frame_aligned_repeats_01/per_run.csv`](results/frame_aligned_repeats_01/per_run.csv)。

## 六个误差指标怎么读

位置闭环误差是同一路轨迹的终点到起点的欧氏距离；角度闭环误差是终点与起点航向差的绝对值，先归一化到 `[-π,π]`。这里的“里程计位置闭环”和“真值位置闭环”各自以自己的起点为准。另一个主要指标“**两路终点位置差**”是对齐起点和参考点后，里程计末点与真值末点之间的距离，它衡量里程计相对仿真真值的累积偏差。这三个距离不能互换。

十圈结果如下，单位和完整逐圈数值见 [`results/closed_loop_stats.md`](results/closed_loop_stats.md)。

| 指标 | 均值 ± 样本标准差 | 直观含义 |
| --- | ---: | --- |
| 真值位置闭环 | 1.40 ± 0.36 cm | 车体实际回到起点的接近程度 |
| 里程计位置闭环 | 4.24 ± 0.49 cm | 里程计认为自己回到起点的接近程度 |
| 两路终点位置差 | 3.37 ± 0.46 cm | 对齐后里程计末点与真值末点的差 |
| 真值 / 里程计角度闭环 | 0.601 ± 0.195° / 2.388 ± 0.215° | 两套坐标各自的首尾航向差 |
| 两路终点航向差 | 1.787 ± 0.203° | 对齐后两套末端朝向的差 |

“±”后的数是十个逐圈指标的**样本标准差**，分母为 `n-1=9`，不是单帧测量噪声，也不是置信区间。十圈连续运行、没有每圈重启仿真，因此这些数描述本次会话的稳定性；不能宣称覆盖不同启动、不同地面或真实车辆。图 [`results/trajectory_compare.png`](results/trajectory_compare.png) 应结合等比例坐标和数值看，不能凭视觉上两线贴近就断言误差为零。

## 按展示指南现场执行

**考前｜终端 A。** 在 M3-1 目录构建；`colcon build` 要成功。**预热之前先检查 M3-1 要用的 Gazebo 地址。** M3-2/M3-3 的普通仿真也会启动 `scene.world`，可能与 M3-1 共用默认端口 11345；即使设置 `gui:=false`，Gazebo 服务器仍会运行。M3-4 的自动试验则可能在另一个端口运行；只看 `pgrep -af gzserver` 会把它误认成冲突。这个检查只拦截占用**同一 Gazebo 地址**的服务器：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
python3 scripts/check_gazebo.py
```

看到 `OK: ... is free for M3-1` 才执行下一条。若显示 `STOP` 和 `scene.world`，先用 `pgrep -af 'ros2 launch|run_experiment.py|run_repeats.py'` 找到启动它的终端，在那里按 Ctrl+C；脚本批量运行时会反复生成新的 PID，只杀一个 `gzserver` 不够。找不到启动终端时，把检查器给出的 PID 和进程列表保存下来，再确定对应启动器；不要盲目关闭其他端口上的 M3-4 试验。停止冲突任务后重新运行 `python3 scripts/check_gazebo.py`，直到显示 `OK`。

```bash
ros2 launch smart_car_description sim.launch.py
```

M3-1 启动文件也会拒绝与同一 Gazebo 地址的已有世界共存；ROS 随后的 `InvalidFrontendLaunchFileError` 是加载失败时附带的通用提示，前面的 `RuntimeError` 才是具体原因。预热成功后，在**终端 B**加载下方环境，再执行 `python3 scripts/check_gazebo.py --expect-world square.world`：应显示 M3-1 的唯一服务器，画面应是空地。保留原始 10 圈 CSV 在本机。预热结束时在终端 A 按 Ctrl+C；待 `python3 scripts/check_gazebo.py` 再次显示 `OK: ... is free` 后，才能再次启动。

**1 起仿真｜终端 A，保持运行。** 正式展示时先运行 `python3 scripts/check_gazebo.py` 确认端口空闲，再在同一终端运行 `ros2 launch smart_car_description sim.launch.py`。等 Gazebo 空地出现且控制器激活；此终端暂时不会回到提示符。整个 M3-1 展示期间不要在同一个 Gazebo 地址上启动 M3-2 或 M3-3 仿真。

**2 查里程计｜另开终端 B。** 每个新终端都要自行加载环境：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/check_gazebo.py --expect-world square.world
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
