# M3-3 通用路点导航

本目录的 `scripts/run_nav_via.py` 依次发送 Nav2 地图坐标系下的中间路点和最终目标。每一段都调用 `NavigateToPose`，只有实际导航成功、终点位置检查通过，才会发送下一段。规划器单独返回一条路径不等于导航成功。

## 运行条件

终端 A 启动仿真、定位和导航，并保持运行：

```bash
cd /home/ubuntu22/Documents/Codex/2026-11-09/https-github-com-blake-john-22smartcar/work/22SmartCar/M3/M3-3
source /opt/ros/humble/setup.bash
source ../M3-1/install/setup.bash
source ../M3-2/install/setup.bash
source install/setup.bash
ros2 launch smart_car_navigation localization.launch.py gui:=true rviz:=true navigation:=true
```

等待地图、定位、规划器和控制器节点变为 active。运行路线前，小车应处于刚启动的起点附近。每次重跑，先完整退出并重新启动仿真。终端 B 也要单独执行上述 `cd` 和四行 `source`，然后运行下面的路线命令。

## 通用命令

在 M3-3 目录的另一个终端执行：

```bash
python3 scripts/run_nav_via.py \
  --via 3 0 0 \
  --goal-x 7 --goal-y 8 --goal-yaw-deg 180 \
  --min-waypoint-travel 2.5 --min-final-travel 5
```

`--via X Y YAW_DEG` 是地图坐标系中的中间位置和车头朝向（度），可以重复多次；最后三个 `--goal-*` 参数给出终点。`--min-waypoint-travel` 和 `--min-final-travel` 检查各段首尾里程计的直线位移，单位米，默认均为 0；它们不等于实际行驶路径长度。`--timeout` 是每段的最长墙上时钟秒数，默认 180。加入 `--dry-run` 只打印路点顺序，不会发送导航目标。

本项目已验证的 `(7,8,180°)` 路线可直接运行 `python3 scripts/run_nav_7_8.py`。它是上述通用脚本的预设调用，不含另一套控制算法：先沿底部通道到 `(3,0,0°)`，再到最终目标。直接从起点向 `(7,8,180°)` 发送单个导航目标曾在中央障碍的左下角中止。

脚本最终打印 `SUCCEEDED: reached map ... through N Nav2 legs` 才表示所有路段完成。任何一段失败或被中断，后续路段都不会发送；查看该段的 Nav2 日志，再从起点重新启动仿真。一个目标在地图上显示为空闲，或 `check_plan.py` 找到了路径，都不能保证实际导航到达。
