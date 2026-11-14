#!/usr/bin/env python3
"""Evaluate the fixed M3-2 world map and a completed ROS 2 sqlite bag."""

import argparse
import math
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def line_quality(occupied, axis, lo, hi, search_lo, search_hi, resolution,
                 exclude=None):
    """RMS perpendicular distance to a fitted straight wall, in metres."""
    samples = []
    for coordinate in range(lo, hi):
        if exclude and exclude[0] <= coordinate < exclude[1]:
            continue
        candidates = np.flatnonzero(occupied[search_lo:search_hi, coordinate]) if axis == "x" else \
            np.flatnonzero(occupied[coordinate, search_lo:search_hi])
        if len(candidates):
            samples.append((coordinate, search_lo + float(np.median(candidates))))
    expected = hi - lo - (exclude[1] - exclude[0] if exclude else 0)
    if len(samples) < expected * 0.8:
        raise RuntimeError(f"Too few wall samples in ROI {axis} {lo}:{hi}")
    coords, walls = np.array(samples).T
    slope, intercept = np.polyfit(coords, walls, 1)
    residual = (walls - (slope * coords + intercept)) * resolution
    return len(samples), float(np.sqrt(np.mean(residual ** 2))), float(np.percentile(np.abs(residual), 95))


def bag_pose(db, topic_id, stamp, msg_type):
    row = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? "
                     "ORDER BY ABS(timestamp-?) LIMIT 1", (topic_id, stamp)).fetchone()
    return row[0], deserialize_message(row[1], msg_type)


def model_pose(message):
    p = message.pose[message.name.index("smart_car")]
    return p.position.x, p.position.y, yaw(p.orientation)


def map_pose(db, ids, stamp):
    _, odom = bag_pose(db, ids["/odom"], stamp, Odometry)
    # /tf contains both dynamic robot TF and the SLAM map->odom correction.
    rows = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? "
                      "AND timestamp BETWEEN ? AND ? ORDER BY timestamp",
                      (ids["/tf"], stamp - 300_000_000, stamp + 300_000_000))
    corrections = []
    for t, blob in rows:
        for tr in deserialize_message(blob, TFMessage).transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                corrections.append((abs(t - stamp), tr))
    if not corrections:
        raise RuntimeError("No map->odom TF near driving endpoint")
    tr = min(corrections, key=lambda item: item[0])[1].transform
    theta = yaw(tr.rotation)
    p = odom.pose.pose.position
    return (tr.translation.x + math.cos(theta) * p.x - math.sin(theta) * p.y,
            tr.translation.y + math.sin(theta) * p.x + math.cos(theta) * p.y,
            wrap(theta + yaw(odom.pose.pose.orientation)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=Path("maps/map.yaml"))
    parser.add_argument("--bag", type=Path, default=Path("bags/formal_02/formal_02_0.db3"))
    parser.add_argument("--out", type=Path, default=Path("results/quality_report.md"))
    args = parser.parse_args()

    meta = yaml.safe_load(args.map.read_text(encoding="utf-8"))
    image = np.array(Image.open(args.map.parent / meta["image"]))
    occupied = image < 100
    free = image == 254
    scale = float(meta["resolution"])
    # These ROIs target the central island in this specific 14 x 10 m world.
    walls = {
        "central north (10 m)": line_quality(occupied, "x", 50, 231, 35, 48, scale),
        "central south (10 m)": line_quality(occupied, "x", 50, 231, 154, 168, scale),
        "central west (6 m)": line_quality(occupied, "y", 50, 151, 34, 48, scale),
        "central east (gate excluded)": line_quality(
            occupied, "y", 50, 151, 234, 251, scale, exclude=(88, 114)),
    }
    # A separated duplicate wall would enlarge the occupied span across a
    # long-wall cross-section. Report this proxy separately from line RMS.
    spans = []
    for column in range(50, 231):
        for low, high in ((35, 48), (154, 168)):
            cells = np.flatnonzero(occupied[low:high, column])
            if len(cells):
                spans.append(int(np.ptp(cells)) + 1)
    for row in list(range(50, 88)) + list(range(114, 151)):
        cells = np.flatnonzero(occupied[row, 234:251])
        if len(cells):
            spans.append(int(np.ptp(cells)) + 1)
    # At the middle of the gate, find the last cell of the inner baffle and
    # first cell of the outer baffle. All cells between must be observed free.
    widths = []
    for row in range(98, 104):
        inner = np.flatnonzero(occupied[row, 245:256]) + 245
        outer = np.flatnonzero(occupied[row, 267:280]) + 267
        if len(inner) and len(outer):
            left, right = int(inner.max()), int(outer.min())
            if right > left and np.all(free[row, left + 1:right]):
                widths.append(right - left - 1)
    if not widths:
        raise RuntimeError("Gate is not a continuous observed-free passage")
    gate_cells = min(widths)

    db = sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True)
    ids = dict(db.execute("SELECT name,id FROM topics"))
    required = ("/cmd_vel", "/odom", "/scan", "/tf", "/tf_static", "/map", "/gazebo/model_states")
    counts = {name: db.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (ids[name],)).fetchone()[0]
              for name in required}
    commands = [(t, deserialize_message(blob, Twist)) for t, blob in
                db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (ids["/cmd_vel"],))]
    moving = [t for t, message in commands if message.linear.x > 0.01]
    start, end = moving[0], moving[-1] + 1_000_000_000
    _, start_model = bag_pose(db, ids["/gazebo/model_states"], start, ModelStates)
    _, end_model = bag_pose(db, ids["/gazebo/model_states"], end, ModelStates)
    truth_start, truth_end = model_pose(start_model), model_pose(end_model)
    slam_start, slam_end = map_pose(db, ids, start), map_pose(db, ids, end)
    truth_xy = math.dist(truth_start[:2], truth_end[:2])
    slam_xy = math.dist(slam_start[:2], slam_end[:2])
    truth_yaw = abs(math.degrees(wrap(truth_end[2] - truth_start[2])))
    slam_yaw = abs(math.degrees(wrap(slam_end[2] - slam_start[2])))

    lines = [
        "# M3-2 正式地图质量评价", "",
        "来源：`maps/map.yaml`、`maps/map.pgm` 和本地 `bags/formal_02/`。",
        f"分辨率 {scale:.3f} m/格；地图 {image.shape[1]} × {image.shape[0]} 格。", "",
        "取样区域见 [quality_regions.png](quality_regions.png)：红框为长墙，绿框为窄门。", "",
        "## 数据完整性", "",
        *[f"- `{name}`：{count} 条" for name, count in counts.items()], "",
        "## 墙体直线度", "",
        "每条墙在避开角点的区域逐列或逐行取占据格中位数，拟合一条直线；",
        "RMS 是墙点到拟合线的均方根距离，95% 是绝对偏差的第 95 百分位。",
        "整体地图旋转不会改变这个指标。", "",
        "| 墙段 | 样本数 | RMS 偏差 | 95% 偏差 |", "|---|---:|---:|---:|",
        *[f"| {name} | {n} | {rms:.3f} m | {p95:.3f} m |"
          for name, (n, rms, p95) in walls.items()], "",
        "## 回环与定位一致性", "",
        f"驾驶首尾 Gazebo 真值位置间距 {truth_xy:.3f} m，航向差 {truth_yaw:.2f}°。",
        f"驾驶首尾 `map` 坐标系下的里程计融合位姿间距 {slam_xy:.3f} m，航向差 {slam_yaw:.2f}°。",
        "后者用同期 `map→odom` TF 与 `/odom` 位姿合成；两个闭合距离并非逐点轨迹误差。",
        f"上下长墙及避开门口的东侧墙横截面最大占据跨度 {max(spans)} 格，"
        f"即 {max(spans) * scale:.2f} m；在这些取样段未见明显分离的双层墙。",
        "这只是最终地图的重影代理指标，不能证明回环优化事件确实触发。", "",
        "## 导航可用性：窄门", "",
        f"门口中央连续已知空闲宽度至少 {gate_cells} 格，即 {gate_cells * scale:.2f} m。",
        "这个净宽尚未扣除车体外形及导航安全膨胀；M3-3 需要再做代价地图验证。", "",
    ]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    figure = Image.open(args.map.parent / meta["image"]).convert("RGB")
    figure = figure.resize((image.shape[1] * 3, image.shape[0] * 3), Image.NEAREST)
    pen = ImageDraw.Draw(figure)
    for box in ((50, 35, 230, 47), (50, 154, 230, 167),
                (34, 50, 47, 150), (234, 50, 250, 87), (234, 114, 250, 150)):
        pen.rectangle(tuple(value * 3 for value in box), outline="red", width=2)
    pen.rectangle((245 * 3, 98 * 3, 279 * 3, 103 * 3), outline="lime", width=2)
    figure.save(args.out.parent / "quality_regions.png")
    args.out.write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
