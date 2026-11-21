#!/usr/bin/env python3
"""Draw the final M3-1 TF topology from the expanded Xacro and controller YAML."""

import argparse
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def quote(value):
    value = str(value).replace('\\n', '\n')
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", type=Path, default=Path("src/smart_car_description/urdf/smart_car.urdf.xacro"))
    parser.add_argument("--controller", type=Path, default=Path("src/smart_car_description/config/ackermann_controllers.yaml"))
    parser.add_argument("--out-prefix", type=Path, default=Path("results/tf_tree"))
    args = parser.parse_args()
    xml = subprocess.run(["xacro", str(args.xacro)], check=True,
                         capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    controller = args.controller.read_text(encoding="utf-8")
    base = re.search(r"^\s*base_frame_id:\s*(\w+)", controller, re.MULTILINE)
    odom = re.search(r"^\s*odom_frame_id:\s*(\w+)", controller, re.MULTILINE)
    enabled = re.search(r"^\s*enable_odom_tf:\s*(true|false)", controller, re.MULTILINE)
    if not (base and odom and enabled and enabled.group(1) == "true"):
        raise ValueError("Controller YAML does not enable an identifiable odom TF")

    meanings = {
        "odom": "continuous local frame",
        "base_footprint": "rear-axle ground point",
        "base_link": "vehicle body",
        "lidar_link": "laser mount",
        "camera_link": "camera mount",
    }
    lines = [
        "digraph M3_1_final_TF {",
        '  graph [rankdir=TB, bgcolor="white", fontname="DejaVu Sans",',
        '         label="M3-1 final TF topology (derived from expanded Xacro + controller YAML)\\nmap -> odom is absent: no localization or SLAM in this task",',
        '         labelloc=t, fontsize=18, pad=0.3, nodesep=0.4, ranksep=0.8];',
        '  node [shape=box, style="rounded,filled", fillcolor="#f7f9fb",',
        '        color="#738496", fontname="DejaVu Sans", fontsize=11];',
        '  edge [fontname="DejaVu Sans", fontsize=9, arrowsize=0.7];',
    ]
    links = [(odom.group(1), base.group(1), "dynamic ~50 Hz\\nackermann_steering_controller", "#b3262b")]
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name, child_name = parent.attrib["link"], child.attrib["link"]
        if joint.attrib["type"] == "fixed":
            label = "static\\nrobot_state_publisher"
            if joint.attrib["name"] in ("base_footprint_joint", "lidar_joint", "camera_joint"):
                origin = joint.find("origin")
                if origin is not None:
                    xyz = origin.attrib.get("xyz", "0 0 0")
                    label += "\\nxyz=" + " ".join(f"{float(x):.3f}" for x in xyz.split())
            color = "#2267a0"
        else:
            label = "dynamic\\nrobot_state_publisher\\nuses /joint_states"
            color = "#b3262b"
        links.append((parent_name, child_name, label, color))

    names = {name for a, b, _, _ in links for name in (a, b)}
    for name in sorted(names):
        detail = meanings.get(name)
        label = name if detail is None else f"{name}\\n{detail}"
        lines.append(f"  {quote(name)} [label={quote(label)}];")
    for parent, child, label, color in links:
        lines.append(f"  {quote(parent)} -> {quote(child)} [label={quote(label)}, color={quote(color)}, fontcolor={quote(color)}];")
    lines += [
        '  "map_note" [shape=note, fillcolor="#fff7e5", color="#be8124",',
        '              label="map -> odom: NOT PUBLISHED\\nFuture SLAM/localization would provide it"];',
        '  "map_note" -> "odom" [style=invis];',
        '  {rank=same; "map_note"; "odom";}',
        "}",
    ]
    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)
    dot_path = args.out_prefix.with_suffix(".gv")
    dot_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    for fmt in ("pdf", "png"):
        subprocess.run(["dot", f"-T{fmt}", str(dot_path), "-o",
                        str(args.out_prefix.with_suffix("." + fmt))], check=True)
    print(f"Saved {dot_path}, {args.out_prefix.with_suffix('.pdf')}, "
          f"{args.out_prefix.with_suffix('.png')}")


if __name__ == "__main__":
    main()
