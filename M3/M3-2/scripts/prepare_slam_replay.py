#!/usr/bin/env python3
"""Make a SLAM input bag from formal_02, excluding its recorded SLAM outputs."""

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage


INPUT_TOPICS = {"/clock", "/scan", "/tf", "/tf_static", "/odom"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=Path("bags/formal_02"))
    parser.add_argument("--out", type=Path, default=Path("bags/slam_replay_inputs"))
    args = parser.parse_args()
    if args.out.exists():
        parser.error(f"output already exists: {args.out}")

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    metadata = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    missing = INPUT_TOPICS - metadata.keys()
    if missing:
        raise RuntimeError(f"missing topics: {sorted(missing)}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.out), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name in sorted(INPUT_TOPICS):
        writer.create_topic(metadata[name])
    counts = {name: 0 for name in INPUT_TOPICS}
    stripped = 0
    while reader.has_next():
        name, blob, stamp = reader.read_next()
        if name not in INPUT_TOPICS:
            continue
        if name == "/tf":
            msg = deserialize_message(blob, TFMessage)
            kept = [tr for tr in msg.transforms
                    if not (tr.header.frame_id == "map" and tr.child_frame_id == "odom")]
            stripped += len(msg.transforms) - len(kept)
            if not kept:
                continue
            msg.transforms = kept
            blob = serialize_message(msg)
        writer.write(name, blob, stamp)
        counts[name] += 1
    del writer
    print(f"Input bag: {args.bag}")
    print(f"Replay bag: {args.out}")
    print(f"Messages: {counts}")
    print(f"Removed recorded map->odom transforms: {stripped}")


if __name__ == "__main__":
    main()
