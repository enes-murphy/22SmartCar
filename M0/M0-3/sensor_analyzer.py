#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_analyzer.py  —— 传感器数据清洗工具（M0-3 修复完成版）

原本意图：
    1. 读取 CSV（列：time, value）
    2. 计算 value 的平均值 mean、标准差 std（总体标准差）
    3. 剔除离群值（|value - mean| > 2 * std）
    4. 把清洗后的数据保存为 cleaned_data.csv
    5. 打印统计摘要

用法：
    python3 sensor_analyzer.py --input sensor_data.csv --output cleaned_data.csv
    python3 sensor_analyzer.py        # 无参数：默认读写当前目录下的
                                      # sensor_data.csv / cleaned_data.csv

修复记录：共修复 8 处缺陷，详见 README.md。
"""
import argparse
import csv
import math
import os
import sys

DEFAULT_INPUT = "sensor_data.csv"
DEFAULT_OUTPUT = "cleaned_data.csv"


def load_data(input_path):
    """读取 CSV，返回 (times, values)。异常以 ValueError 抛可读信息。"""
    if not os.path.isfile(input_path):
        raise ValueError(f"找不到输入文件: {input_path}")
    times = []
    values = []
    try:
        with open(input_path, "r") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"输入文件为空或无表头: {input_path}")
            for row in reader:
                times.append(float(row["time"]))
                values.append(float(row["value"]))
    except KeyError as e:
        raise ValueError(f"缺少列: {e}（CSV 需要 time 与 value 两列）")
    except (ValueError, csv.Error) as e:
        raise ValueError(f"数据格式错误: {e}")
    if not values:
        raise ValueError(f"输入文件无有效数据: {input_path}")
    return times, values


def compute_mean(values):
    return sum(values) / len(values)


def compute_std(values, mean):
    """总体标准差 = sqrt( Σ(v-mean)^2 / n )"""
    acc = sum((v - mean) ** 2 for v in values)
    return math.sqrt(acc / len(values))


def remove_outliers(times, values, mean, std):
    """剔除 |value - mean| > 2*std 的离群点，返回清洗后的 (time, value) 列表。"""
    cleaned = []
    for t, v in zip(times, values):
        if abs(v - mean) > 2 * std:
            continue
        cleaned.append((t, v))
    return cleaned


def save_cleaned(output_path, cleaned):
    """把清洗结果写入 CSV（time, value 两列）。"""
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "value"])
        writer.writerows(cleaned)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="传感器数据清洗：剔除离群值并输出统计摘要"
    )
    parser.add_argument("--input", default=DEFAULT_INPUT,
                        help=f"输入 CSV 路径（默认 {DEFAULT_INPUT}）")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"输出 CSV 路径（默认 {DEFAULT_OUTPUT}）")
    args = parser.parse_args(argv)

    print("=== 传感器数据分析 ===")
    try:
        times, values = load_data(args.input)
        print(f"共读取 {len(values)} 条数据")
        mean = compute_mean(values)
        std = compute_std(values, mean)
        cleaned = remove_outliers(times, values, mean, std)
        save_cleaned(args.output, cleaned)
        print(f"均值 mean = {mean:.4f}")
        print(f"标准差 std = {std:.4f}")
        print(f"清洗后剩余 {len(cleaned)} 条")
        print(f"已保存到 {args.output}")
    except (ValueError, OSError) as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:  # 兜底：不裸崩
        print(f"错误: 未预期异常: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
