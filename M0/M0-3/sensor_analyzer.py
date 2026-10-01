#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_analyzer.py  —— 上一届学长留下的"能用"的脚本
注释（学长原话）：
    "处理一下传感器数据就能用"
原本意图：
    1. 读取 sensor_data.csv（列：time, value）
    2. 计算 value 的平均值、标准差
    3. 剔除离群值（|value - mean| > 2 * std）
    4. 把清洗后的数据保存为 cleaned_data.csv
    5. 打印一份统计摘要
现状：跑不通 / 跑出来数不对。就交给你了。
"""
import csv
import math
import os
INPUT_FILE = "sensor_data.csv"
OUTPUT_FILE = "cleaned_data.csv"
OUTPUT_DIR = "out"  # 输出目录（不再用于路径拼接，见缺陷6修复）
data = []
times = []
cleaned = []
print("=== 传感器数据分析 ===")
# --- 读取数据 ---
# 修复缺陷8：用 with 管理文件句柄，避免资源泄漏
with open(INPUT_FILE, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row["time"])
        v = float(row["value"])
        times.append(t)
        data.append(v)
print("共读取 %d 条数据" % len(data))
# --- 计算平均值 ---
total = 0
for v in data:
    total += v
mean = total / len(data)
# --- 计算标准差 ---
# 修复缺陷2：总体标准差 = sqrt( Σ(v-mean)^2 / n )
acc = 0
for v in data:
    acc += (v - mean) ** 2
std = math.sqrt(acc / len(data))
# --- 剔除离群值 ---
# 修复缺陷3：|v-mean| > 2*std 的绝对值判定（偏大偏小都剔除）
# 修复缺陷4：遍历时不再修改 data（改填充保留列表 cleaned）
# 修复缺陷5：cleaned 填充为 (time, value) 对，供后续输出
cleaned = []
for t, v in zip(times, data):
    if abs(v - mean) > 2 * std:
        continue  # 离群值剔除
    cleaned.append((t, v))
# --- 输出清洗后的数据 ---
output_path = OUTPUT_FILE  # 修复缺陷6：写当前工作目录，不再写死 /out/
with open(output_path, "w") as f:  # 修复缺陷8：with 管理文件句柄
    writer = csv.writer(f)
    writer.writerow(["time", "value"])
    for t, v in cleaned:
        writer.writerow([t, v])  # 修复缺陷7：输出完整的 time, value 两列
print("均值 mean = %.4f" % mean)
print("标准差 std = %.4f" % std)
print("清洗后剩余 %d 条" % len(cleaned))
print("已保存到 %s" % output_path)
