"""corr.py — 计算两列数据的皮尔逊相关系数。

M0-2 重构入口脚本（第 1 阶段：仅封装为函数并加入口保护）。
注意：本版本只做结构重构，相关系数公式尚未修复（保留祖传 bug）。
"""
import csv
import math
import yaml


def load_config(config_path):
    """读取 YAML 配置，返回配置字典。"""
    with open(config_path) as f:
        return yaml.safe_load(f)


def load_data(csv_path, col_x, col_y):
    """从 CSV 读取两列数值，返回 (xs, ys) 两个列表。"""
    xs = []
    ys = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row[col_x]))
            ys.append(float(row[col_y]))
    return xs, ys


def compute_correlation(xs, ys):
    """计算两列数据的皮尔逊相关系数。

    当前实现把祖传逻辑原样搬入函数，公式仍为 r = prod / (dx*dy)，
    存在缺失平方根的 bug，将在后续提交中修复。
    """
    n = len(xs)
    sum_x = sum(xs)
    sum_y = sum(ys)
    mean_x = sum_x / n
    mean_y = sum_y / n
    dx = 0.0
    dy = 0.0
    prod = 0.0
    for x, y in zip(xs, ys):
        a = x - mean_x
        b = y - mean_y
        dx += a * a
        dy += b * b
        prod += a * b
    denom = dx * dy
    r = prod / denom
    return n, mean_x, mean_y, r


def main(config_path="config.yaml"):
    """主流程：读配置 -> 读数据 -> 计算 -> 打印。"""
    cfg = load_config(config_path)
    csv_path = cfg["input_csv"]
    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]
    xs, ys = load_data(csv_path, col_x, col_y)
    n, mean_x, mean_y, r = compute_correlation(xs, ys)
    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)


if __name__ == "__main__":
    main()
