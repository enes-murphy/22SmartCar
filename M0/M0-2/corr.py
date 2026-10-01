"""corr.py — 计算两列数据的皮尔逊相关系数（重构完成版）。

M0-2 入口脚本。核心计算手写，支持 --config 指定配置文件，异常统一以
可读信息输出并以非 0 退出码结束（不抛 Traceback）。

用法：
    python3 corr.py --config config.yaml
    python3 corr.py config.yaml            # 位置参数亦可
"""
import argparse
import csv
import math
import os
import sys
import yaml


class CorrError(Exception):
    """业务可读错误：统一捕获后打印友好信息并以非 0 退出。"""


def load_config(config_path):
    """读取 YAML 配置。文件不存在或解析失败时抛 CorrError。"""
    if not os.path.isfile(config_path):
        raise CorrError(f"找不到配置文件: {config_path}")
    try:
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise CorrError(f"配置文件 YAML 解析失败: {config_path}: {e}")
    if not isinstance(cfg, dict):
        raise CorrError(f"配置文件内容不是合法的映射（dict）: {config_path}")
    return cfg


def resolve_data_path(config_path, csv_path):
    """解析数据文件路径。

    支持两种相对基准：优先以配置文件所在目录为基准，其次以当前工作目录为基准；
    绝对路径直接使用。
    """
    candidates = []
    if not os.path.isabs(csv_path):
        cfg_dir = os.path.dirname(os.path.abspath(config_path))
        candidates.append(os.path.join(cfg_dir, csv_path))
    candidates.append(csv_path)
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise CorrError(
        f"找不到数据文件: {csv_path}（已尝试: {candidates}）"
    )


def load_data(csv_path, col_x, col_y):
    """从 CSV 读取两列数值，返回 (xs, ys)。

    覆盖：文件不存在、空文件、列名不匹配三种异常场景。
    """
    if not os.path.isfile(csv_path):
        raise CorrError(f"找不到数据文件: {csv_path}")
    xs = []
    ys = []
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise CorrError(f"数据文件为空或无表头: {csv_path}")
            for row in reader:
                if row[col_x] == "" or row[col_y] == "":
                    continue  # 跳过空值行
                xs.append(float(row[col_x]))
                ys.append(float(row[col_y]))
    except KeyError as e:
        raise CorrError(f"列名不匹配: {e}，请检查 csv 表头与 yaml 的 columns 配置")
    except ValueError as e:
        raise CorrError(f"数据无法转换为数值: {e}")
    if not xs:
        raise CorrError(f"数据文件无有效数据行: {csv_path}")
    return xs, ys


def compute_correlation(xs, ys):
    """计算两列数据的皮尔逊相关系数（手写核心逻辑）。

    公式：r = Σ((x-mean_x)(y-mean_y)) / sqrt(Σ(x-mean_x)^2 * Σ(y-mean_y)^2)
    覆盖：除零（某列取值恒定 → 方差为 0 → 相关系数无定义）。
    """
    n = len(xs)
    if n < 2:
        raise CorrError("数据点不足（至少需要 2 个样本）才能计算相关系数")
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
    if dx == 0.0 or dy == 0.0:
        raise CorrError("某列取值恒定（方差为 0），相关系数无定义")
    denom = math.sqrt(dx * dy)
    r = prod / denom
    return n, mean_x, mean_y, r


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="计算 CSV 中两列数据的皮尔逊相关系数"
    )
    parser.add_argument("config", nargs="?", default=None,
                        help="配置文件路径（位置参数）")
    parser.add_argument("--config", dest="config_opt",
                        help="配置文件路径（--config 方式）")
    args = parser.parse_args(argv)
    config_path = args.config_opt or args.config or "config.yaml"

    try:
        cfg = load_config(config_path)
        csv_path = cfg.get("input_csv")
        if not csv_path:
            raise CorrError("配置缺少 input_csv 字段")
        try:
            col_x = cfg["columns"]["x"]
            col_y = cfg["columns"]["y"]
        except (KeyError, TypeError):
            raise CorrError("配置缺少 columns.x / columns.y 字段")
        verbose = bool(cfg.get("task", {}).get("verbose", True))

        data_path = resolve_data_path(config_path, csv_path)
        xs, ys = load_data(data_path, col_x, col_y)
        n, mean_x, mean_y, r = compute_correlation(xs, ys)

        if verbose:
            print(f"n = {n}")
            print(f"mean_x = {mean_x:.10f}")
            print(f"mean_y = {mean_y:.10f}")
        print(f"r = {r:.10f}")
    except CorrError as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:  # 兜底：绝不把 Traceback 抛给用户
        print(f"错误: 未预期的异常: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
