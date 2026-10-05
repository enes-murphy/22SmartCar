#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scheduler.py —— 命令行任务调度模拟器（M0-4）

读一个 tasks.yaml 或 tasks.json，按依赖顺序执行任务，
模拟每次任务的成功/失败、失败重试、跳过与超时，最后写一份 report.json。

用法示例：
    python3 scheduler.py --config tasks_demo.yaml --seed 42
    python3 scheduler.py --config tasks.json --timeout 10 --report out.json
"""

import argparse
import json
import os
import random
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None

# 终端颜色码；非终端（重定向到文件）时自动降级为纯文本
_COLOR = {
    "SUCCESS": "32",   # 绿
    "FAILED": "31",    # 红
    "RETRY": "33",     # 黄
    "SKIPPED": "34",   # 蓝
    "TIMEOUT": "35",   # 紫
}


def paint(text, tag):
    """给文字上色，重定向时不输出转义符，避免 report 里一堆乱码。"""
    if sys.stdout.isatty():
        return f"\033[{_COLOR[tag]}m{text}\033[0m"
    return text


def load_config(path):
    """读配置，返回 (全局 timeout, 任务列表)。"""
    if not os.path.isfile(path):
        raise ValueError(f"找不到配置文件: {path}")
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError as e:
        raise ValueError(f"无法读取配置文件: {path}: {e}")

    if path.lower().endswith((".yaml", ".yml")):
        if yaml is None:
            raise ValueError("缺少 yaml 依赖，请先安装：pip install pyyaml")
        try:
            cfg = yaml.safe_load(text)
        except yaml.YAMLError as e:
            raise ValueError(f"配置文件 YAML 解析失败: {path}: {e}")
    else:
        try:
            cfg = json.loads(text)
        except json.JSONDecodeError as e:
            raise ValueError(f"配置文件 JSON 解析失败: {path}: {e}")

    if not isinstance(cfg, dict):
        raise ValueError(f"配置格式错误（应为对象）: {path}")
    tasks = cfg.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("配置中 tasks 必须是非空的任务列表")
    return cfg.get("timeout"), tasks


def validate_tasks(tasks):
    """校验任务字段，返回 name -> 任务信息的映射。"""
    mapping = {}
    seen = set()
    for item in tasks:
        if not isinstance(item, dict):
            raise ValueError("任务项必须是对象")
        name = item.get("name")
        if not name:
            raise ValueError("任务缺少 name 字段")
        if name in seen:
            raise ValueError(f"任务名重复: {name}")
        seen.add(name)

        sr = item.get("success_rate", 1.0)
        if not isinstance(sr, (int, float)) or not (0.0 <= sr <= 1.0):
            raise ValueError(f"任务 {name} 的 success_rate 越界: {sr}")
        duration = item.get("duration", 0.0)
        if not isinstance(duration, (int, float)) or duration < 0:
            raise ValueError(f"任务 {name} 的 duration 不合法: {duration}")
        deps = item.get("dependencies", [])
        if not isinstance(deps, list):
            raise ValueError(f"任务 {name} 的 dependencies 必须是列表")

        mapping[name] = {
            "name": name,
            "duration": float(duration),
            "success_rate": float(sr),
            "dependencies": deps,
        }

    for name, t in mapping.items():
        for d in t["dependencies"]:
            if d not in mapping:
                raise ValueError(f"任务 {name} 依赖了不存在的任务: {d}")
    return mapping


def topo_order(mapping):
    """拓扑排序，返回依赖在前、被依赖者在后的顺序；检测到环则报错。"""
    result = []
    visiting = []      # 当前 DFS 的访问栈（有序），用于还原完整的环
    done = set()

    def visit(name):
        if name in done:
            return
        if name in visiting:
            i = visiting.index(name)
            cycle = visiting[i:] + [name]
            raise ValueError(f"检测到依赖环: {' -> '.join(cycle)}")
        visiting.append(name)
        for d in mapping[name]["dependencies"]:
            visit(d)
        visiting.pop()
        done.add(name)
        result.append(name)

    for name in mapping:
        visit(name)
    return result


def simulate_once(task, rng):
    """按 success_rate 模拟一次执行，返回是否成功。"""
    return rng.random() < task["success_rate"]


def run_schedule(mapping, order, timeout, seed):
    """按拓扑序执行任务，返回 report 字典。"""
    rng = random.Random(seed)
    elapsed = 0.0
    timed_out = False

    status = {}
    attempts = {}
    started_at = {}
    ended_at = {}
    used_duration = {}

    def stamp_timed_out():
        for name in order:
            if name not in status:
                status[name] = "TIMEOUT"
                attempts.setdefault(name, 0)
                used_duration.setdefault(name, 0.0)

    for name in order:
        task = mapping[name]

        # 前置被跳过 → 自己也跳过，不执行
        if any(status.get(d) == "SKIPPED" for d in task["dependencies"]):
            status[name] = "SKIPPED"
            attempts[name] = 0
            used_duration[name] = 0.0
            print(f"  - {name}  -> {paint('SKIPPED', 'SKIPPED')}（前置未完成）")
            continue

        # 执行前检查一次超时
        if timeout is not None and elapsed >= timeout:
            timed_out = True
            break

        started_at[name] = time.time()
        tr = 0
        ok = False
        for attempt in range(1, 4):          # 最多 3 次尝试
            time.sleep(task["duration"])
            elapsed += task["duration"]
            tr += 1
            ok = simulate_once(task, rng)
            if ok:
                print(f"  - {name}  -> {paint('SUCCESS', 'SUCCESS')}（第 {attempt} 次成功）")
                break
            if attempt < 3:
                print(f"  - {name}  -> {paint('FAILED', 'FAILED')}，{paint(f'第 {attempt + 1} 次重试', 'RETRY')}")
            else:
                print(f"  - {name}  -> {paint('FAILED', 'FAILED')}，3 次均失败 → {paint('SKIPPED', 'SKIPPED')}")

            # 每次尝试之后也检查一次超时，避免"睡过头"才被发现
            if timeout is not None and elapsed >= timeout:
                timed_out = True
                break

        ended_at[name] = time.time()
        used_duration[name] = task["duration"] * tr
        attempts[name] = tr

        if timed_out:
            break
        if ok:
            status[name] = "SUCCESS"
        else:
            status[name] = "SKIPPED"

    if timed_out:
        stamp_timed_out()
        print(paint("  TIMEOUT：总耗时超过设定值，已终止所有任务", "TIMEOUT"))

    tasks_report = []
    for name in order:
        tasks_report.append({
            "name": name,
            "status": status.get(name, "TIMEOUT"),
            "attempts": attempts.get(name, 0),
            "duration": round(used_duration.get(name, 0.0), 3),
            "started_at": started_at.get(name),
            "ended_at": ended_at.get(name),
        })

    return {
        "timeout": timed_out,
        "total_duration": round(elapsed, 3),
        "tasks": tasks_report,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="命令行任务调度模拟器")
    parser.add_argument("--config", required=True, help="任务配置文件（.yaml / .yml / .json）")
    parser.add_argument("--timeout", type=float, help="覆盖配置里的 timeout（秒）")
    parser.add_argument("--report", default="report.json", help="报告输出路径，默认 report.json")
    parser.add_argument("--seed", type=int, help="随机种子，用于复现同一结果")
    args = parser.parse_args(argv)

    try:
        cfg_timeout, tasks = load_config(args.config)
        mapping = validate_tasks(tasks)
        order = topo_order(mapping)

        timeout = args.timeout if args.timeout is not None else cfg_timeout
        if timeout is not None and timeout < 0:
            raise ValueError("timeout 不能为负")
        seed = args.seed if args.seed is not None else 0

        print("=== TaskScheduler ===")
        print(f"共 {len(order)} 个任务，执行顺序: {' -> '.join(order)}")
        report = run_schedule(mapping, order, timeout, seed)

        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"\n报告已写入 {args.report}")
        print(f"总耗时 = {report['total_duration']:.3f}s，超时 = {report['timeout']}")
    except ValueError as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"错误: 未预期异常: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
