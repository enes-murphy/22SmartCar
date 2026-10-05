# M0-4 · Mini 项目实战：TaskScheduler

一个跑在终端里的任务调度模拟器。读一份 `tasks.yaml` 或 `tasks.json`，按依赖顺序执行任务，
模拟每个任务的成败、失败重试、跳过和超时，最后写一份 `report.json`。

全程不依赖 ROS 和 GUI，就在终端跑。

## 环境

用 M0-1 搭好的虚拟环境运行（uv 建的 `.venv`）：

```bash
cd /home/enes/22SmartCar
source .venv/bin/activate
pip install -r M0/M0-4/requirements.txt
```

## 参数说明

| 参数 | 必填 | 说明 |
| :-- | :--: | :-- |
| `--config <path>` | 是 | 任务配置文件，支持 `.yaml` / `.yml` / `.json` |
| `--timeout <float>` | 否 | 覆盖配置文件里的全局 timeout（秒） |
| `--report <path>` | 否 | 报告输出路径，默认 `report.json` |
| `--seed <int>` | 否 | 随机种子，固定后同样输入能复现同样结果 |

## 配置文件格式

`tasks.yaml`（或等价 `.json`）：

```yaml
timeout: 15          # 可选，全局超时（秒）
tasks:
  - name: "Navigate"        # 任务名（唯一）
    duration: 2.5           # 预期耗时（秒），程序会 sleep 这么久模拟
    success_rate: 0.8       # 模拟成功率 0.0 ~ 1.0
    dependencies: ["Init"]  # 前置任务名列表
```

字段说明：

| 字段 | 说明 |
| :-- | :-- |
| `timeout` | 全局超时（秒），可用 `--timeout` 覆盖 |
| `name` | 任务名，必须唯一 |
| `duration` | 每次尝试的模拟耗时（秒） |
| `success_rate` | 单次模拟成功率，0~1 |
| `dependencies` | 前置任务名列表，前置全部完成后才能执行 |

## 运行示例

### 示例 1：正常调度（依赖失败会向下游传播）

```bash
python3 scheduler.py --config tasks_demo.yaml --seed 42
```

输出（`AvoidObstacle` 三次失败后 `SKIPPED`，依赖它的 `Place`、`ReturnHome` 也 `SKIPPED`）：

```
=== TaskScheduler ===
共 7 个任务，执行顺序: Init -> Navigate -> DetectQR -> Grasp -> AvoidObstacle -> Place -> ReturnHome
  - Init  -> SUCCESS（第 1 次成功）
  - Navigate  -> SUCCESS（第 1 次成功）
  - DetectQR  -> SUCCESS（第 1 次成功）
  - Grasp  -> SUCCESS（第 1 次成功）
  - AvoidObstacle  -> FAILED，第 2 次重试
  - AvoidObstacle  -> FAILED，第 3 次重试
  - AvoidObstacle  -> FAILED，3 次均失败 → SKIPPED
  - Place  -> SKIPPED（前置未完成）
  - ReturnHome  -> SKIPPED（前置未完成）

报告已写入 report.json
总耗时 = 11.500s，超时 = False
```

### 示例 2：超时中断

```bash
python3 scheduler.py --config tasks_demo.yaml --seed 42 --timeout 3
```

输出（总耗时到 3 秒立即终止，未执行的任务标记 `TIMEOUT`）：

```
=== TaskScheduler ===
共 7 个任务，执行顺序: Init -> Navigate -> DetectQR -> Grasp -> AvoidObstacle -> Place -> ReturnHome
  - Init  -> SUCCESS（第 1 次成功）
  - Navigate  -> SUCCESS（第 1 次成功）
  TIMEOUT：总耗时超过设定值，已终止所有任务

报告已写入 report_timeout.json
总耗时 = 3.000s，超时 = True
```

### 示例 3：依赖成环报错

```bash
python3 scheduler.py --config cycle.yaml
```

输出（可读错误，非 0 退出，不抛栈回溯）：

```
错误: 检测到依赖环: A -> B -> A
```

### 示例 4：用 json 配置

```bash
python3 scheduler.py --config tasks_demo.json --seed 1
```

## 实现要点

- **拓扑排序**：用 DFS 求执行顺序，天然处理分叉（一个任务被多个任务依赖）和汇合（一个任务依赖多个前置），遇到环直接报错。
- **依赖失败传播**：只要前置里有 `SKIPPED`，自己就不执行、直接 `SKIPPED`。
- **失败重试**：每个任务最多尝试 3 次，3 次都失败记 `SKIPPED`。
- **超时判定**：执行前检查一次，每次 `sleep` 之后再检查一次，避免最后一个任务"睡过头"才发现。
- **彩色输出**：`SUCCESS` 绿、`FAILED` 红、`RETRY` 黄、`SKIPPED` 蓝；重定向到文件时自动去掉转义符。

## report.json 结构

一次运行后生成的报告：

```json
{
  "timeout": false,
  "total_duration": 11.5,
  "tasks": [
    {
      "name": "Init",
      "status": "SUCCESS",
      "attempts": 1,
      "duration": 0.5,
      "started_at": 1791211082.37,
      "ended_at": 1791211082.87
    }
  ]
}
```

- `status` 取值：`SUCCESS` / `FAILED` / `SKIPPED` / `TIMEOUT`
- `attempts`：该任务的尝试次数（整数，最多 3）
- `tasks` 会包含配置里的每一个任务（包括被跳过和超时未执行的）

## 说明

本任务部分代码和文档由 AI 辅助完成；核心逻辑（拓扑排序、重试与跳过、超时、报告输出）均在本机虚拟环境里实际跑通并验证过，示例输出即为真实运行结果。
