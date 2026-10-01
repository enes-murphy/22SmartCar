# 22SmartCar

智能车开发总仓库。基于 **Ubuntu 22.04 + ROS2 Humble**，全部代码按模块与题目划分存放。

## 仓库结构

```
22SmartCar/
├── M0/                      # 环境准备与入门
│   ├── M0-1/                # 环境搭建：README + 踩坑记录（本机实录）
│   ├── M0-2/                # 首个代码工具
│   ├── M0-3/                # 代码排查
│   └── M0-4/                # Mini 项目实战
├── M1/                      # 后续模块（按题目划分）
└── README.md                # 仓库总说明（本文件）
```

## 开发环境速览

| 组件 | 版本/说明 | 位置 |
|------|-----------|------|
| 系统 | Ubuntu 22.04.5 LTS（WSL2） | `D:\WSL\Ubuntu-22.04`（不占 C 盘） |
| 内核 | 6.6.87.2-microsoft-standard-WSL2 | — |
| ROS2 | Humble（desktop 完整版） | `/opt/ros/humble` |
| Python | 3.10.12 | 系统 `/usr/bin/python3.10` |
| 虚拟环境 | uv 0.12.21 + `.venv` | 项目根 `.venv` |
| C/C++ | gcc/g++ 11.4.0 · make 4.3 · cmake 3.22.1 | 系统 |
| 编辑器 | VS Code（Python / C/C++ / Remote-WSL 插件） | `D:\Microsoft VS Code` |
| 版本控制 | Git 2.34.1（`enesmurphy` / `zxx666276@163.com`） | — |

## 常用命令

```bash
# 进入 WSL 开发环境
wsl -d Ubuntu-22.04

# 加载 ROS2 环境（已写入 ~/.bashrc，打开终端自动生效）
source /opt/ros/humble/setup.bash

# 激活 Python 虚拟环境
cd ~/22SmartCar && source .venv/bin/activate

# 验证 ROS2
ros2 --help
rviz2            # 可视化
ros2 run turtlesim turtlesim_node   # 经典小乌龟
```

## 详细说明

- 环境搭建全过程与「本机踩坑实录」见 [M0/M0-1/README.md](M0/M0-1/README.md)
