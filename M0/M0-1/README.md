# M0-1・环境搭建说明 + 踩坑实录

> 目标：在本机（Windows 物理机 + WSL2）搭建 
>
> **Ubuntu 22.04 + ROS2 Humble**
>
>  完整开发环境，全程
>
> **不占 C 盘**
>
> 。
> 完成时间：2026-10-01



***

## 一、最终环境一览



| 组件                 | 安装方式      | 版本                                                     | 是否占 C 盘          |
| ------------------ | --------- | ------------------------------------------------------ | ---------------- |
| Ubuntu 22.04.5 LTS | WSL2 发行版  | 22.04.5 LTS（jammy）                                     | ❌ 已迁移至 D 盘       |
| ROS2 Humble        | apt（清华镜像） | Humble（desktop 完整版）                                    | ❌（位于 D 盘 VHDX 内） |
| Python             | 系统自带      | 3.10.12                                                | ❌                |
| 虚拟环境               | uv        | uv 0.12.21                                             | ❌                |
| C/C++ 工具链          | apt       | gcc/g++ 11.4.0 · make 4.3 · cmake 3.22.1 · ninja · gdb | ❌                |
| 编辑器                | VS Code   | 最新 + Python/C++/Remote-WSL 插件                          | ❌（本体在 D 盘）       |
| Git                | apt       | 2.34.1（已配置身份）                                          | ❌                |

> 「不占 C 盘」的实现原理：WSL2 的 Ubuntu 整个文件系统封装在 
>
> `ext4.vhdx`
>
>  虚拟磁盘里，只要把这个发行版迁移到 D 盘，里面装的 ROS2 / Python / 工具链就全部落在 D 盘。



***

## 二、安装步骤（分阶段）

### 1. 确认 WSL 与发行版



```
wsl --list --verbose
#   NAME            STATE      VERSION
# * Ubuntu-22.04    Running    2        ← 正是 ROS2 Humble 官方支持的版本
wsl --status        # 确认 WSL2
```

### 2. 把发行版迁移到 D 盘（关键，见踩坑 #1）



```
wsl --shutdown
wsl --export  Ubuntu-22.04  D:\WSL\ubuntu-22.04-backup.tar   # 全量备份，约 1.15GB
wsl --unregister Ubuntu-22.04                                # 注销 C 盘原发行版
wsl --import   Ubuntu-22.04  D:\WSL\Ubuntu-22.04  D:\WSL\ubuntu-22.04-backup.tar --version 2
# 验证：注册表 HKCU:\...\Lxss 的 BasePath 应变为 D:\WSL\Ubuntu-22.04
```

### 3. 更新系统 + 基础工具（以 root 执行）



```
wsl -d Ubuntu-22.04 -u root   # 以 root 进入，避开 sudo 交互密码（见踩坑 #4）
apt-get update
# 基础包：locales software-properties-common curl wget git gnupg lsb-release
#         build-essential gcc g++ gdb make cmake ninja-build pkg-config
#         python3 python3-pip python3-venv openssh-client sshfs rsync net-tools
```

### 4. 设置 locale（ROS2 硬性要求）



```
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

### 5. 安装 ROS2 Humble（清华镜像，见踩坑 #3）



```
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
chmod 0644 /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list
apt-get update
apt-get install -y ros-humble-desktop ros-dev-tools ros-humble-turtlesim
# 写入 ~/.bashrc：source /opt/ros/humble/setup.bash
```

### 6. Python 虚拟环境（uv）



```
curl -LsSf https://astral.sh/uv/install.sh | sh          # 安装到 ~/.local/bin
cd ~/22SmartCar && uv venv .venv --python 3.10           # 创建项目虚拟环境
source .venv/bin/activate                                # 激活验证
```

### 7. VS Code（本体已在 D 盘）



```
# 已装于 D:\Microsoft VS Code，补装插件：
& 'D:\Microsoft VS Code\bin\code.cmd' --install-extension ms-python.python --force
& 'D:\Microsoft VS Code\bin\code.cmd' --install-extension ms-vscode.cpptools --force
& 'D:\Microsoft VS Code\bin\code.cmd' --install-extension ms-vscode-remote.remote-wsl --force
# 日常：在 Windows 的 VS Code 里打开远程 WSL 窗口，即可用 Linux 工具链开发
```

### 8. Git 身份（见踩坑 #5）



```
git config --global user.name  "enesmurphy"
git config --global user.email "zxx666276@163.com"
git config --global init.defaultBranch main
```

### 9. SSH 免密登录



```
ssh-keygen -t ed25519 -C "enes@DESKTOP-C5T7QF4" -N "" -f ~/.ssh/id_ed25519
# 公钥已生成，可加到 GitHub / 开发板 / 服务器：
#   ssh-copy-id user@host   （Linux 服务器）
#   或手动粘贴到 GitHub Settings → SSH keys
```



***

## 三、本机踩坑实录（重点）

> 以下全部为本次搭建过程中真实遇到、并已验证解决的坑。

### 坑 1：WSL 默认装在 C 盘，违反「不装 C 盘」要求



* **现象**：明明只有 WSL，C 盘却越来越少。查询发行版安装位置才发现默认在 C 盘。

* **报错 / 现象原文**：



```
BasePath : C:\Users\DELL\AppData\Local\wsl\{ed694c88-...}
```



* **定位方法**：读注册表拿到发行版真实路径



```
Get-ItemProperty 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss\*' |
  Select DistributionName, BasePath
```

看到 `BasePath = C:\...`，确认发行版在 C 盘。



* **解决**：用 WSL 官方推荐的 导出→注销→导入 三步，整体搬到 `D:\WSL`，验证 BasePath 变为 `\\?\D:\WSL\Ubuntu-22.04`，C 盘残留清除（Test-Path 返回 False）。

* **教训**：WSL 发行版默认落 C 盘，要 "不占 C 盘" 必须显式迁移；迁移前一定先 `wsl --export` 全量备份。

### 坑 2：PowerShell 调用 WSL 时参数引号被吞，命令不执行



* **现象**：`wsl -d Ubuntu-22.04 -- bash -lc 'echo $USER'` 这类命令要么**无任何输出**、要么**卡住 15 秒后被强制转后台**。

* **定位方法**：先用最小命令对比



```
wsl -d Ubuntu-22.04 -e echo hello    # 正常输出 hello → 说明 WSL 本身没问题
```

对比后发现：问题出在 PowerShell → `wsl.exe` → `bash` 之间多层引号 / 变量 `$()` 的嵌套转义。



* **解决**：不再内联传复杂命令，改成**把命令写成&#x20;**`.sh`**&#x20;脚本文件，用&#x20;**`wsl -d Ubuntu-22.04 -e bash /mnt/d/WSL/scripts/xxx.sh`**&#x20;执行**。脚本路径放在 D 盘（`/mnt/d/WSL/scripts`），彻底绕开引号地狱。

* **教训**：跨 PowerShell 和 WSL 传多段命令极易在引号层出问题，脚本文件是最稳的通道。

### 坑 3：官方 ROS2 源 `packages.ros.org` 超时（国内网络）



* **现象**：`curl https://packages.ros.org/ros2/ubuntu/...` 返回 HTTP 000（连接超时 / 失败），apt 无法用官方源装 ROS2。

* **定位方法**：写网络探测脚本一次性测多个源的连通性：



```
[ros-packages] packages.ros.org      -> HTTP 000   ← 官方源不可达
[ros-raw-key]  raw.githubusercontent.com/.../ros.key -> HTTP 200  ← 但 key 能拿
[tuna-ros2]    mirrors.tuna.tsinghua.edu.cn/ros2/... -> HTTP 200  ← 清华镜像可用
[aliyun-ros2]  mirrors.aliyun.com/ros2/...            -> HTTP 200  ← 阿里镜像也可用
```

结论：key 仍从官方 GitHub raw 拿（可达），**包源改用清华镜像**。



* **解决**：`/etc/apt/sources.list.d/ros2.list` 指向 `https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main`，随后 apt 安装顺利。

* **教训**：国内装 ROS2 先把源换成镜像，能省大量时间；key 与包源可以分开选择。

### 坑 4：非交互执行 `sudo` 需要密码，自动安装卡住



* **现象**：脚本里 `sudo apt-get install ...` 在无人值守时等待密码输入，安装无法推进。

* **定位**：`sudo -n true`（非交互测试）失败，说明该用户 sudo 需密码。

* **解决**：WSL 支持直接以 root 身份执行系统级安装：



```
wsl -d Ubuntu-22.04 -u root -e bash /path/setup.sh
```

系统级 apt 安装用 root，用户级配置（git、uv、ssh）用 `enes`，分工清晰。



* **教训**：无人值守环境别用 sudo，直接 `-u root` 更省事；但注意日常开发仍用普通用户。

### 坑 5：WSL 的 stdout 经 PowerShell 回传不稳定、被截断



* **现象**：同样一段脚本，有时只回传最后一行，中间过程丢失，难以判断哪步失败。

* **定位**：发现回传输出与脚本实际执行顺序不一致，判定是 stdout 回传通道的问题，而非脚本逻辑。

* **解决**：脚本内把输出**重定向写入 D 盘日志文件**（`/mnt/d/WSL/logs/*.log`），执行完用文件读取工具读日志确认每步结果，稳定可靠。

* **教训**：自动化安装务必落日志，别只依赖回显。

### 坑 6：装完 desktop 版 `turtlesim_node` 仍是缺失



* **现象**：`ros-humble-desktop` 装完，`command -v turtlesim_node` 无结果。

* **定位**：`ls /opt/ros/humble/lib/turtlesim/` 发现目录里没有该节点；turtlesim 作为教学示例不一定随 desktop 包附带。

* **解决**：`apt-get install -y ros-humble-turtlesim` 补装，随后 `turtlesim_node` / `turtle_teleop_key` 均可用。

* **教训**：desktop 版≠所有演示包都在，按需补装即可。

### 坑 7：uv 装在 `~/.local/bin`，默认 PATH 找不到（check_env.sh 报 FAIL）

* **现象**：跑现场验收脚本 `check_env.sh`，第 6 项报 `[FAIL] 未发现 uv 或 conda`，但 `uv --version` 明明能用。
* **定位**：发现 uv 实际装在 `~/.local/bin/uv`，而**非交互 shell 的默认 PATH 不含 `~/.local/bin`**，脚本的 `command -v uv` 自然找不到。
* **解决**：把 uv 软链到系统 PATH 目录，保证任何 shell 都能找到：
  ```bash
  sudo ln -sf ~/.local/bin/uv  /usr/local/bin/uv
  sudo ln -sf ~/.local/bin/uvx /usr/local/bin/uvx
  ```
  同时把 `export PATH="$HOME/.local/bin:$PATH"` 追加进 `~/.bashrc`（交互 shell 使用也便利）。
* **教训**：用 `curl | sh` 装的工具常在 `~/.local/bin`，该目录默认不在 PATH；跑验收脚本前要么软链到系统目录、要么确保 PATH 包含它。



***

## 四、参考资料



* ROS2 Humble 官方安装（Deb 包）：[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

* WSL 官方文档（安装 / 发行版 / 文件位置）：[https://learn.microsoft.com/zh-cn/windows/wsl/](https://learn.microsoft.com/zh-cn/windows/wsl/)

* WSL 发行版导出与导入（迁移到其他盘）：[https://learn.microsoft.com/zh-cn/windows/wsl/use-custom-distro](https://learn.microsoft.com/zh-cn/windows/wsl/use-custom-distro)

* 清华 TUNA ROS2 镜像帮助页：[https://mirrors.tuna.tsinghua.edu.cn/help/ros2/](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)

* uv 官方文档（Python 包 / 环境管理）：[https://docs.astral.sh/uv/](https://docs.astral.sh/uv/)

* VS Code Remote-WSL 说明：[https://code.visualstudio.com/docs/remote/wsl](https://code.visualstudio.com/docs/remote/wsl)



***

## 五、验证清单（全部通过）



* [x] `wsl --status` → WSL2；Ubuntu-22.04 位于 `D:\WSL\Ubuntu-22.04`，C 盘无残留

* [x] `source /opt/ros/humble/setup.bash` 后 `$ROS_DISTRO=humble`、`ros2 --help` 正常

* [x] `rviz2`、`ros2 run turtlesim turtlesim_node` 可用

* [x] `gcc/g++ 11.4.0`、`make 4.3`、`cmake 3.22.1`、`git 2.34.1` 可用

* [x] `uv --version` → 0.12.21；`source .venv/bin/activate` 激活后 `python 3.10.12`

* [x] Git 身份已配置（`enesmurphy` / `zxx666276@163.com`）

* [x] `~/.ssh/id_ed25519` 已生成（公钥可免密推送）
* [x] `check_env.sh` 现场验收脚本：**FAIL=0**（PASS=27 / WARN=2），uv 已软链至系统 PATH