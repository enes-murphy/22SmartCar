# M0-1 环境搭建记录

# 题目简介

把开发环境配置好：装好 Ubuntu，装上 ROS2、Python、C/C++ 工具链，配好 git 和 SSH，让后面写代码有地方放、能跑、能提交。

# 本机踩坑实录

# 一：BIOS 里的 Secure Boot，我一开始当成了 Security

装 Ubuntu 双系统的时候，进 BIOS 想调启动顺序，看到安全设置（security），我以为是"secure boot"，结果security里面不能调成disabled，我不知道怎么解决。

后来查了资料、问同学才知道，Secure Boot 是"安全启动"，跟安全设置是两码事。双系统场景下它经常要关掉才能正常引导，跟我要的"双重系统共存"是冲突的。把它关了之后，系统才正常起来。

教训：看到不认识的选项，先查清楚是干嘛的再动。

# 二：装 VS Code 和开发插件，终端报"无法定位软件包 g++"

系统终于正常了，开始装环境。我想装写 C++ 要用的编译器 `g++`，在终端里敲：

sudo apt install g++

结果出现 `E: Unable to locate package g++`，我以为是包名打错了，又试了几遍，还是一样。后来搜了一下才发现，新装的系统 apt 的软件源索引还是空的，得先让它去拉一遍：

sudo apt update

更新完再装 g++，一下就装上了。以后装任何软件，第一件事先 `sudo apt update`。

# 三：WSL 默认装进了 C 盘

环境继续往下配的时候发现，装好的 WSL 发行版默认是在 C 盘的。可 C 盘本来就没多少地方了，我不想让它越占越多。后来用 WSL 自带的"导出→导入"把它整个挪到了 D 盘。

# 四：ROS2 官方源连不上，换成清华镜像

装 ROS2 的时候，官网给的那个软件源连不通，一直超时，装到一半就卡住。试了一圈发现国内镜像（清华的）能用，就把软件源换成镜像，瞬间就装好了。

# 通过什么方法 / 查了什么资料

- 主要看官方文档：Ubuntu、ROS2、WSL 官网那几篇，跟着一步步来。
- 装wsl的时候和Ubuntu的时候我跟着b站教程进行自学。Ubuntu安装：【VMware虚拟机下载安装配置Ubuntu（乌班图）操作系统！附软件安装包！】https://www.bilibili.com/video/BV1FKKA6bEDz?vd_source=6ec9242c77cefc0ad49094109bfd0936；wsl安装：【Windows电脑安装WSL【不用命令行，10分钟完成】】https://www.bilibili.com/video/BV1y8VJ6hEAs?vd_source=6ec9242c77cefc0ad49094109bfd0936
- 卡住的时候问了之前做过Ubuntu双系统的同学，也找 AI 帮忙定位问题。

# 完成了什么

- Ubuntu 环境能正常跑，ROS2 装好能启动
- Python / C/C++ 工具链都可用
- git 配好了身份，仓库建起来，能推到 GitHub
- SSH 密钥弄好，能远程登录别的机器
