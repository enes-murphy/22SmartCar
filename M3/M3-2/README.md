# M3-2｜SLAM 建图

本阶段依赖 M3-1 中可运行的小车、雷达、里程计和 TF。

## 完成清单

- [ ] 自建包含长走廊、回环和一个自定难点的 Gazebo 世界。
- [ ] 选择并配置 `slam_toolbox` 或 `cartographer`。
- [ ] 用可复现路线覆盖场景并记录 `/odom`、`/scan`、`/tf`。
- [ ] 保存标准 `map.pgm` 与 `map.yaml`。
- [ ] 定量评估墙体直线度、回环一致性和地图可用性。
- [ ] 可选：只改变一个关键参数做对比实验。

```text
M3-2/
├── worlds/
├── config/
├── maps/
├── bags/
├── scripts/
├── results/
└── README.md
```

现在不要提前实现本题；先完成 M3-1，确保输入数据可信。
