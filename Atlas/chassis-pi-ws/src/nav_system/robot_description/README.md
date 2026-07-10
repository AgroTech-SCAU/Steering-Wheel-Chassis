# robot_description

> **定位：** 机器人 URDF 模型与 TF 发布包。
> **核心依赖：** `robot_state_publisher`
> **下游消费者：** `at_nav2`（TF 坐标变换）、`robot_startup`（启动加载）、`robot_cartographer_mapping`（建图 TF）

---

## 1. 包概述

`robot_description` 负责定义 AGT 竞赛机器人的运动学模型（URDF 格式），并通过 `robot_state_publisher` 发布 TF 静态变换。本包不包含任何算法逻辑，纯粹是模型定义和 TF 发布。

## 2. 机器人模型

当前 URDF 包含以下连杆（link）与关节（joint）：

```
base_footprint
  └── base_joint (fixed) → base_link
        ├── laser_joint (fixed) → laser
        ├── LFs (continuous) → LFs_Link
        │     └── LFd (continuous) → LFd_Link
        ├── LRs (continuous) → LRs_Link
        │     └── LRd (continuous) → LRd_Link
        ├── RFs (continuous) → RFs_Link
        │     └── RFd (continuous) → RFd_Link
        └── RRs (continuous) → RRs_Link
              └── RRd (continuous) → RRd_Link
```

- **base_footprint**：机器人地面投影原点
- **base_link**：机器人本体（含惯性参数和碰撞几何）
- **laser**：激光雷达支架（LSLIDAR N10P 安装位置，`laser_link` 坐标系）
- **LFs/LRs/RFs/RRs**：四组全向轮转向关节（continuous 类型），每组含转向子关节和滚轮子关节

> 模型由 SolidWorks 导出，无机械臂部分。

## 3. 包结构

```
robot_description/
├── CMakeLists.txt               # 构建配置（安装 launch/urdf/meshes 到 share）
├── package.xml                  # 包依赖声明
├── README.md                    # 本文档
├── launch/
│   └── robot_description.launch.py  # 启动 robot_state_publisher
├── urdf/
│   └── robot_description.urdf       # URDF 模型文件
├── meshes/
│   ├── base_link.STL                 # 底盘 STL
│   ├── laser_frame.STL               # LiDAR 支架 STL
│   ├── LFs_Link.STL / LFd_Link.STL   # 左前轮 STL
│   ├── LRs_Link.STL / LRd_Link.STL   # 左后轮 STL
│   ├── RFs_Link.STL / RFd_Link.STL   # 右前轮 STL
│   └── RRs_Link.STL / RRd_Link.STL   # 右后轮 STL
```

## 4. 运行方式

```bash
ros2 launch robot_description robot_description.launch.py
```

该 launch 文件启动 `robot_state_publisher` 节点，加载 URDF 模型并发布 TF 静态变换（`/tf_static`）以及根据 `/joint_states` 发布的动态 TF。

## 5. TF 变换链

```
map
  └── odom (由 Cartographer / 里程计发布)
        └── base_footprint (由里程计发布)
              └── base_link (base_joint: z=0.033m)
                    ├── laser (laser_joint: z≈0.112m)
                    ├── LFs_Link → LFd_Link (左前轮)
                    ├── LRs_Link → LRd_Link (左后轮)
                    ├── RFs_Link → RFd_Link (右前轮)
                    └── RRs_Link → RRd_Link (右后轮)
```

> `base_link → laser` 为 fixed joint（z≈0.112m），即 LiDAR 安装在底盘上方约 11.2cm 处。

## 6. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| RViz2 中无机器人模型 | robot_state_publisher 未启动 | `ros2 node list \| grep robot_state_publisher` |
| TF 树不完整 | URDF 加载失败或 `/joint_states` 缺失 | `ros2 run tf2_tools view_frames` 导出 TF 树 |
| 轮子不转 | `/joint_states` 未包含轮关节 | 检查底盘驱动是否正确发布关节状态 |
