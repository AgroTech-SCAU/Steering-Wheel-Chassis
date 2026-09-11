# atlas_nav_direct_backend

正式比赛轻量导航后端：任务开始时临时启动 Cartographer 纯定位，利用比赛地图把当前机器人真实位姿对齐到 field/map 坐标；定位稳定后冻结 `map -> odom`，关闭临时定位进程，并先回到地图原点。此后 `pickup`、`park_1`、`park_2` 不再使用 Nav2/Cartographer 控制，只依赖 MCU 已融合 IMU 的 `/odom` 直接闭环到点。

速度仍发布到 `/atlas/navigation/cmd_vel`，最终由 YASMIN 的 AutoPi/Fault/EStop 安全门控后转发到 `/motor_cmd_vel`。
