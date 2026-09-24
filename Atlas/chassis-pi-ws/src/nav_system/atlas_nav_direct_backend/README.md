# atlas_nav_direct_backend

正式比赛轻量导航后端：任务开始时启动 Cartographer 纯定位，利用比赛地图把机器人真实位姿对齐到 field/map 坐标。全局匹配通过后保持 Cartographer 运行，对 `map -> odom` 做限速、防跳变的持续校正，消除多段往返中的里程计累计漂移。`pickup`、`park_1`、`park_2` 仍由轻量直达控制器闭环，不经过 Nav2/DWB。

速度仍发布到 `/atlas/navigation/cmd_vel`，最终由 YASMIN 的 AutoPi/Fault/EStop 安全门控后转发到 `/motor_cmd_vel`。
