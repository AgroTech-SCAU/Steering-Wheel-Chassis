# atlas_mission_interfaces

比赛任务层公共接口：

- `ClassifySortingRule`：中转区一次识别返回 A/B 场和 gear/t_bolt → park_1/park_2 映射
- `StartNavigation`：YASMIN 传 `arena + waypoint_id`，导航后端维护 A/B 两张半图与真实坐标
- `DetectCameraTarget`：视觉返回货物类别、视野完整性和层数校验结果；像素占比等细节留在视觉节点内部
- `StartManipulation`：机械臂按 `slot + layer + cargo_class` 执行预识别、换视角、抓取和放置

固定语义：`arena=A/B`，`cargo_class=gear/t_bolt`，`waypoint_id=pickup/park_1/park_2`
