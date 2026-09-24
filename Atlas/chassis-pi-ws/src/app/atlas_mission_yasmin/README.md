# atlas_mission_yasmin — 智械争锋全自主区

状态机只负责比赛任务编排，不保存导航坐标、视觉像素比例或底层运动学参数

主流程：

`WAIT_AUTO -> ARM_ZERO -> INSPECT_SORT_ZONE -> NAV_ORIGIN -> ARM_NAV_SAFE_INITIAL -> NAV_PICKUP -> OBSERVE_PICKUP -> PICK -> ...`

智能分拣区先由机械臂移动到观察位，一次识别确定 `arena=A/B` 与 gear/t_bolt 的园区映射；随后 `NAV_ORIGIN` 将该 arena 传给导航后端，只加载对应半图完成地图匹配与原点校正

## 货物调度

`CompetitionModel` 只维护真实场上状态：4 个货物点当前剩余层数、手中货物、园区堆叠与已送达数量。调度规则集中在独立 `PickupScheduler`，不再由 `CompetitionModel` 内部的多阶段 phase FSM 控制

每个半区先按两个点位各两次主访问执行。实际抓取层永远以该点当前剩余层为准，因此第一次高层失败后，第二次访问仍抓高层；若第二次才抓到高层，会立即再给刚暴露的低层一次机会。半区主访问结束后，对仍有货物的点位做 retry pass；一整轮 retry 有进展就继续补抓，一整轮毫无进展则记录 abandoned 并进入下一半区，优先保证完整比赛流程不会卡死

`round` 只用于日志/人机显示，不再参与调度决策。只有抓取成功才减少真实剩余层数，纯视觉 miss 或 cargo-local 抓取失败只通知 scheduler 推进/defer

## 视觉恢复

识别恢复顺序：固定预识别位 -> 机械臂换视角 -> 底盘小角度 yaw 扫描；底盘扫描完成后必须回标准朝向。语义视觉 miss 属于货物局部失败，系统级 reset/recovery/shutdown 仍正常向外传播

## 抓取职责

`PICK` 只选择 `slot + layer`。机械臂接触位直接复用 `handeye_bridge/screw_pick.launch.py` 链路：bridge 使用当前 A/B、slot 观察位标定的第一/二层 `layer_z_m`，第三层继续使用 `bridge_node.yaml` 的 `plane3_z_m`，并使用顶层配置中的 `target_z_offset_m` 完成视觉到 `SetArmPose`。抓第一/二层时，bridge 会先到同一目标 XY 的第三层等待高度，收到实时 FK 到位确认后再下探到目标层。任务层不覆盖 `target_z / pitch / yaw / approach`；bridge 到最终接触位后 manipulation backend 开吸盘、保持真空，再保持工具轴用 5D `SetArmPose` 抬升

园区放置的工具方向优先使用每个 `placement_points` 的实测 `pitch_rad/yaw_rad`；
兼容旧标定时复用对应 `park.prepare` 的工具方向，不再强制竖直向下。放置过程
任一阶段失败或取消都会强制尝试关闭吸盘；正常退出的机械臂命令也会重复携带
`suction OFF`，防止单帧命令丢失。
