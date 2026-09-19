# atlas_mission_yasmin — 智械争锋全自主区

状态机只负责比赛任务编排，不保存导航坐标、视觉像素比例或底层运动学参数

主流程：

`WAIT_AUTO -> ARM_ZERO -> NAV_ORIGIN -> INSPECT_SORT_ZONE -> ARM_NAV_SAFE_INITIAL -> NAV_PICKUP -> OBSERVE_PICKUP -> PICK -> ...`

智能分拣区一次识别确定 `arena=A/B` 与 gear/t_bolt 的园区映射，导航后端按 arena 选择对应半图

## 货物调度

`CompetitionModel` 只维护真实场上状态：4 个货物点当前剩余层数、手中货物、园区堆叠与已送达数量。调度规则集中在独立 `PickupScheduler`，不再由 `CompetitionModel` 内部的多阶段 phase FSM 控制

每个半区先按两个点位各两次主访问执行。实际抓取层永远以该点当前剩余层为准，因此第一次高层失败后，第二次访问仍抓高层；若第二次才抓到高层，会立即再给刚暴露的低层一次机会。半区主访问结束后，对仍有货物的点位做 retry pass；一整轮 retry 有进展就继续补抓，一整轮毫无进展则记录 abandoned 并进入下一半区，优先保证完整比赛流程不会卡死

`round` 只用于日志/人机显示，不再参与调度决策。只有抓取成功才减少真实剩余层数，纯视觉 miss 或 cargo-local 抓取失败只通知 scheduler 推进/defer

## 视觉恢复

识别恢复顺序：固定预识别位 -> 机械臂换视角 -> 底盘小角度 yaw 扫描；底盘扫描完成后必须回标准朝向。语义视觉 miss 属于货物局部失败，系统级 reset/recovery/shutdown 仍正常向外传播

## 抓取职责

`PICK` 只选择 `slot + layer`。机械臂接触位直接复用 `handeye_bridge/screw_pick.launch.py` 已实机验证链路：bridge 使用 `bridge_node.yaml` 的 plane1/plane2 高度、初始抓取姿态和顶层配置中的 `target_z_offset_m` 完成视觉到 `SetArmPose`。任务层不再覆盖 `target_z / pitch / yaw / approach`；bridge 到位后 manipulation backend 开吸盘、保持真空，再通过 3D `SetArmPosition` 抬升
