# atlas_mission_yasmin — 智械争锋全自主区

状态机只负责比赛任务编排，不保存导航坐标、视觉像素占比或底层 `wz` 参数

主流程：

`WAIT_AUTO -> INSPECT_SORT_ZONE -> NAV_PICKUP -> OBSERVE_PICKUP -> PICK -> NAV_PARK -> OBSERVE_PARK -> PLACE -> CHECK_DONE`

规则：智能分拣区在车前进方向左侧判为 B 区，右侧判为 A 区；同一次识别返回 gear/t_bolt 的园区映射；导航后端根据 `arena=A/B` 选择对应半图

上下文只记录 4 个固定槽位的层数；待派送区初始每槽 2 层；只有抓取成功才减层，只有放置成功才给园区槽位加层；视觉内部可用目标像素占比校验 `expected_layer`，只向 YASMIN 返回 `layer_ok/complete`

识别恢复顺序：固定预识别位 -> 机械臂换视角 -> 底盘小角度 yaw 扫描；底盘扫描完成后必须回标准朝向
