# Step 3—5 交付与验证

## 当前状态

| 项目 | 结果 |
|---|---|
| Step 1/2 已有 IK 修复 | 保留，主机回归通过 |
| Step 3 Pi 全程工具轴保持 | 已实现并通过专项测试 |
| Step 4 MCU → Pi → Pick 真实结果 | 已实现，主机协议与回调测试通过 |
| Step 5 视觉有效性 | 已实现，专项测试通过 |
| Step 5 deferred retry | 本基线逻辑已正确，新增回归通过，未放宽 scheduler 校验 |
| Step 6 集中真机验收 | NOT RUN，需要现场执行 |

## 主要改动

- handeye 从检测帧对应变换提取工具 z 轴，停止依赖旧 initial magic RPY
- lift、view scan、place 三段全部使用 SetArmPose，place 工具轴向下
- MCU 新增 0x28 结果帧和有界序号结果缓存，Pi 发布 ArmCommandResult
- ACCEPTED 来自实际 ArmStatus，分别处理无解、参数错误、运动学失败、舵机失败、超时和未知结果
- PickResult 按 request_id 关联视觉拒绝或 MCU 结果，并携带 contact 目标
- 上层先等真实接受，再等新鲜、不同时间戳的 XYZ 和工具轴到位反馈
- 视觉门禁要求 TCP 在 0.2 s 窗口位移小于 1 mm、工具轴误差小于 5°，并检查反馈新鲜度
- 切换观察位、移动、过期门禁和检测重启清理旧帧
- 补齐取消抓取后禁止继续下探、取消旧任务未退出时拒绝新任务的回归

## 实际验证

运行 `run_host_regressions.sh`，依赖 Python pytest、numpy、PyYAML 及主机 cc/g++

| 验证 | 实际结果 |
|---|---|
| manipulation、handeye、detector 专项 Python 测试 | 59 passed |
| 通用 SO(3)、5D/6D、A/B pickup 和 park IK 主机回归 | ALL TESTS PASSED |
| MCU ArmStatus 映射、重复帧、超时、取消、UART 重试 | PASS |
| MCU 未初始化与非 AutoPi 模式拒绝 | PASS |
| Pi 结果帧校验和线协议往返 | PASS |
| CompetitionModel 主机断言测试 | 11 tests, 0 failed |

C++ 调度测试通过包内简易断言适配器执行现有 gtest 风格测试，不等同于 ROS/ament 构建

MCU 协议测试使用模拟 UART/arm 服务，不能代替物理舵机和真实 IK 拒绝验收

## 审查与执行判断

独立代码审查提出三项 Important 问题，均已修复并有针对性回归

1. handeye 在门禁失效后保留旧最终帧 → 清理检测缓存、限制当前 readiness 时间区间
2. 取消 approach 后仍可能发送 contact → 按 request_id 取消后续阶段
3. backend 把旧反馈接收时间当新采样 → 检查消息源时间戳、新鲜度及重复样本

执行判断

- 使用上传 Step 1/2 包作为续作基线，未新建分支、commit 或 push
- PickResult 增加最终目标，避免把 MCU 接受误当成实际到位；部署须重新生成消息
- review 中 scheduler lookup 改变游标的前提现已不成立，保留既有合法 slot 校验；如果现场使用其他版本，须重新核对该版本
- 取消只阻止后续自动阶段，不改变已接受动作的物理停止策略

## 未验证边界

当前环境无 arm-none-eabi-gcc、colcon/ROS 2，因此完整固件编译、消息生成、ROS 节点链接、安装后资源解析、实时参数、UART/DDS 联调和真机矩阵均未执行

结果缓存为最近 16 条 MCU 命令，跨 MCU 重启不保留；全部重传结果丢失时上层按超时处理

源码保留原标定与偏置，未引入 Pi 第二套 IK

完整逐文件清单见 `CHANGED_FILES.txt`，现场操作见 `../../STEP6_ON_ROBOT.md`
