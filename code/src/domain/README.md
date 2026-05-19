# domain SDK 基本说明

> `sdks/domain/` 提供与真实硬件无关的领域算法和机构模型

---

## 1. 模块定位

domain 模块只描述数学模型、坐标系、机构参数、运动学和控制量转换；它不应直接接触 CAN/UART/GPIO，也不应依赖某个芯片平台

---

## 2. 当前模块

| 模块 | 文件 | 说明 |
|---|---|---|
| six_dof_arm_kine | `six_dof_arm_kine.h` / `six_dof_arm_kine.c` | 六轴串联机械臂 FK/IK、RPY/四元数转换 |
| steer_wheel_kine | `steer_wheel_kine.h` / `steer_wheel_kine.c` | 四舵轮底盘正逆运动学 |

---

## 3. 单位和坐标约定

| 类型 | 默认单位 |
|---|---|
| 长度 | `m`，除非接口注释明确写 `mm` |
| 角度 | `rad`，除非接口注释明确写 `deg` |
| 线速度 | `m/s` |
| 角速度 | `rad/s` |
| 关节角 | `rad` |
| 时间 | `s` 或 `ms`，由字段名和注释明确 |

坐标系、正方向、轮序、关节序号和解算失败条件必须写在 public header 或 README 中

---

## 4. 设计约束

- public header 不包含 HAL/FSP/CubeMX/CMSIS 头文件
- 不直接调用 platform、device 或真实外设
- 输入输出必须明确单位、坐标系和范围
- 初始化标志、是否启用等二值状态统一使用 `bool`，public header 需要自行包含 `<stdbool.h>`
- 初始化参数必须做合法性检查
- 解算失败应返回明确错误码，而不是只输出无效数据
- 能在 PC 上测试的模块应优先写边界测试和典型工况测试

---

## 5. 新增 domain 模块检查表

- [ ] 模型参数说明完整
- [ ] 坐标系和正方向明确
- [ ] 输入输出单位明确
- [ ] 输入输出范围明确
- [ ] 失败条件明确
- [ ] 不依赖 HAL/FSP/CubeMX
- [ ] 有最小 example 或 PC 测试样例

---

## 6. 后续补充方向

- angle normalize
- chassis limiter
- trajectory profile
- coordinate transform
- ackermann kinematics
- wheel-rail chassis model
- unit conversion helper
