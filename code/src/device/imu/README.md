# IMU

提供 IMU 的统一入口接口，以及 BMI088 的两种具体实例：

- `bmi088_instance`
  `IT + DMA` 异步实例，适合正式运行阶段
- `bmi088_blocking_instance`
  阻塞式实例，适合作为最小使用示例、初始化验证和低频联调

## 目录结构

```text
src/device/imu
├── imu.h
├── imu.c
├── bmi088.h
└── bmi088.c
```

## 统一接口

```c
#include "imu/imu.h"
#include "imu/bmi088.h"
```

### 阻塞式最小使用示例

适合先快速确认 BMI088 通了没有

```c
#include "imu/imu.h"
#include "imu/bmi088.h"

void app_init(void) {
    imu_set_instance(&bmi088_blocking_instance);
    imu_init();
}

void app_loop(void) {
    imu_update();

    ImuAcc acc = imu_get_acc();
    ImuGyro gyro = imu_get_gyro();
    ImuAngle angle = imu_get_angle();
    float temp = bmi088_get_temp();

    (void)acc;
    (void)gyro;
    (void)angle;
    (void)temp;
}
```

### IT + DMA 使用示例

适合运行阶段的高频采样

```c
#include "imu/imu.h"
#include "imu/bmi088.h"

void app_init(void) {
    imu_set_instance(&bmi088_instance);
    imu_init();

    exti_register_callback(ACC_INT_Pin, bmi088_exti_callback);
    exti_register_callback(GYRO_INT_Pin, bmi088_exti_callback);
    spi_register_txrx_complete_callback(&hspi2, bmi088_spi_txrx_cplt_callback);
    spi_register_error_callback(&hspi2, bmi088_spi_error_callback);
}

void app_loop(void) {
    imu_update();

    ImuAcc acc = imu_get_acc();
    ImuGyro gyro = imu_get_gyro();
    ImuAngle angle = imu_get_angle();

    (void)acc;
    (void)gyro;
    (void)angle;
}
```

## 接口说明

- `imu_init()`
  初始化当前绑定的 IMU 实例
- `imu_update()`
  刷新当前实例缓存
- `imu_get_acc()`
  返回最近一次缓存的三轴加速度 `ImuAcc`
- `imu_get_gyro()`
  返回最近一次缓存的三轴角速度 `ImuGyro`
- `imu_get_angle()`
  返回最近一次缓存的姿态角 `ImuAngle`

## BMI088 说明

- `bmi088.c` 已经自包含了 BMI088 寄存器定义、SPI 读写、阻塞读取、DMA 轮询和回调逻辑
- 当前默认使用 `SPI2`，由 `bmi088.c` 内部的 `BMI088_USING_SPI_UNIT` 绑定到 `hspi2`
- `roll / pitch` 由加速度解算并做简单互补融合
- `yaw` 由陀螺仪角速度积分得到

## 备注

- `bmi088_instance` 依赖 EXTI + SPI DMA 回调转发
- `bmi088_blocking_instance` 不依赖 EXTI 和 DMA，可直接使用
- 当前温度读取保持阻塞式，因为频率要求通常不高
