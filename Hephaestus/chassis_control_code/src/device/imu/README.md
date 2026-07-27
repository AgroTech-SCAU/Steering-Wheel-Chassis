# IMU

`src/device/imu` 提供 IMU 的统一设备接口、BMI088 具体驱动，以及一个不依赖硬件的姿态融合纯算法模块

## 目录结构

```text
src/device/imu
├── imu.h
├── imu.c
├── imu_attitude.h
├── imu_attitude.c
├── bmi088.h
└── bmi088.c
```

## 模块职责

- `imu.h / imu.c`：统一 IMU 门面，只负责绑定具体实例并转发 `init/update/get_*` 等通用能力
- `bmi088.h / bmi088.c`：BMI088 具体驱动，负责 SPI 读写、EXTI/DMA 回调、数据缓存和可选姿态融合
- `imu_attitude.h / imu_attitude.c`：姿态融合纯算法，不访问硬件，不作为设备注册，只由具体 IMU 或测试代码喂入 `ImuSample` 使用

## 统一接口

上层通常只依赖 `imu.h`：

```c
#include "imu/imu.h"
#include "imu/bmi088.h"
```

初始化时先绑定具体实例，再传入该实例需要的配置：

```c
imu_set_instance(&bmi088_instance);

Bmi088Config config;
bmi088_make_config(&config, stm32_bmi088_get_ops(), ACC_INT_Pin, GYRO_INT_Pin);
imu_init(&config);
```

运行时刷新并读取缓存：

```c
if(imu_update() == IMU_STATUS_OK) {
    ImuAcc acc = imu_get_acc();
    ImuGyro gyro = imu_get_gyro();
    ImuAngle angle = imu_get_angle();
}
```

也可以使用便捷宏：

```c
imu.update();
ImuAngle angle = imu.get_angle();
```

## BMI088 实例

### `bmi088_instance`

`bmi088_instance` 是正式运行使用的 IT + SPI DMA 异步实例

它依赖：

- `Bmi088PortOps.transmit_receive_dma`
- `Bmi088PortOps.get_spi_handle`
- 加速度计和陀螺仪 data-ready EXTI 回调
- SPI DMA 完成/错误回调转发

建议同时提供：

- `Bmi088PortOps.now_us`

这样 BMI088 驱动会直接生成高精度 `*_timestamp_us`；如果 `now_us` 为空，则回退到 `now_ms() * 1000`

典型初始化：

```c
Bmi088Config config;
bmi088_make_config(&config, stm32_bmi088_get_ops(), ACC_INT_Pin, GYRO_INT_Pin);

imu_set_instance(&bmi088_instance);
imu_init(&config);

exti_register_callback(ACC_INT_Pin, bmi088_exti_callback);
exti_register_callback(GYRO_INT_Pin, bmi088_exti_callback);
spi_register_txrx_complete_callback(&hspi2, stm32_bmi088_spi_txrx_complete_callback);
spi_register_error_callback(&hspi2, stm32_bmi088_spi_error_callback);
```

### `bmi088_blocking_instance`

`bmi088_blocking_instance` 使用阻塞 SPI 读取，适合最小化调试、初始化验证和低频测试

阻塞实例不需要：

- `transmit_receive_dma`
- `get_spi_handle`
- EXTI data-ready 回调

示例：

```c
Bmi088Config config;
bmi088_make_config(&config, stm32_bmi088_get_ops(), 0, 0);

imu_set_instance(&bmi088_blocking_instance);
imu_init(&config);
```

## 姿态融合

### 设计原则

`imu_attitude.*` 是纯算法模块，它只接收 `ImuSample`，输出 `ImuAngle` 或 `ImuQuat`

当前 BMI088 驱动内部持有一个 `ImuAttitude` 实例：

- `bmi088_make_config()` 会填入默认 `Bmi088Config.attitude`
- `bmi088_update()` 收到新陀螺数据后调用 `imu_attitude_update()`
- `bmi088_get_angle()` 返回内部融合得到的姿态角
- 上层只需要正常调用 `imu_update()` 和 `imu_get_angle()`

### BMI088 中配置姿态融合

默认配置使用 Mahony 六轴融合：

```c
Bmi088Config config;
bmi088_make_config(&config, stm32_bmi088_get_ops(), ACC_INT_Pin, GYRO_INT_Pin);

config.attitude.mode = IMU_ATTITUDE_MAHONY_6AXIS;
config.attitude.gyro_calib_samples = 100;
config.attitude.acc_norm = 9.80665f;
config.attitude.acc_norm_tolerance = 2.5f;
config.attitude.mahony_kp = 2.0f;
config.attitude.mahony_ki = 0.0f;
config.attitude.mahony_ki_z = 0.0f;
config.attitude.gyro_x_temp_coeff = 0.0f;
config.attitude.gyro_y_temp_coeff = 0.0f;
config.attitude.gyro_z_temp_coeff = 0.00010f;
```

关闭姿态融合：

```c
config.attitude.mode = IMU_ATTITUDE_NONE;
```

改用互补滤波：

```c
config.attitude.mode = IMU_ATTITUDE_COMPLEMENTARY;
config.attitude.complementary_tau = 0.5f;
```

### 单独使用 `imu_attitude`

如果需要在单元测试、仿真或其他 IMU 驱动里复用算法，可以直接创建算法实例：

```c
#include "imu/imu_attitude.h"

static ImuAttitude attitude;

void attitude_init(void) {
    ImuAttitudeConfig config = {
        .mode = IMU_ATTITUDE_MAHONY_6AXIS,
        .gyro_calib_samples = 100,
        .acc_norm = 9.80665f,
        .acc_norm_tolerance = 2.5f,
        .complementary_tau = 0.5f,
        .mahony_kp = 2.0f,
        .mahony_ki = 0.0f,
        .mahony_ki_z = 0.0f,
        .gyro_x_temp_coeff = 0.0f,
        .gyro_y_temp_coeff = 0.0f,
        .gyro_z_temp_coeff = 0.00010f,
    };

    imu_attitude_init(&attitude, &config);
}

void attitude_update_from_sample(const ImuSample* sample) {
    if(imu_attitude_update(&attitude, sample) == IMU_ATTITUDE_STATUS_OK) {
        ImuAngle angle;
        imu_attitude_get_angle(&attitude, &angle);
    }
}
```

注意：

- `sample->gyro` 单位必须是 `rad/s`
- `sample->acc` 单位建议是 `m/s^2`，并与 `acc_norm` 一致
- `sample->timestamp_us` 必须单调递增
- 启动零偏校准期间会返回 `IMU_ATTITUDE_STATUS_CALIBRATING`，这不是错误
- 六轴融合没有磁力计或外部航向参考，`yaw` 只能靠陀螺积分，会随时间漂移

## 常用接口

- `imu_set_instance(instance)`：绑定具体 IMU 实例
- `imu_init(config)`：初始化当前绑定实例
- `imu_update()`：刷新当前实例缓存
- `imu_get_acc()`：读取最近三轴加速度
- `imu_get_gyro()`：读取最近三轴角速度
- `imu_get_angle()`：读取最近姿态角
- `imu_get_sample(sample)`：读取最近采样帧，主要给调试或算法复用使用
- `imu_get_gyro_corrected()`：读取扣除零偏和温漂补偿后的角速度
- `bmi088_get_temp()`：读取 `update()` 周期刷新后的 BMI088 温度缓存

## 调试建议

- 如果 `acc/gyro` 有变化但 `angle` 长期全 0，先确认姿态融合是否被关闭，以及是否已经过了 `gyro_calib_samples` 校准窗口
- 如果 `imu_update()` 长期返回 `IMU_STATUS_NOT_READY`，检查 EXTI data-ready 和 SPI DMA 回调是否已正确转发
- 如果使用阻塞实例能读到数据而异步实例读不到，优先检查 `transmit_receive_dma()`、SPI 句柄匹配和中断引脚映射
- 静止水平放置时，`acc.z` 应接近 `9.8 m/s^2`，`roll/pitch` 应接近 0
