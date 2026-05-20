#include "assemble.h"

// ! system ! //
#include <assert.h>

// ! service ! //
#include "chassis.h"
#include "remote.h"

// ! device ! //
#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "rgb_led/rgb_led.h"
#include "rgb_led/ws2812_rgb_led.h"
#include "imu/imu.h"
#include "imu/bmi088.h"
#include "fs_ia10b.h"

// ! infra ! //
#include "delay.h"
#include "log.h"

// ! platform ! //
#include "stm32_hal_can.h"
#include "stm32_hal_exti.h"
#include "stm32_hal_spi.h"
#include "stm32_hal_tim.h"
#include "stm32_hal_uart.h"
#include "stm32_hal_tim.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 启动上电稳定等待时间
 *
 * 接收机、电机和电源轨在刚上电时需要先稳定一段时间；
 * 该延时集中放在装配初始化入口，避免各模块里散落过多等待
 */
#define ASSEMBLE_BOOT_SETTLE_DELAY_MS 2000u

/**
 * @brief 底盘转向电机 can_send 接口的绑定函数
 */
static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len);
/**
 * @brief 底盘驱动电机 can_send 接口的绑定函数
 */
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len);

/**
 * @brief WS2812 RGB 灯的颜色缓存
 *
 * 装配层负责把通用 RGB 接口绑定到具体 WS2812 实现；
 * 因此颜色缓存也由装配层统一持有
 */
static uint8_t rgb_color_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * RGB_LED_COLOR_BYTES];
/**
 * @brief WS2812 编码波形的 SPI 发送缓存
 *
 * 缓存放在 D3 RAM 并按 32 字节对齐；
 * 这样 SPI DMA 读取时不会被缓存行边界干扰
 */
static uint8_t rgb_tx_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * WS2812_RGB_LED_BITS_PER_PIXEL + WS2812_RGB_LED_DEFAULT_RESET_BYTES]
__attribute__((section(".ram_d3"), aligned(32)));

/**
 * @brief RGB 灯设备层使用的底层端口操作表
 *
 * 表内写接口绑定到平台 SPI 写函数；
 * RGB 设备层通过该接口发送已编码的灯带波形数据
 */
static const RgbLedPortOps rgb_ops = {
    .write = spi_write,
};

/**
 * @brief 日志模块使用的底层端口操作表
 *
 * 表内写接口绑定到 USART1 DMA 发送函数；
 * 启动和诊断日志都会通过该端口输出
 */
static const LogPortOps log_ops = {
    .write = uart1_write,
};
/**
 * @brief 底盘转向电机使用的总线端口操作表
 *
 * 组装层将抽象 send 端口绑定到 FDCAN1；
 * chassis 通过注册接口拿到该端口
 */
static const BusMotorPortOps chassis_steer_motor_ops = {
    .send = chassis_steer_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};
/**
 * @brief 底盘驱动电机使用的总线端口操作表
 *
 * 组装层将抽象 send 端口绑定到 FDCAN2；
 * chassis 通过注册接口拿到该端口
 */
static const BusMotorPortOps chassis_drive_motor_ops = {
    .send = chassis_drive_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

/**
 * @brief 底盘控制周期事件标志
 *
 * TIM6 中断只置位该标志；
 * 主循环看到标志后再执行完整底盘控制流程
 */
volatile bool tim6_500hz_flag = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 初始化异步日志服务
 *
 * 该函数绑定 USART1 DMA 写端口；
 * 后续初始化阶段的路标日志依赖它输出
 */
static void assemble_log(void);

/**
 * @brief 初始化 IMU 设备
 *
 * 函数会注册外部中断和 SPI DMA 回调；
 * 初始化失败时只记录错误日志，不阻塞整机启动
 */
static void assemble_imu(void);

/**
 * @brief 初始化 RGB 灯并设置启动指示色
 *
 * 启动阶段先显示红灯；
 * 后续由主循环根据底盘和遥控状态切换为绿灯
 */
static void assemble_rgb_led(void);

/**
 * @brief 初始化 CAN、底盘服务和控制定时器
 *
 * 该函数保留关键启动日志；
 * 用于判断冷启动卡在 CAN、底盘服务还是定时器阶段
 */
static void assemble_chassis(void);

/**
 * @brief 初始化 i.BUS 接收机和遥控服务
 *
 * 接收机初始化阶段会输出日志；
 * 遥控服务本身只负责把通道值转换为底盘速度命令
 */
static void assemble_remote(void);

/**
 * @brief 初始化 TIM6 定时器并注册控制周期回调
 *
 * TIM6 定时器以 500Hz 频率触发中断；
 * 回调函数内只设置事件标志，具体控制计算留给主循环执行
 */
static void assemble_tim6_500hz(void);

/**
 * @brief 转发 SPI 发送完成事件给 RGB 灯驱动
 *
 * 该回调由平台 SPI 层调用；
 * RGB 驱动收到完成通知后才允许下一次异步刷新
 *
 * @param hspi 触发完成事件的 SPI 句柄
 */
static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi);

/**
 * @brief 标记一个新的底盘控制周期
 *
 * 定时器中断中只设置事件标志；
 * 具体控制计算留给主循环执行
 */
static void tim6_callback(void);

/**
 * @brief 执行单个转向电机的上电准备序列
 *
 * 序列包含清错、使能、切到位置速度模式、写默认限速和刹车；
 * chassis 在初始化和重试阶段都会通过注册回调调用该函数
 *
 * @param id 转向电机 ID
 * @return BusMotorStatus 组装序列执行结果
 */
static BusMotorStatus chassis_prepare_steer_motor(uint16_t id);
/**
 * @brief 执行单个驱动电机的上电准备序列
 *
 * 序列包含清错、使能、切到速度模式、写默认限速和刹车；
 * chassis 在初始化和重试阶段都会通过注册回调调用该函数
 *
 * @param id 驱动电机 ID
 * @return BusMotorStatus 组装序列执行结果
 */
static BusMotorStatus chassis_prepare_drive_motor(uint16_t id);
/**
 * @brief 分发转向电机 CAN 反馈帧
 *
 * 平台 CAN 层收到 FDCAN1 回包后会进入该回调；
 * 回调只负责把帧转交给转向电机解析器
 *
 * @param hcan 收到帧的 FDCAN 句柄
 * @param header FDCAN 接收帧头
 * @param data 8 字节 CAN 数据
 * @param user 注册回调时传入的用户上下文
 */
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
/**
 * @brief 分发驱动电机 CAN 反馈帧
 *
 * 平台 CAN 层收到 FDCAN2 回包后会进入该回调；
 * 回调只负责把帧转交给驱动电机解析器
 *
 * @param hcan 收到帧的 FDCAN 句柄
 * @param header FDCAN 接收帧头
 * @param data 8 字节 CAN 数据
 * @param user 注册回调时传入的用户上下文
 */
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void assemble_init(void) {
    delay_ms_init(HAL_GetTick);
    delay_ms(ASSEMBLE_BOOT_SETTLE_DELAY_MS);

    assemble_log();
    log_info("BOOT log ready");
    delay_ms(100);

    assemble_rgb_led();
    log_info("BOOT rgb init step done");
    delay_ms(100);

    assemble_imu();
    log_info("BOOT imu init step done");
    delay_ms(100);

    assemble_chassis();
    log_info("BOOT chassis init step done");
    delay_ms(100);

    assemble_remote();
    log_info("BOOT remote init step done");
    delay_ms(100);

    assemble_tim6_500hz();
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

}

static void assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = true,
    };

    assert(log_init(&log_config) == LOG_STATUS_OK);
    uart_register_tx_complete_callback(&huart1, log_write_complete);
}

static void assemble_imu(void) {
    ImuStatus status = IMU_STATUS_OK;

    assert(imu_set_instance(&bmi088_instance) == IMU_STATUS_OK);

    status = imu.init();
    if(status != IMU_STATUS_OK) {
        log_error(
            "BMI088 initialization failed: %s (%s)",
            imu.status_str(status),
            bmi088_error_str(bmi088_get_init_error()));
    }

    exti_register_callback(ACC_INT_Pin, bmi088_exti_callback);
    exti_register_callback(GYRO_INT_Pin, bmi088_exti_callback);
    spi_register_txrx_complete_callback(&hspi2, bmi088_spi_txrx_cplt_callback);
    spi_register_error_callback(&hspi2, bmi088_spi_error_callback);
}

static void assemble_rgb_led(void) {
    RgbLedConfig rgb_config;

    rgb_led_set_instance(&ws2812_rgb_led_instance);
    assert(ws2812_rgb_led_make_config(
        &rgb_config,
        &rgb_ops,
        rgb_color_buffer,
        sizeof(rgb_color_buffer),
        rgb_tx_buffer,
        sizeof(rgb_tx_buffer)) == RGB_LED_STATUS_OK);

    rgb_config.async_write = true;

    assert(rgb_led.init(&rgb_config) == RGB_LED_STATUS_OK);
    spi_register_tx_complete_callback(&hspi6, rgb_led_write_complete_callback);
    rgb_led.fill(255U, 0U, 0U);
    rgb_led.show();
}

static void assemble_chassis(void) {
    ChassisConfig chassis_config = {
        .steer_motor_interface = &dm_motor_instance,
        .drive_motor_interface = &dji_motor_instance,
        .steer_ops = &chassis_steer_motor_ops,
        .drive_ops = &chassis_drive_motor_ops,
        .prepare_steer_motor = chassis_prepare_steer_motor,
        .prepare_drive_motor = chassis_prepare_drive_motor,
        .model = {
            .length = 0.26572986916f,
            .width = 0.26572986916f,
            .wheel_radius = 0.057965f,
            .max_wheel_linear_speed = 2.0f
        },
        .wheel_drive_ratio = 1.0f
    };

    log_info("CHASSIS assemble begin");
    assert(can_filter_init() == STM32_HAL_CAN_OK);
    log_info("CHASSIS CAN filter ok");
    assert(can_start(&hfdcan1) == STM32_HAL_CAN_OK);
    log_info("CHASSIS FDCAN1 start ok");
    assert(can_start(&hfdcan2) == STM32_HAL_CAN_OK);
    log_info("CHASSIS FDCAN2 start ok");

    assert(can_register_rx_callback(&hfdcan1, chassis_steer_can_rx_callback, NULL) == STM32_HAL_CAN_OK);
    assert(can_register_rx_callback(&hfdcan2, chassis_drive_can_rx_callback, NULL) == STM32_HAL_CAN_OK);

    delay_ms(100);

    assert(chassis_init(&chassis_config) == chassis.OK);
    log_info("CHASSIS service init ok");
}

static void assemble_remote(void) {
    log_info("REMOTE init begin");
    ibus_init();
    remote_init();
    log_info("REMOTE init done");
}

static void assemble_tim6_500hz(void) {
    tim_register_callback(&htim6, tim6_callback);
    tim_start();
    log_info("Tim6 started");
}

static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi) {
    (void)hspi;
    (void)rgb_led_write_complete();
}

static void tim6_callback(void) {
    tim6_500hz_flag = true;
}

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}

static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan2, id, data, len) == STM32_HAL_CAN_OK;
}

static BusMotorStatus chassis_prepare_steer_motor(uint16_t id) {
    if(dm_motor_clear_error(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(steer_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(steer_motor.switch_mode(id, DM_MOTOR_MODE_POS_VEL) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.set_spd(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.set_tor(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.brake(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    delay_ms(100);
    return MOTOR_STATUS_OK;
}

static BusMotorStatus chassis_prepare_drive_motor(uint16_t id) {
    if(drive_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.switch_mode(id, DJI_MOTOR_MODE_SPEED) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.stop(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    return MOTOR_STATUS_OK;
}

static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dm_motor_parse_feedback_frame(header->Identifier, data, NULL);
}

static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dji_motor_parse_feedback_frame(header->Identifier, data, NULL);
}
