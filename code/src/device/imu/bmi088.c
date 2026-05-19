#include "bmi088.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define BMI088_USING_SPI_UNIT               hspi2

#define BMI088_TEMP_FACTOR                  0.125f
#define BMI088_TEMP_OFFSET                  23.0f

#define BMI088_WRITE_ACCEL_REG_NUM          6U
#define BMI088_WRITE_GYRO_REG_NUM           6U

#define BMI088_LONG_DELAY_TIME              80U
#define BMI088_COM_WAIT_SENSOR_TIME         150U

#define BMI088_ACCEL_3G_SEN                 0.0008974358974f
#define BMI088_GYRO_2000_SEN                0.0010652644360316953f

#define BMI088_DMA_GYRO_FRAME_LEN           7U
#define BMI088_DMA_ACCEL_FRAME_LEN          8U
#define BMI088_SPI_DUMMY_BYTE               0x55U

#define BMI088_COMPLEMENTARY_FILTER_ALPHA   0.98f
#define BMI088_PI                           3.14159265358979323846f
#define BMI088_2PI                          (2.0f * BMI088_PI)

#define BMI088_ACC_CHIP_ID                  0x00U
#define BMI088_ACC_CHIP_ID_VALUE            0x1EU
#define BMI088_ACCEL_XOUT_L                 0x12U
#define BMI088_TEMP_M                       0x22U
#define BMI088_ACC_CONF                     0x40U
#define BMI088_ACC_CONF_MUST_SET            0x80U
#define BMI088_ACC_NORMAL                   (0x2U << 4)
#define BMI088_ACC_800_HZ                   (0xBU << 0)
#define BMI088_ACC_RANGE                    0x41U
#define BMI088_ACC_RANGE_3G                 (0x0U << 0)
#define BMI088_INT1_IO_CTRL                 0x53U
#define BMI088_ACC_INT1_IO_ENABLE           (0x1U << 3)
#define BMI088_ACC_INT1_GPIO_PP             (0x0U << 2)
#define BMI088_ACC_INT1_GPIO_HIGH           (0x1U << 1)
#define BMI088_INT_MAP_DATA                 0x58U
#define BMI088_ACC_INT1_DRDY_INTERRUPT      (0x1U << 2)
#define BMI088_ACC_PWR_CONF                 0x7CU
#define BMI088_ACC_PWR_ACTIVE_MODE          0x00U
#define BMI088_ACC_PWR_CTRL                 0x7DU
#define BMI088_ACC_ENABLE_ACC_ON            0x04U
#define BMI088_ACC_SOFTRESET                0x7EU
#define BMI088_ACC_SOFTRESET_VALUE          0xB6U

#define BMI088_GYRO_CHIP_ID                 0x00U
#define BMI088_GYRO_CHIP_ID_VALUE           0x0FU
#define BMI088_GYRO_X_L                     0x02U
#define BMI088_GYRO_RANGE                   0x0FU
#define BMI088_GYRO_2000                    (0x0U << 0)
#define BMI088_GYRO_BANDWIDTH               0x10U
#define BMI088_GYRO_BANDWIDTH_MUST_SET      0x80U
#define BMI088_GYRO_1000_116_HZ             0x02U
#define BMI088_GYRO_LPM1                    0x11U
#define BMI088_GYRO_NORMAL_MODE             0x00U
#define BMI088_GYRO_SOFTRESET               0x14U
#define BMI088_GYRO_SOFTRESET_VALUE         0xB6U
#define BMI088_GYRO_CTRL                    0x15U
#define BMI088_DRDY_ON                      0x80U
#define BMI088_GYRO_INT3_INT4_IO_CONF       0x16U
#define BMI088_GYRO_INT3_GPIO_PP            (0x0U << 1)
#define BMI088_GYRO_INT3_GPIO_HIGH          (0x1U << 0)
#define BMI088_GYRO_INT3_INT4_IO_MAP        0x18U
#define BMI088_GYRO_DRDY_IO_INT3            0x01U

typedef enum {
    BMI088_DMA_IDLE = 0,
    BMI088_DMA_GYRO,
    BMI088_DMA_ACCEL
} Bmi088DmaState;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static ImuStatus bmi088_init(void);
static ImuStatus bmi088_update(void);
static ImuStatus bmi088_blocking_update(void);
static ImuAcc bmi088_get_acc(void);
static ImuGyro bmi088_get_gyro(void);
static ImuAngle bmi088_get_angle(void);
static const char* bmi088_status_str(ImuStatus status);

static void bmi088_gpio_init(void);
static void bmi088_com_init(void);
static void bmi088_delay_ms(uint16_t ms);
static void bmi088_delay_us(uint16_t us);
static void bmi088_accel_cs_low(void);
static void bmi088_accel_cs_high(void);
static void bmi088_gyro_cs_low(void);
static void bmi088_gyro_cs_high(void);
static uint8_t bmi088_read_write_byte(uint8_t tx_data);
static SPI_HandleTypeDef* bmi088_get_spi_handle(void);
static HAL_StatusTypeDef bmi088_spi_transmit_receive_dma(uint8_t* tx_data, uint8_t* rx_data, uint16_t len);

static Bmi088Error bmi088_device_init(void);
static Bmi088Error bmi088_accel_init(void);
static Bmi088Error bmi088_gyro_init(void);
static void bmi088_read_gyro_raw(float gyro[3]);
static void bmi088_read_accel_raw(float accel[3]);
static void bmi088_read_temp_raw(float* temperature);
static void bmi088_async_init(void);
static void bmi088_notify_gyro_data_ready(void);
static void bmi088_notify_accel_data_ready(void);
static bool bmi088_async_poll(void);
static bool bmi088_async_get_gyro(float gyro[3]);
static bool bmi088_async_get_accel(float accel[3]);
static void bmi088_spi_txrx_complete(SPI_HandleTypeDef* hspi);
static void bmi088_spi_error(SPI_HandleTypeDef* hspi);

static void bmi088_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088_read_single_reg(uint8_t reg, uint8_t* return_data);
static void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088_accel_read_single_reg(uint8_t reg, uint8_t* data);
static void bmi088_accel_read_burst(uint8_t reg, uint8_t* buf, uint8_t len);
static void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
static void bmi088_release_all_cs(void);
static bool bmi088_start_gyro_dma(void);
static bool bmi088_start_accel_dma(void);
static void bmi088_parse_gyro_dma_buffer(void);
static void bmi088_parse_accel_dma_buffer(void);
static void bmi088_copy_vector3(float dst[3], const float src[3]);
static void bmi088_dma_prepare_txrx(uint16_t len);
static void bmi088_dma_maintain_before_start(uint16_t len);
static void bmi088_dma_maintain_after_finish(uint16_t len);

static ImuAcc bmi088_make_acc(const float acc[3]);
static ImuGyro bmi088_make_gyro(const float gyro[3]);
static void bmi088_update_angle_by_accel(const ImuAcc* acc);
static void bmi088_update_angle_by_gyro(const ImuGyro* gyro, float dt);
static float bmi088_wrap_pi(float angle);

// ! ========================= 变 量 声 明 ========================= ! //

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

static float s_bmi088_accel_sen = BMI088_ACCEL_3G_SEN;
static float s_bmi088_gyro_sen = BMI088_GYRO_2000_SEN;

static volatile Bmi088DmaState s_bmi088_dma_state = BMI088_DMA_IDLE;
static volatile uint8_t s_bmi088_gyro_pending = 0U;
static volatile uint8_t s_bmi088_accel_pending = 0U;
static volatile uint8_t s_bmi088_gyro_ready = 0U;
static volatile uint8_t s_bmi088_accel_ready = 0U;

__attribute__((aligned(32))) static uint8_t s_bmi088_tx[BMI088_DMA_ACCEL_FRAME_LEN];
__attribute__((aligned(32))) static uint8_t s_bmi088_rx[BMI088_DMA_ACCEL_FRAME_LEN];

static float s_bmi088_gyro_dma[3];
static float s_bmi088_accel_dma[3];

static const uint8_t s_bmi088_accel_reg_init[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ERROR_ACC_PWR_CTRL},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ERROR_ACC_PWR_CONF},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_SET, BMI088_ERROR_ACC_CONF},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ERROR_ACC_RANGE},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH, BMI088_ERROR_INT1_IO_CTRL},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_ERROR_INT_MAP_DATA}
};

static const uint8_t s_bmi088_gyro_reg_init[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_ERROR_GYRO_RANGE},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET, BMI088_ERROR_GYRO_BANDWIDTH},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_ERROR_GYRO_LPM1},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_ERROR_GYRO_CTRL},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH, BMI088_ERROR_GYRO_INT3_INT4_IO_CONF},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_ERROR_GYRO_INT3_INT4_IO_MAP}
};

static ImuAcc s_bmi088_acc = { 0.0f, 0.0f, 0.0f };
static ImuGyro s_bmi088_gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle s_bmi088_angle = { 0.0f, 0.0f, 0.0f };

static Bmi088Error s_bmi088_init_error = BMI088_ERROR_NO_ERROR;
static float s_bmi088_temp = 0.0f;
static uint32_t s_bmi088_last_update_tick = 0U;
static bool s_bmi088_is_initialized = false;
static bool s_bmi088_has_gyro = false;
static bool s_bmi088_has_acc = false;

/**
 * @brief BMI088 通用 IMU 实例
 */
const ImuInterface bmi088_instance = {
    .init = bmi088_init,
    .update = bmi088_update,
    .get_acc = bmi088_get_acc,
    .get_gyro = bmi088_get_gyro,
    .get_angle = bmi088_get_angle,
    .status_str = bmi088_status_str,
};

/**
 * @brief BMI088 阻塞式 IMU 实例
 */
const ImuInterface bmi088_blocking_instance = {
    .init = bmi088_init,
    .update = bmi088_blocking_update,
    .get_acc = bmi088_get_acc,
    .get_gyro = bmi088_get_gyro,
    .get_angle = bmi088_get_angle,
    .status_str = bmi088_status_str,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取 BMI088 初始化错误码字符串
 */
const char* bmi088_error_str(Bmi088Error error) {
    switch(error) {
        case BMI088_ERROR_NO_ERROR: return "NO_ERROR";
        case BMI088_ERROR_ACC_PWR_CTRL: return "ACC_PWR_CTRL_ERROR";
        case BMI088_ERROR_ACC_PWR_CONF: return "ACC_PWR_CONF_ERROR";
        case BMI088_ERROR_ACC_CONF: return "ACC_CONF_ERROR";
        case BMI088_ERROR_ACC_SELF_TEST: return "ACC_SELF_TEST_ERROR";
        case BMI088_ERROR_ACC_RANGE: return "ACC_RANGE_ERROR";
        case BMI088_ERROR_INT1_IO_CTRL: return "INT1_IO_CTRL_ERROR";
        case BMI088_ERROR_INT_MAP_DATA: return "INT_MAP_DATA_ERROR";
        case BMI088_ERROR_GYRO_RANGE: return "GYRO_RANGE_ERROR";
        case BMI088_ERROR_GYRO_BANDWIDTH: return "GYRO_BANDWIDTH_ERROR";
        case BMI088_ERROR_GYRO_LPM1: return "GYRO_LPM1_ERROR";
        case BMI088_ERROR_GYRO_CTRL: return "GYRO_CTRL_ERROR";
        case BMI088_ERROR_GYRO_INT3_INT4_IO_CONF: return "GYRO_INT3_INT4_IO_CONF_ERROR";
        case BMI088_ERROR_GYRO_INT3_INT4_IO_MAP: return "GYRO_INT3_INT4_IO_MAP_ERROR";
        case BMI088_ERROR_SELF_TEST_ACCEL: return "SELF_TEST_ACCEL_ERROR";
        case BMI088_ERROR_SELF_TEST_GYRO: return "SELF_TEST_GYRO_ERROR";
        case BMI088_ERROR_NO_SENSOR: return "NO_SENSOR";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 获取 BMI088 最近一次初始化错误码
 */
Bmi088Error bmi088_get_init_error(void) {
    return s_bmi088_init_error;
}

/**
 * @brief 获取 BMI088 温度
 */
float bmi088_get_temp(void) {
    if(!s_bmi088_is_initialized) {
        return 0.0f;
    }

    bmi088_read_temp_raw(&s_bmi088_temp);
    return s_bmi088_temp;
}

/**
 * @brief BMI088 EXTI 回调转发
 */
void bmi088_exti_callback(uint16_t gpio_pin) {
    if(gpio_pin == GYRO_INT_Pin) {
        bmi088_notify_gyro_data_ready();
    }
    else if(gpio_pin == ACC_INT_Pin) {
        bmi088_notify_accel_data_ready();
    }
}

/**
 * @brief BMI088 SPI DMA 完成回调转发
 */
void bmi088_spi_txrx_cplt_callback(SPI_HandleTypeDef* hspi) {
    bmi088_spi_txrx_complete(hspi);
}

/**
 * @brief BMI088 SPI 错误回调转发
 */
void bmi088_spi_error_callback(SPI_HandleTypeDef* hspi) {
    bmi088_spi_error(hspi);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 BMI088 具体 IMU 实例
 */
static ImuStatus bmi088_init(void) {
    s_bmi088_init_error = bmi088_device_init();
    bmi088_async_init();

    s_bmi088_acc = (ImuAcc){ 0.0f, 0.0f, 0.0f };
    s_bmi088_gyro = (ImuGyro){ 0.0f, 0.0f, 0.0f };
    s_bmi088_angle = (ImuAngle){ 0.0f, 0.0f, 0.0f };
    s_bmi088_temp = 0.0f;
    s_bmi088_last_update_tick = HAL_GetTick();
    s_bmi088_has_gyro = false;
    s_bmi088_has_acc = false;
    s_bmi088_is_initialized = (s_bmi088_init_error == BMI088_ERROR_NO_ERROR);

    if(!s_bmi088_is_initialized) {
        return IMU_STATUS_ERROR;
    }

    return IMU_STATUS_OK;
}

/**
 * @brief 更新 BMI088 具体 IMU 实例缓存
 */
static ImuStatus bmi088_update(void) {
    float raw_acc[3] = { 0.0f };
    float raw_gyro[3] = { 0.0f };
    bool updated = false;

    if(!s_bmi088_is_initialized) {
        return IMU_STATUS_NOT_INITIALIZE;
    }

    (void)bmi088_async_poll();

    if(bmi088_async_get_gyro(raw_gyro)) {
        uint32_t now_tick = HAL_GetTick();
        float dt = (float)(now_tick - s_bmi088_last_update_tick) * 0.001f;

        s_bmi088_gyro = bmi088_make_gyro(raw_gyro);

        if(s_bmi088_has_gyro) {
            bmi088_update_angle_by_gyro(&s_bmi088_gyro, dt);
        }

        s_bmi088_last_update_tick = now_tick;
        s_bmi088_has_gyro = true;
        updated = true;
    }

    if(bmi088_async_get_accel(raw_acc)) {
        s_bmi088_acc = bmi088_make_acc(raw_acc);
        bmi088_update_angle_by_accel(&s_bmi088_acc);
        s_bmi088_has_acc = true;
        updated = true;
    }

    if(!updated) {
        return IMU_STATUS_NOT_READY;
    }

    return IMU_STATUS_OK;
}

/**
 * @brief 以阻塞读取方式更新 BMI088 具体 IMU 实例缓存
 */
static ImuStatus bmi088_blocking_update(void) {
    float raw_acc[3] = { 0.0f };
    float raw_gyro[3] = { 0.0f };
    uint32_t now_tick = 0U;
    float dt = 0.0f;

    if(!s_bmi088_is_initialized) {
        return IMU_STATUS_NOT_INITIALIZE;
    }

    bmi088_read_gyro_raw(raw_gyro);
    bmi088_read_accel_raw(raw_acc);

    now_tick = HAL_GetTick();
    dt = (float)(now_tick - s_bmi088_last_update_tick) * 0.001f;

    s_bmi088_gyro = bmi088_make_gyro(raw_gyro);
    s_bmi088_acc = bmi088_make_acc(raw_acc);

    if(s_bmi088_has_gyro) {
        bmi088_update_angle_by_gyro(&s_bmi088_gyro, dt);
    }

    bmi088_update_angle_by_accel(&s_bmi088_acc);

    s_bmi088_last_update_tick = now_tick;
    s_bmi088_has_gyro = true;
    s_bmi088_has_acc = true;
    return IMU_STATUS_OK;
}

/**
 * @brief 获取 BMI088 最近一次缓存的加速度
 */
static ImuAcc bmi088_get_acc(void) {
    return s_bmi088_acc;
}

/**
 * @brief 获取 BMI088 最近一次缓存的角速度
 */
static ImuGyro bmi088_get_gyro(void) {
    return s_bmi088_gyro;
}

/**
 * @brief 获取 BMI088 最近一次缓存的姿态角
 */
static ImuAngle bmi088_get_angle(void) {
    return s_bmi088_angle;
}

/**
 * @brief 将 IMU 状态码转换为常量字符串
 */
static const char* bmi088_status_str(ImuStatus status) {
    switch(status) {
        case IMU_STATUS_OK: return "OK";
        case IMU_STATUS_ERROR: return "ERROR";
        case IMU_STATUS_INVALID_PARAM: return "INVALID_PARAM";
        case IMU_STATUS_NO_INSTANCE: return "NO_INSTANCE";
        case IMU_STATUS_NOT_INITIALIZE: return "NOT_INITIALIZE";
        case IMU_STATUS_NOT_READY: return "NOT_READY";
        case IMU_STATUS_UNSUPPORTED: return "UNSUPPORTED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 初始化 BMI088 设备
 */
static Bmi088Error bmi088_device_init(void) {
    Bmi088Error error = BMI088_ERROR_NO_ERROR;

    bmi088_gpio_init();
    bmi088_com_init();
    bmi088_async_init();

    error = (Bmi088Error)(error | bmi088_accel_init());
    error = (Bmi088Error)(error | bmi088_gyro_init());
    return error;
}

/**
 * @brief 初始化 BMI088 加速度计
 */
static Bmi088Error bmi088_accel_init(void) {
    uint8_t res = 0U;

    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_ERROR_NO_SENSOR;
    }

    for(uint8_t write_reg_num = 0U; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        bmi088_accel_write_single_reg(
            s_bmi088_accel_reg_init[write_reg_num][0],
            s_bmi088_accel_reg_init[write_reg_num][1]);
        bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        bmi088_accel_read_single_reg(s_bmi088_accel_reg_init[write_reg_num][0], &res);
        bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if(res != s_bmi088_accel_reg_init[write_reg_num][1]) {
            return (Bmi088Error)s_bmi088_accel_reg_init[write_reg_num][2];
        }
    }

    return BMI088_ERROR_NO_ERROR;
}

/**
 * @brief 初始化 BMI088 陀螺仪
 */
static Bmi088Error bmi088_gyro_init(void) {
    uint8_t res = 0U;

    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_ERROR_NO_SENSOR;
    }

    for(uint8_t write_reg_num = 0U; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        bmi088_gyro_write_single_reg(
            s_bmi088_gyro_reg_init[write_reg_num][0],
            s_bmi088_gyro_reg_init[write_reg_num][1]);
        bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        bmi088_gyro_read_single_reg(s_bmi088_gyro_reg_init[write_reg_num][0], &res);
        bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if(res != s_bmi088_gyro_reg_init[write_reg_num][1]) {
            return (Bmi088Error)s_bmi088_gyro_reg_init[write_reg_num][2];
        }
    }

    return BMI088_ERROR_NO_ERROR;
}

/**
 * @brief 阻塞读取 BMI088 陀螺仪
 */
static void bmi088_read_gyro_raw(float gyro[3]) {
    uint8_t buf[6] = { 0U };
    int16_t raw_value = 0;

    if(gyro == 0) {
        return;
    }

    bmi088_gyro_cs_low();
    bmi088_read_write_byte(BMI088_GYRO_X_L | 0x80U);

    for(uint8_t i = 0U; i < sizeof(buf); i++) {
        buf[i] = bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
    }

    bmi088_gyro_cs_high();

    raw_value = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    gyro[0] = raw_value * s_bmi088_gyro_sen;

    raw_value = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    gyro[1] = raw_value * s_bmi088_gyro_sen;

    raw_value = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    gyro[2] = raw_value * s_bmi088_gyro_sen;
}

/**
 * @brief 阻塞读取 BMI088 加速度计
 */
static void bmi088_read_accel_raw(float accel[3]) {
    uint8_t buf[6] = { 0U };
    int16_t raw_value = 0;

    if(accel == 0) {
        return;
    }

    bmi088_accel_read_burst(BMI088_ACCEL_XOUT_L, buf, sizeof(buf));

    raw_value = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    accel[0] = raw_value * s_bmi088_accel_sen;

    raw_value = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    accel[1] = raw_value * s_bmi088_accel_sen;

    raw_value = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    accel[2] = raw_value * s_bmi088_accel_sen;
}

/**
 * @brief 阻塞读取 BMI088 温度
 */
static void bmi088_read_temp_raw(float* temperature) {
    uint8_t buf[2] = { 0U };
    int16_t raw_value = 0;

    if(temperature == 0) {
        return;
    }

    bmi088_accel_read_burst(BMI088_TEMP_M, buf, sizeof(buf));

    raw_value = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if(raw_value > 1023) {
        raw_value -= 2048;
    }

    *temperature = raw_value * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/**
 * @brief 初始化 BMI088 异步状态机
 */
static void bmi088_async_init(void) {
    s_bmi088_dma_state = BMI088_DMA_IDLE;
    s_bmi088_gyro_pending = 0U;
    s_bmi088_accel_pending = 0U;
    s_bmi088_gyro_ready = 0U;
    s_bmi088_accel_ready = 0U;
    memset(s_bmi088_tx, 0, sizeof(s_bmi088_tx));
    memset(s_bmi088_rx, 0, sizeof(s_bmi088_rx));
    memset(s_bmi088_gyro_dma, 0, sizeof(s_bmi088_gyro_dma));
    memset(s_bmi088_accel_dma, 0, sizeof(s_bmi088_accel_dma));
    bmi088_release_all_cs();
}

/**
 * @brief 标记陀螺仪数据就绪
 */
static void bmi088_notify_gyro_data_ready(void) {
    s_bmi088_gyro_pending = 1U;
}

/**
 * @brief 标记加速度计数据就绪
 */
static void bmi088_notify_accel_data_ready(void) {
    s_bmi088_accel_pending = 1U;
}

/**
 * @brief 轮询并启动 BMI088 DMA 读取
 */
static bool bmi088_async_poll(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    if(s_bmi088_gyro_pending != 0U) {
        s_bmi088_gyro_pending = 0U;
        if(bmi088_start_gyro_dma()) {
            return true;
        }

        s_bmi088_gyro_pending = 1U;
        return false;
    }

    if(s_bmi088_accel_pending != 0U) {
        s_bmi088_accel_pending = 0U;
        if(bmi088_start_accel_dma()) {
            return true;
        }

        s_bmi088_accel_pending = 1U;
        return false;
    }

    return false;
}

/**
 * @brief 获取最近一次 DMA 陀螺仪数据
 */
static bool bmi088_async_get_gyro(float gyro[3]) {
    if(gyro == 0 || s_bmi088_gyro_ready == 0U) {
        return false;
    }

    bmi088_copy_vector3(gyro, s_bmi088_gyro_dma);
    s_bmi088_gyro_ready = 0U;
    return true;
}

/**
 * @brief 获取最近一次 DMA 加速度数据
 */
static bool bmi088_async_get_accel(float accel[3]) {
    if(accel == 0 || s_bmi088_accel_ready == 0U) {
        return false;
    }

    bmi088_copy_vector3(accel, s_bmi088_accel_dma);
    s_bmi088_accel_ready = 0U;
    return true;
}

/**
 * @brief 处理 BMI088 SPI DMA 完成中断
 */
static void bmi088_spi_txrx_complete(SPI_HandleTypeDef* hspi) {
    if(hspi != bmi088_get_spi_handle()) {
        return;
    }

    if(s_bmi088_dma_state == BMI088_DMA_GYRO) {
        bmi088_gyro_cs_high();
        bmi088_dma_maintain_after_finish(BMI088_DMA_GYRO_FRAME_LEN);
        bmi088_parse_gyro_dma_buffer();
        s_bmi088_gyro_ready = 1U;
    }
    else if(s_bmi088_dma_state == BMI088_DMA_ACCEL) {
        bmi088_accel_cs_high();
        bmi088_dma_maintain_after_finish(BMI088_DMA_ACCEL_FRAME_LEN);
        bmi088_parse_accel_dma_buffer();
        s_bmi088_accel_ready = 1U;
    }

    s_bmi088_dma_state = BMI088_DMA_IDLE;
}

/**
 * @brief 处理 BMI088 SPI 错误中断
 */
static void bmi088_spi_error(SPI_HandleTypeDef* hspi) {
    if(hspi != bmi088_get_spi_handle()) {
        return;
    }

    bmi088_release_all_cs();
    s_bmi088_dma_state = BMI088_DMA_IDLE;
}

/**
 * @brief 初始化 BMI088 GPIO
 */
static void bmi088_gpio_init(void) {
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
    bmi088_delay_ms(1U);
}

/**
 * @brief 初始化 BMI088 通信接口
 */
static void bmi088_com_init(void) {}

/**
 * @brief 毫秒级阻塞延时
 */
static void bmi088_delay_ms(uint16_t ms) {
    while(ms--) {
        bmi088_delay_us(1000U);
    }
}

/**
 * @brief 微秒级阻塞延时
 */
static void bmi088_delay_us(uint16_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t told = SysTick->VAL;
    uint32_t tcnt = 0U;
    uint32_t reload = SysTick->LOAD;

    while(1) {
        uint32_t tnow = SysTick->VAL;
        if(tnow != told) {
            if(tnow < told) {
                tcnt += told - tnow;
            }
            else {
                tcnt += reload - tnow + told;
            }

            told = tnow;
            if(tcnt >= ticks) {
                break;
            }
        }
    }
}

/**
 * @brief 拉低 accel 片选
 */
static void bmi088_accel_cs_low(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 accel 片选
 */
static void bmi088_accel_cs_high(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 拉低 gyro 片选
 */
static void bmi088_gyro_cs_low(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 gyro 片选
 */
static void bmi088_gyro_cs_high(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief SPI 单字节收发
 */
static uint8_t bmi088_read_write_byte(uint8_t tx_data) {
    uint8_t rx_data = 0U;

    (void)HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &tx_data, &rx_data, 1U, 1000U);
    return rx_data;
}

/**
 * @brief 获取 BMI088 当前使用的 SPI 句柄
 */
static SPI_HandleTypeDef* bmi088_get_spi_handle(void) {
    return &BMI088_USING_SPI_UNIT;
}

/**
 * @brief 启动 BMI088 SPI DMA 收发
 */
static HAL_StatusTypeDef bmi088_spi_transmit_receive_dma(uint8_t* tx_data, uint8_t* rx_data, uint16_t len) {
    return HAL_SPI_TransmitReceive_DMA(&BMI088_USING_SPI_UNIT, tx_data, rx_data, len);
}

/**
 * @brief 向 BMI088 写单寄存器
 */
static void bmi088_write_single_reg(uint8_t reg, uint8_t data) {
    bmi088_read_write_byte(reg);
    bmi088_read_write_byte(data);
}

/**
 * @brief 从 BMI088 读单寄存器
 */
static void bmi088_read_single_reg(uint8_t reg, uint8_t* return_data) {
    bmi088_read_write_byte(reg | 0x80U);
    *return_data = bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
}

/**
 * @brief 向 BMI088 accel 写单寄存器
 */
static void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    bmi088_accel_cs_low();
    bmi088_write_single_reg(reg, data);
    bmi088_accel_cs_high();
}

/**
 * @brief 从 BMI088 accel 读单寄存器
 */
static void bmi088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
    bmi088_accel_cs_low();
    bmi088_read_write_byte(reg | 0x80U);
    bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
    *data = bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
    bmi088_accel_cs_high();
}

/**
 * @brief 从 BMI088 accel 连续读寄存器
 */
static void bmi088_accel_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    bmi088_accel_cs_low();
    bmi088_read_write_byte(reg | 0x80U);
    bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);

    while(len != 0U) {
        *buf = bmi088_read_write_byte(BMI088_SPI_DUMMY_BYTE);
        buf++;
        len--;
    }

    bmi088_accel_cs_high();
}

/**
 * @brief 向 BMI088 gyro 写单寄存器
 */
static void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
    bmi088_gyro_cs_low();
    bmi088_write_single_reg(reg, data);
    bmi088_gyro_cs_high();
}

/**
 * @brief 从 BMI088 gyro 读单寄存器
 */
static void bmi088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
    bmi088_gyro_cs_low();
    bmi088_read_single_reg(reg, data);
    bmi088_gyro_cs_high();
}

/**
 * @brief 释放所有片选
 */
static void bmi088_release_all_cs(void) {
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
}

/**
 * @brief 启动 gyro DMA 读取
 */
static bool bmi088_start_gyro_dma(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    s_bmi088_dma_state = BMI088_DMA_GYRO;
    bmi088_dma_prepare_txrx(BMI088_DMA_GYRO_FRAME_LEN);
    s_bmi088_tx[0] = BMI088_GYRO_X_L | 0x80U;
    bmi088_dma_maintain_before_start(BMI088_DMA_GYRO_FRAME_LEN);
    bmi088_gyro_cs_low();

    if(bmi088_spi_transmit_receive_dma(s_bmi088_tx, s_bmi088_rx, BMI088_DMA_GYRO_FRAME_LEN) != HAL_OK) {
        bmi088_gyro_cs_high();
        s_bmi088_dma_state = BMI088_DMA_IDLE;
        return false;
    }

    return true;
}

/**
 * @brief 启动 accel DMA 读取
 */
static bool bmi088_start_accel_dma(void) {
    if(s_bmi088_dma_state != BMI088_DMA_IDLE) {
        return false;
    }

    s_bmi088_dma_state = BMI088_DMA_ACCEL;
    bmi088_dma_prepare_txrx(BMI088_DMA_ACCEL_FRAME_LEN);
    s_bmi088_tx[0] = BMI088_ACCEL_XOUT_L | 0x80U;
    bmi088_dma_maintain_before_start(BMI088_DMA_ACCEL_FRAME_LEN);
    bmi088_accel_cs_low();

    if(bmi088_spi_transmit_receive_dma(s_bmi088_tx, s_bmi088_rx, BMI088_DMA_ACCEL_FRAME_LEN) != HAL_OK) {
        bmi088_accel_cs_high();
        s_bmi088_dma_state = BMI088_DMA_IDLE;
        return false;
    }

    return true;
}

/**
 * @brief 解析 gyro DMA 缓冲区
 */
static void bmi088_parse_gyro_dma_buffer(void) {
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[2] << 8) | s_bmi088_rx[1]);
    s_bmi088_gyro_dma[0] = raw_value * s_bmi088_gyro_sen;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[4] << 8) | s_bmi088_rx[3]);
    s_bmi088_gyro_dma[1] = raw_value * s_bmi088_gyro_sen;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[6] << 8) | s_bmi088_rx[5]);
    s_bmi088_gyro_dma[2] = raw_value * s_bmi088_gyro_sen;
}

/**
 * @brief 解析 accel DMA 缓冲区
 */
static void bmi088_parse_accel_dma_buffer(void) {
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[3] << 8) | s_bmi088_rx[2]);
    s_bmi088_accel_dma[0] = raw_value * s_bmi088_accel_sen;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[5] << 8) | s_bmi088_rx[4]);
    s_bmi088_accel_dma[1] = raw_value * s_bmi088_accel_sen;

    raw_value = (int16_t)(((uint16_t)s_bmi088_rx[7] << 8) | s_bmi088_rx[6]);
    s_bmi088_accel_dma[2] = raw_value * s_bmi088_accel_sen;
}

/**
 * @brief 拷贝三轴数据
 */
static void bmi088_copy_vector3(float dst[3], const float src[3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

/**
 * @brief 准备 DMA 收发缓冲区
 */
static void bmi088_dma_prepare_txrx(uint16_t len) {
    memset(s_bmi088_tx, BMI088_SPI_DUMMY_BYTE, len);
    memset(s_bmi088_rx, 0, len);
}

/**
 * @brief DMA 启动前维护 Cache
 */
static void bmi088_dma_maintain_before_start(uint16_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t tx_start = ((uintptr_t)s_bmi088_tx) & ~(uintptr_t)31U;
    uintptr_t rx_start = ((uintptr_t)s_bmi088_rx) & ~(uintptr_t)31U;
    int32_t tx_size = (int32_t)((((uintptr_t)s_bmi088_tx + len + 31U) & ~(uintptr_t)31U) - tx_start);
    int32_t rx_size = (int32_t)((((uintptr_t)s_bmi088_rx + len + 31U) & ~(uintptr_t)31U) - rx_start);

    SCB_CleanDCache_by_Addr((uint32_t*)tx_start, tx_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_start, rx_size);
#else
    (void)len;
#endif
}

/**
 * @brief DMA 结束后维护 Cache
 */
static void bmi088_dma_maintain_after_finish(uint16_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t rx_start = ((uintptr_t)s_bmi088_rx) & ~(uintptr_t)31U;
    int32_t rx_size = (int32_t)((((uintptr_t)s_bmi088_rx + len + 31U) & ~(uintptr_t)31U) - rx_start);

    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_start, rx_size);
#else
    (void)len;
#endif
}

/**
 * @brief 将原始三轴数组转换为加速度结构体
 */
static ImuAcc bmi088_make_acc(const float acc[3]) {
    ImuAcc result = { 0.0f, 0.0f, 0.0f };

    if(acc == 0) {
        return result;
    }

    result.x = acc[0];
    result.y = acc[1];
    result.z = acc[2];
    return result;
}

/**
 * @brief 将原始三轴数组转换为角速度结构体
 */
static ImuGyro bmi088_make_gyro(const float gyro[3]) {
    ImuGyro result = { 0.0f, 0.0f, 0.0f };

    if(gyro == 0) {
        return result;
    }

    result.x = gyro[0];
    result.y = gyro[1];
    result.z = gyro[2];
    return result;
}

/**
 * @brief 使用加速度更新 roll / pitch
 */
static void bmi088_update_angle_by_accel(const ImuAcc* acc) {
    float roll_acc = 0.0f;
    float pitch_acc = 0.0f;

    if(acc == 0) {
        return;
    }

    roll_acc = atan2f(acc->y, acc->z);
    pitch_acc = atan2f(-acc->x, sqrtf(acc->y * acc->y + acc->z * acc->z));

    if(!s_bmi088_has_acc) {
        s_bmi088_angle.roll = roll_acc;
        s_bmi088_angle.pitch = pitch_acc;
        return;
    }

    s_bmi088_angle.roll =
        BMI088_COMPLEMENTARY_FILTER_ALPHA * s_bmi088_angle.roll +
        (1.0f - BMI088_COMPLEMENTARY_FILTER_ALPHA) * roll_acc;
    s_bmi088_angle.pitch =
        BMI088_COMPLEMENTARY_FILTER_ALPHA * s_bmi088_angle.pitch +
        (1.0f - BMI088_COMPLEMENTARY_FILTER_ALPHA) * pitch_acc;
}

/**
 * @brief 使用角速度积分更新姿态角
 */
static void bmi088_update_angle_by_gyro(const ImuGyro* gyro, float dt) {
    if(gyro == 0 || dt <= 0.0f) {
        return;
    }

    s_bmi088_angle.roll += gyro->x * dt;
    s_bmi088_angle.pitch += gyro->y * dt;
    s_bmi088_angle.yaw += gyro->z * dt;

    s_bmi088_angle.roll = bmi088_wrap_pi(s_bmi088_angle.roll);
    s_bmi088_angle.pitch = bmi088_wrap_pi(s_bmi088_angle.pitch);
    s_bmi088_angle.yaw = bmi088_wrap_pi(s_bmi088_angle.yaw);
}

/**
 * @brief 将角度包裹到 [-pi, pi)
 */
static float bmi088_wrap_pi(float angle) {
    while(angle >= BMI088_PI) {
        angle -= BMI088_2PI;
    }

    while(angle < -BMI088_PI) {
        angle += BMI088_2PI;
    }

    return angle;
}
