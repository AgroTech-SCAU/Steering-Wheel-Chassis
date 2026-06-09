#include "assemble.h"

#include "arm.h"
#include "bus_servo/bus_servo.h"
#include "bus_servo/zhong_ling_servo.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_uart.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f))

// ! ========================= 变 量 声 明 ========================= ! //

#define ARM_SERVO_SPEED_RAD_S 3.14f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static inline bool servo_write(const uint8_t* data, uint16_t len) {
    return uart7_write_blocking((const char*)data, len);
}

static int servo_read(uint8_t* data, uint16_t len);
static void servo_flush_rx(void);

static const BusServoPortOps servo_port_ops = {
    .write = servo_write,
    .read = servo_read,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
    .flush_rx = servo_flush_rx,
};

static const ZhongLingServoConfig servo_config = {
    .ops = &servo_port_ops,
    .timeout_ms = 100,
    .retry_count = 3,
    .pos_min_rad = -3.1415926f * 3.0f / 4.0f,
    .pos_center_rad = 0.0f,
    .pos_max_rad = 3.1415926f * 3.0f / 4.0f,
    .pwm_min = ZHONG_LING_SERVO_PWM_MIN,
    .pwm_center = ZHONG_LING_SERVO_PWM_CENTER,
    .pwm_max = ZHONG_LING_SERVO_PWM_MAX,
    .invert = false,
    .allow_torque_ignore = false,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_arm(void) {
    ArmConfig arm_config;
    BusServoStatus servo_ret;
    ArmStatus arm_ret;

    log_info("ARM assemble begin");

    servo_ret = bus_servo_set_instance(&zhong_ling_servo_common_instance);
    if(servo_ret != SERVO_STATUS_OK) {
        log_error("ARM bus_servo_set_instance failed: %s", bus_servo_status_str(servo_ret));
        return SYSTEM_STATUS_ERROR;
    }

    servo_ret = bus_servo.init(&servo_config);
    if(servo_ret != SERVO_STATUS_OK) {
        log_error("ARM servo init failed: %s", bus_servo.status_str(servo_ret));
        return SYSTEM_STATUS_ERROR;
    }

    arm_config = arm.default_config();
    arm_config.servo_interface = &zhong_ling_servo_common_instance;
    arm_config.zhong_ling_interface = zhong_ling_servo_instance;
    arm_config.servo_zero_joints.dof = ARM_DOF;
    arm_config.servo_zero_joints.q[0] = DEG_TO_RAD(0.0f);
    arm_config.servo_zero_joints.q[1] = DEG_TO_RAD(94.5f);
    arm_config.servo_zero_joints.q[2] = DEG_TO_RAD(135.0f);
    arm_config.servo_zero_joints.q[3] = DEG_TO_RAD(-54.0f);
    arm_config.servo_zero_joints.q[4] = DEG_TO_RAD(-13.5f);

    arm_ret = arm.init(&arm_config);
    if(arm_ret != arm.OK) {
        log_error("ARM service init failed: %s", arm.status_str(arm_ret));
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ARM assemble done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static int servo_read(uint8_t* data, uint16_t len) {
    return 0;
}

static void servo_flush_rx(void) {
    return;
}
