#include "assemble.h"

#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static BusMotorStatus chassis_prepare_steer_motor(uint16_t id);
static BusMotorStatus chassis_prepare_drive_motor(uint16_t id);
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);

// ! ========================= 变 量 声 明 ========================= ! //

static const BusMotorPortOps chassis_steer_motor_ops = {
    .send = chassis_steer_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

static const BusMotorPortOps chassis_drive_motor_ops = {
    .send = chassis_drive_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_chassis(void) {
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
            .wheel_radius = 0.035276f,
            .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .yaw_bias = {
            .enabled = true,
            .k_vx = -0.049f,
            .k_vy = 0.055f,
            .v_deadband = 0.01f,
        }
    };

    log_info("CHASSIS assemble begin");
    if(can_register_rx_callback(&hfdcan1, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    if(can_register_rx_callback(&hfdcan2, chassis_drive_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;

    if(can_filter_init() != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS CAN filter ok");
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN1 start ok");
    if(can_start(&hfdcan2) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN2 start ok");

    delay_ms(100);

    if(chassis_init(&chassis_config) != chassis.OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("CHASSIS service init ok");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

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
