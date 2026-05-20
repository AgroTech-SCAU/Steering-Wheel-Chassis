#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"
#include "remote.h"

// ! device ! //
#include "imu/imu.h"
#include "rgb_led/rgb_led.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };
static uint8_t remote = 0;
static uint8_t led_state = 0u;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 *
 * 该函数由 main 初始化完成后调用；
 * 负责装配各服务并清零底盘速度命令
 */
static inline void entry_init(void) {
    assemble_init();

    log_info("System initialized successfully");
    delay_ms(500);
}

/**
 * @brief 程序主循环入口函数
 *
 * 该函数在 while(1) 中持续调用；
 * 根据定时器事件执行底盘、遥控、IMU 和日志任务
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        chassis.process();

        if(remote++ % 5 == 0) {
            remote_process();
            remote = 0;

            if(imu.update() == IMU_STATUS_OK) {
                accel = imu.get_acc();
                gyro = imu.get_gyro();
                angle = imu.get_angle();
            }
        }
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&heartbeat_task, 1000)) {
        RemoteCommand remote_command;
        const bool chassis_ready = chassis.is_ready();
        const bool remote_online = remote_get_command(&remote_command);

        uint8_t target_state = chassis_ready ? 1u : 0u;
        if(target_state == 1u && remote_online) target_state = 2u;

        if(led_state != target_state) {
            if(target_state == 2u) rgb_led.fill(0U, 0U, 255U);
            else if(target_state == 1u) rgb_led.fill(0U, 255U, 0U);
            else rgb_led.fill(255U, 0U, 0U);
            if(rgb_led.show() == RGB_LED_STATUS_OK) led_state = target_state;
            log_info("Chassis %s, Remote %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline");
        }
    }

    if(delay_nb_ms(&log_task, 2000)) {
        log_info("Heartbeat");
    }
}

#endif
