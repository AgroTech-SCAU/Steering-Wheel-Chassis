#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>


// ! app ! //
#include "remote.h"


// ! service ! //
#include "assemble/assemble.h"
#include "chassis.h"
#include "odom.h"
#include "arm.h"

// ! device ! //
#include "rgb_led/rgb_led.h"


// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static uint8_t remote_tick = 0;
static uint8_t arm_tick = 0;
static uint8_t odom_tick = 0;
static uint8_t led_state = 0u;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    delay_ms(100);

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb init step done");
    delay_ms(100);

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu init step done");
    delay_ms(100);

    if(assemble_odom() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT odom init step done");
    delay_ms(100);

    if(assemble_chassis() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT chassis init step done");
    delay_ms(100);

    if(assemble_arm() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT arm init step done");
    delay_ms(100);

    if(assemble_remote() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT remote init step done");
    delay_ms(100);

    remote_init();
    delay_ms(100);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

    log_info("System initialized successfully");
    delay_ms(500);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        chassis.process();

        if(++odom_tick % 2 == 0) {
            odom.process();
            odom_tick = 0;
        }

        if(++remote_tick % 5 == 0) {
            remote_process();
            remote_tick = 0;
        }

        if(++arm_tick % 10 == 0) {
            arm.refresh_current_state();
            arm_tick = 0;
        }
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&heartbeat_task, 1000)) {
        RemoteCommand remote_command;
        const bool chassis_ready = chassis.is_ready();
        const bool remote_online = remote_get_command(&remote_command);

        uint8_t target_state = chassis_ready ? 1u : 0u;
        if(target_state == 1u && remote_online)
            target_state = 2u;

        if(led_state != target_state) {
            if(target_state == 2u)
                rgb_led.fill(0U, 0U, 255U);
            else if(target_state == 1u)
                rgb_led.fill(0U, 255U, 0U);
            else
                rgb_led.fill(255U, 0U, 0U);
            if(rgb_led.show() == RGB_LED_STATUS_OK)
                led_state = target_state;
            log_info("Chassis %s, Remote %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline");
        }
    }

    if(delay_nb_ms(&log_task, 1000)) {
        Vector3 od = { 0 };
        Vector3 ag = { 0 };

        odom.get_odom(&od);
        odom.get_angle(&ag);
        log_vofa(od.x, od.y, od.z, ag.x, ag.y, ag.z);
    }
}

#endif
