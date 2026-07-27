#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

#include "app_runtime.h"
#include "app_status.h"
#include "arm.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "pc_comms.h"
#include "pi_comms.h"
#include "remote.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 系统初始化完成标记
 */
static bool init_ok = false;

/**
 * @brief 250Hz 节拍计数器
 */
static uint8_t s_entry_250hz_tick = 0u;

/**
 * @brief 100Hz 节拍计数器
 */
static uint8_t s_entry_100hz_tick = 0u;

/**
 * @brief 50Hz 节拍计数器
 */
static uint8_t s_entry_50hz_tick = 0u;

/**
 * @brief 机械臂反馈刷新异常日志节流计时器
 */
static ms_t s_entry_arm_refresh_log_timer = 0u;

/**
 * @brief 机械臂反馈刷新连续失败计数
 */
static uint16_t s_entry_arm_refresh_fail_count = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 判断当前 500Hz 周期是否命中 250Hz 节拍
 * @return bool `true` 表示应执行 250Hz 任务
 */
static inline bool entry_tick_250hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 100Hz 节拍
 * @return bool `true` 表示应执行 100Hz 任务
 */
static inline bool entry_tick_100hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 50Hz 节拍
 * @return bool `true` 表示应执行 50Hz 任务
 */
static inline bool entry_tick_50hz(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 系统初始化入口
 * @details 按照启动依赖顺序完成底层装配, 应用层初始化, 最后启动 500Hz 定时调度
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    delay_ms(100u);

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb ready");
    delay_ms(100u);

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu ready");
    delay_ms(100u);

    if(assemble_odom() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT odom ready");
    delay_ms(100u);

    if(assemble_chassis() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT chassis ready");
    delay_ms(100u);

    if(assemble_arm() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT arm ready");
    delay_ms(100u);

    if(assemble_suction() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT suction ready");
    delay_ms(100u);

    if(assemble_remote() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT remote ready");
    delay_ms(100u);

    {
        remote_init();

        if(assemble_pc_comms() != SYSTEM_STATUS_OK)
            return;

        if(assemble_pi_comms() != SYSTEM_STATUS_OK)
            return;

        app_runtime_init();
        app_status_init();
    }
    log_info("BOOT app ready");
    delay_ms(100u);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz ready");
    delay_ms(100u);

    log_info("System initialized successfully");
    delay_ms(500u);

    init_ok = true;
}

/**
 * @brief 主循环调度入口
 * @details
 *
 * 500Hz base order:
 * 1. chassis.process()
 * 2. 250Hz slot: odom.process()
 * 3. 100Hz slot: remote_process() -> pc_comms_process() -> pi_comms_process()
 * 4. 50Hz slot: arm.refresh_current_state()
 * 5. app_runtime_process()
 *
 * background:
 * - app_status_process()
 */
static inline void entry_loop(void) {
    if(!init_ok) {
        return;
    }

    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        chassis.process();

        if(entry_tick_250hz()) {
            odom.process();
        }

        if(entry_tick_100hz()) {
            remote_process();
            pc_comms_process();
            pi_comms_process();
        }

        if(entry_tick_50hz()) {
            ArmStatus arm_status = arm.refresh_current_state();

            if(arm_status == ARM_OK) {
                s_entry_arm_refresh_fail_count = 0u;
            }
            else {
                s_entry_arm_refresh_fail_count++;
                if(s_entry_arm_refresh_fail_count >= 10u && delay_nb_ms(&s_entry_arm_refresh_log_timer, 1000u)) {
                    log_warn("ARM refresh unstable: %s, consecutive=%u",
                             arm.status_str(arm_status),
                             s_entry_arm_refresh_fail_count);
                }
            }
        }

        app_runtime_process();
    }

    app_status_process();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static inline bool entry_tick_250hz(void) {
    s_entry_250hz_tick = (uint8_t)((s_entry_250hz_tick + 1u) % 2u);
    return s_entry_250hz_tick == 0u;
}

static inline bool entry_tick_100hz(void) {
    s_entry_100hz_tick = (uint8_t)((s_entry_100hz_tick + 1u) % 5u);
    return s_entry_100hz_tick == 0u;
}

static inline bool entry_tick_50hz(void) {
    s_entry_50hz_tick = (uint8_t)((s_entry_50hz_tick + 1u) % 10u);
    return s_entry_50hz_tick == 0u;
}

#endif
