#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;

extern volatile bool tim6_500hz_flag;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SystemStatus assemble_delay(void);
SystemStatus assemble_log(void);
SystemStatus assemble_rgb(void);
SystemStatus assemble_imu(void);
SystemStatus assemble_chassis(void);
SystemStatus assemble_suction(void);
SystemStatus assemble_odom(void);
SystemStatus assemble_remote(void);
SystemStatus assemble_tim6_500hz(void);
SystemStatus assemble_arm(void);
SystemStatus assemble_pc_comms(void);
SystemStatus assemble_pi_comms(void);

#endif
