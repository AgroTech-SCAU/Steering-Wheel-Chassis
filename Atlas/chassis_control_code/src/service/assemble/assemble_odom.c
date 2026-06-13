#include "assemble.h"

#include "log.h"
#include "odom.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_odom(void) {
    OdomConfig config = odom.default_config();

    if(odom.init(&config) != odom.OK) {
        log_error("ODOM service init failed");
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ODOM service init ok");
    return SYSTEM_STATUS_OK;
}
