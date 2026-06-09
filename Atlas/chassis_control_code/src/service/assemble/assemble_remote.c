#include "assemble.h"

#include "fs_ia10b.h"
#include "log.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_remote(void) {
    log_info("REMOTE init begin");
    ibus_init();
    log_info("REMOTE init done");
    return SYSTEM_STATUS_OK;
}
