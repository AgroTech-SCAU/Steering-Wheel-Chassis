#include "asrpro.h"

#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 语音播报函数
 * @param cmd 语音播报指令
 * @return 是否成功发送播报指令
 */
bool asrpro_broadcast(AsrProCmd cmd) {
    // 边界检查，防止传入非法指令
    if(cmd >= MAX || cmd < TEAM_INTRO) {
        return false;
    }

    // 所有的指令格式为 AA 55 02 01 [ID] [ID+2] FF
    // 直接利用枚举值 cmd 填入第5字节，cmd + 2 填入第6字节即可
    char data[] = { 0xAA, 0x55, 0x02, 0x01, (char)cmd, (char)(cmd + 2), 0xFF };

    uart10_write_blocking(data, sizeof(data));

    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //