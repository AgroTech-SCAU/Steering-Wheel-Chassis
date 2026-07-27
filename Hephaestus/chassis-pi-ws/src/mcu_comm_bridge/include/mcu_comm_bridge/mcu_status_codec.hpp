#pragma once

#include <cstdint>
#include <vector>

namespace mcu_comm_bridge {

struct DecodedMcuStatus {
    uint32_t stamp_ms = 0;
    uint8_t app_state = 0;
    uint8_t manual_mode = 0;
    uint8_t ready_flags = 0;
    uint8_t online_flags = 0;
    uint8_t fault_source = 0;
    uint8_t fault_level = 0;
    int16_t fault_code = 0;
    bool auto_start_latched = false;
};

enum class McuStatusDecodeStatus {
    Ok,
    InvalidLength,
    InvalidAutoStartLatchedValue,
};

struct McuStatusDecodeResult {
    McuStatusDecodeStatus status = McuStatusDecodeStatus::Ok;
    DecodedMcuStatus decoded{};
    uint8_t raw_auto_start_latched = 0;
};

McuStatusDecodeResult decode_mcu_status(const std::vector<uint8_t>& payload);

}  // namespace mcu_comm_bridge
