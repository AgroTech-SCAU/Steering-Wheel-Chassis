#include "mcu_comm_bridge/mcu_status_codec.hpp"

#include "mcu_comm_bridge/binary_frame.hpp"

namespace mcu_comm_bridge {

McuStatusDecodeResult decode_mcu_status(const std::vector<uint8_t>& payload) {
    McuStatusDecodeResult result;
    if(payload.size() != PAYLOAD_MCU_STATUS_LEN) {
        result.status = McuStatusDecodeStatus::InvalidLength;
        return result;
    }

    result.decoded.stamp_ms = read_u32_le(payload, 0);
    result.decoded.app_state = payload[4];
    result.decoded.manual_mode = payload[5];
    result.decoded.ready_flags = payload[6];
    result.decoded.online_flags = payload[7];
    result.decoded.fault_source = payload[8];
    result.decoded.fault_level = payload[9];
    result.decoded.fault_code = read_i16_le(payload, 10);
    result.raw_auto_start_latched = payload[12];

    if(result.raw_auto_start_latched == 0u) {
        result.decoded.auto_start_latched = false;
    }
    else if(result.raw_auto_start_latched == 1u) {
        result.decoded.auto_start_latched = true;
    }
    else {
        result.status = McuStatusDecodeStatus::InvalidAutoStartLatchedValue;
        result.decoded.auto_start_latched = true;
    }

    return result;
}

}  // namespace mcu_comm_bridge
