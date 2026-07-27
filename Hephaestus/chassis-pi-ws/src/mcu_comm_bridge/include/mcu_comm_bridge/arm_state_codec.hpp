#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace mcu_comm_bridge {

enum class ArmStateDecodeError {
    None = 0,
    BadLength,
    QuaternionOutOfRange,
    QuaternionNormOutOfRange,
};

struct DecodedArmState {
    uint32_t stamp_ms = 0;
    uint16_t status_flags = 0;
    uint16_t sequence_count = 0;
    std::array<double, 5> joints_rad{ { 0.0, 0.0, 0.0, 0.0, 0.0 } };
    double position_x_m = 0.0;
    double position_y_m = 0.0;
    double position_z_m = 0.0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;
    bool joint_valid = false;
    bool pose_flag_set = false;
    bool pose_valid = false;
    bool quaternion_was_normalized = false;
};

ArmStateDecodeError decode_arm_state_payload(const std::vector<uint8_t>& payload, DecodedArmState* out);

}  // namespace mcu_comm_bridge
