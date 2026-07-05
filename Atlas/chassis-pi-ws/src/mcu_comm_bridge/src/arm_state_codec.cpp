#include "mcu_comm_bridge/arm_state_codec.hpp"

#include "mcu_comm_bridge/binary_frame.hpp"

#include <cmath>

namespace mcu_comm_bridge {

ArmStateDecodeError decode_arm_state_payload(const std::vector<uint8_t>& payload, DecodedArmState* out) {
    if(out == nullptr) {
        return ArmStateDecodeError::BadLength;
    }

    *out = DecodedArmState{};
    if(payload.size() != PAYLOAD_MCU_ARM_STATE_LEN) {
        return ArmStateDecodeError::BadLength;
    }

    out->stamp_ms = read_u32_le(payload, 0);
    out->status_flags = read_u16_le(payload, 4);
    out->sequence_count = read_u16_le(payload, 6);
    out->joints_rad[0] = urad_to_rad(read_i32_le(payload, 8));
    out->joints_rad[1] = urad_to_rad(read_i32_le(payload, 12));
    out->joints_rad[2] = urad_to_rad(read_i32_le(payload, 16));
    out->joints_rad[3] = urad_to_rad(read_i32_le(payload, 20));
    out->joints_rad[4] = urad_to_rad(read_i32_le(payload, 24));
    out->position_x_m = mm_to_m(read_i32_le(payload, 28));
    out->position_y_m = mm_to_m(read_i32_le(payload, 32));
    out->position_z_m = mm_to_m(read_i32_le(payload, 36));
    out->orientation_x = static_cast<double>(read_i16_le(payload, 40)) / 32767.0;
    out->orientation_y = static_cast<double>(read_i16_le(payload, 42)) / 32767.0;
    out->orientation_z = static_cast<double>(read_i16_le(payload, 44)) / 32767.0;
    out->orientation_w = static_cast<double>(read_i16_le(payload, 46)) / 32767.0;
    out->joint_valid = (out->status_flags & ARM_STATE_FLAG_JOINT_VALID) != 0u;
    out->pose_flag_set = (out->status_flags & ARM_STATE_FLAG_POSE_VALID) != 0u;

    if(!out->pose_flag_set) {
        return ArmStateDecodeError::None;
    }

    constexpr double kMaxAbsComponent = 1.1;
    if(!std::isfinite(out->orientation_x) ||
       !std::isfinite(out->orientation_y) ||
       !std::isfinite(out->orientation_z) ||
       !std::isfinite(out->orientation_w) ||
       std::abs(out->orientation_x) > kMaxAbsComponent ||
       std::abs(out->orientation_y) > kMaxAbsComponent ||
       std::abs(out->orientation_z) > kMaxAbsComponent ||
       std::abs(out->orientation_w) > kMaxAbsComponent) {
        return ArmStateDecodeError::QuaternionOutOfRange;
    }

    const double norm = std::sqrt(out->orientation_x * out->orientation_x +
                                  out->orientation_y * out->orientation_y +
                                  out->orientation_z * out->orientation_z +
                                  out->orientation_w * out->orientation_w);
    if(!std::isfinite(norm) || norm < 0.5 || norm > 1.5) {
        return ArmStateDecodeError::QuaternionNormOutOfRange;
    }

    out->orientation_x /= norm;
    out->orientation_y /= norm;
    out->orientation_z /= norm;
    out->orientation_w /= norm;
    out->pose_valid = true;
    out->quaternion_was_normalized = std::abs(norm - 1.0) > 1e-9;
    return ArmStateDecodeError::None;
}

}  // namespace mcu_comm_bridge
