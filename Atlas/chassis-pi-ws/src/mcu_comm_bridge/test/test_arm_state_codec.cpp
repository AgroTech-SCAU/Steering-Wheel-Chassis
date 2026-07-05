#include "mcu_comm_bridge/arm_state_codec.hpp"
#include "mcu_comm_bridge/binary_frame.hpp"

#include <gtest/gtest.h>

namespace mcu_comm_bridge {
namespace {

std::vector<uint8_t> make_arm_state_payload(int32_t quat_w_e6 = 1000000,
                                            uint16_t status_flags = ARM_STATE_FLAG_JOINT_VALID | ARM_STATE_FLAG_POSE_VALID) {
    std::vector<uint8_t> payload(PAYLOAD_MCU_ARM_STATE_LEN, 0u);
    write_u32_le(payload, 0, 1234u);
    write_u16_le(payload, 4, status_flags);
    write_u16_le(payload, 6, 7u);
    write_i32_le(payload, 8, 3140000);
    write_i32_le(payload, 12, 1570000);
    write_i32_le(payload, 16, 6280000);
    write_i32_le(payload, 20, 3140000);
    write_i32_le(payload, 24, 3140000);
    write_i32_le(payload, 28, 100);
    write_i32_le(payload, 32, -200);
    write_i32_le(payload, 36, 300);
    write_i32_le(payload, 40, 0);
    write_i32_le(payload, 44, 0);
    write_i32_le(payload, 48, 0);
    write_i32_le(payload, 52, quat_w_e6);
    return payload;
}

TEST(ArmStateCodecTest, DecodesKnownPayload) {
    DecodedArmState state;
    const auto result = decode_arm_state_payload(make_arm_state_payload(), &state);

    ASSERT_EQ(result, ArmStateDecodeError::None);
    EXPECT_TRUE(state.joint_valid);
    EXPECT_TRUE(state.pose_valid);
    EXPECT_NEAR(state.joints_rad[0], 3.14, 1e-6);
    EXPECT_NEAR(state.joints_rad[1], 1.57, 1e-6);
    EXPECT_NEAR(state.joints_rad[2], 6.28, 1e-6);
    EXPECT_NEAR(state.position_x_m, 0.1, 1e-9);
    EXPECT_NEAR(state.position_y_m, -0.2, 1e-9);
    EXPECT_NEAR(state.position_z_m, 0.3, 1e-9);
    EXPECT_DOUBLE_EQ(state.orientation_x, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation_y, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation_z, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation_w, 1.0);
}

TEST(ArmStateCodecTest, NormalizesNearUnitQuaternion) {
    DecodedArmState state;
    const auto result = decode_arm_state_payload(make_arm_state_payload(999999), &state);

    ASSERT_EQ(result, ArmStateDecodeError::None);
    EXPECT_TRUE(state.pose_valid);
    EXPECT_TRUE(state.quaternion_was_normalized);
    EXPECT_NEAR(state.orientation_w, 1.0, 1e-9);
}

TEST(ArmStateCodecTest, RejectsZeroQuaternionButKeepsJointDecode) {
    DecodedArmState state;
    const auto result = decode_arm_state_payload(make_arm_state_payload(0), &state);

    ASSERT_EQ(result, ArmStateDecodeError::QuaternionNormOutOfRange);
    EXPECT_TRUE(state.joint_valid);
    EXPECT_TRUE(state.pose_flag_set);
    EXPECT_FALSE(state.pose_valid);
}

TEST(ArmStateCodecTest, ParserRejectsLegacyArmStatePayloadLength) {
    std::vector<uint8_t> legacy_payload(40u, 0u);
    const auto frame = pack_frame(MSG_MCU_ARM_STATE, 1u, 0u, legacy_payload);
    BinaryFrameParser parser;

    const auto frames = parser.feed(frame.data(), frame.size());
    const auto errors = parser.take_error_events();

    EXPECT_TRUE(frames.empty());
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors.front().kind, ParserErrorKind::KnownMessageBadLength);
    ASSERT_TRUE(errors.front().msg_id.has_value());
    EXPECT_EQ(errors.front().msg_id.value(), MSG_MCU_ARM_STATE);
}

}  // namespace
}  // namespace mcu_comm_bridge
