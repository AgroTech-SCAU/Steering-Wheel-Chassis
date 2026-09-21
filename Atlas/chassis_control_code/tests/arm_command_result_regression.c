#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "binary_frame.h"
#include "../src/app/app_control.c"

static uint32_t clock_ms = 1000u;
static unsigned moves;
static bool ready = true;
static ArmStatus move_status = ARM_OK;
static uint8_t result_payload[7];
static unsigned result_count;
static bool transport_ok = true;
static uint32_t now_ms(void) { return clock_ms; }
static bool write_frame(const char* bytes, uint32_t size) {
    const uint8_t* data = (const uint8_t*)bytes;
    if(!transport_ok) { return false; }
    if(size >= 10u && data[5] == 0x28u) {
        assert(size == 17u);
        memcpy(result_payload, data + 8, 7);
        result_count++;
    }
    return true;
}
static bool is_ready(void) { return ready; }
static const Arm* get_arm(void) { return NULL; }
static const char* status_str(ArmStatus status) { (void)status; return "test status"; }
static ArmStatus move_joints(const FiveDofArmJointArray* joints, float speed) {
    (void)joints; (void)speed; moves++; return move_status;
}
static ArmStatus move_pose(float x, float y, float z, float pitch, float yaw, float speed) {
    (void)x; (void)y; (void)z; (void)pitch; (void)yaw; (void)speed;
    moves++; return move_status;
}
static ArmStatus move_position(float x, float y, float z, float speed) {
    return move_pose(x, y, z, 0, 0, speed);
}
static ArmStatus move_orientation(float pitch, float yaw, float speed) {
    return move_pose(0, 0, 0, pitch, yaw, speed);
}
static ArmStatus stop(void) { return ARM_OK; }
const struct ArmInterface arm_interface = {
    .is_ready = is_ready, .get_arm = get_arm, .status_str = status_str,
    .move_joints = move_joints, .move_pose_5d = move_pose,
    .move_position = move_position, .move_orientation_2d = move_orientation,
    .stop = stop, .enable = stop
};
SuctionResult suction_set(bool enable) { (void)enable; return SUCTION_RESULT_OK; }

static void receive(uint16_t seq, uint8_t mode, int32_t pitch) {
    uint8_t payload[38] = {0};
    uint8_t bytes[64];
    uint16_t len;
    binary_frame_write_u32_le(payload, clock_ms);
    payload[4] = BINARY_FRAME_PI_CONTROL_MASK_ARM_VALID;
    payload[5] = mode;
    binary_frame_write_u16_le(payload + 6, seq);
    binary_frame_write_i32_le(payload + 26, pitch);
    binary_frame_write_u16_le(payload + 34, 1000);
    assert(binary_frame_pack(BINARY_FRAME_MSG_PI_CONTROL, 1, 0, payload, sizeof(payload), bytes, sizeof(bytes), &len));
    for(unsigned i = 0; i < len; ++i) { pi_comms_on_rx_byte(bytes[i]); }
    pi_comms_process();
}
static void expect_result(unsigned before, uint16_t seq, uint8_t result, int32_t status) {
    if(result_count != before + 1u) {
        fprintf(stderr, "FAIL: seq %u expected asynchronous result, count before=%u after=%u\n", seq, before, result_count);
        assert(result_count == before + 1u);
    }
    assert(binary_frame_read_u16_le(result_payload) == seq);
    assert(result_payload[2] == result);
    assert(binary_frame_read_i32_le(result_payload + 3) == status);
}
int main(void) {
    PiCommsConfig config = { .port_ops = { .write = write_frame, .now_ms = now_ms } };
    delay_ms_init(now_ms);
    assert(pi_comms_init(&config) == PI_COMMS_STATUS_OK);
    unsigned before = result_count;
    receive(41, 99, 0);
    expect_result(before, 41, 2, ARM_INVALID_PARAM);
    assert(!pi_comms_has_pending_arm_control());

    ArmStatus statuses[] = {ARM_OK, ARM_NO_SOLUTION, ARM_INVALID_PARAM, ARM_KINEMATICS_FAILED, ARM_SERVO_FAILED, ARM_OUT_OF_LIMIT, ARM_NOT_INITIALIZED};
    uint8_t results[] = {0, 1, 2, 3, 4, 2, 6};
    for(unsigned i = 0; i < sizeof(results); ++i) {
        move_status = statuses[i];
        before = result_count;
        receive((uint16_t)(50+i), BINARY_FRAME_PI_ARM_MODE_POSE_5D, -1570796);
        app_control_apply_pi_arm();
        expect_result(before, (uint16_t)(50+i), results[i], statuses[i]);
        unsigned executed = moves;
        before = result_count;
        receive((uint16_t)(50+i), BINARY_FRAME_PI_ARM_MODE_POSE_5D, -1570796);
        app_control_apply_pi_arm();
        expect_result(before, (uint16_t)(50+i), results[i], statuses[i]);
        assert(moves == executed);
    }
    /* An older duplicate must replay its original result after a newer command */
    before = result_count;
    unsigned executed = moves;
    receive(50, BINARY_FRAME_PI_ARM_MODE_POSE_5D, -1570796);
    app_control_apply_pi_arm();
    expect_result(before, 50, 0, ARM_OK);
    assert(moves == executed);

    before = result_count;
    receive(70, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 2000000);
    app_control_apply_pi_arm();
    expect_result(before, 70, 2, ARM_INVALID_PARAM);
    assert(moves == executed);
    ready = false;
    before = result_count;
    receive(71, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    app_control_apply_pi_arm();
    expect_result(before, 71, 6, ARM_NOT_INITIALIZED);
    ready = true;

    before = result_count;
    receive(72, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    clock_ms += 201;
    app_control_apply_pi_arm();
    expect_result(before, 72, 5, -1);
    assert(moves == executed);

    /* Unconsumed commands and cleared controls cannot silently disappear */
    before = result_count;
    receive(73, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    receive(74, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    expect_result(before, 73, 6, -1);
    before = result_count;
    pi_comms_clear_controls();
    expect_result(before, 74, 6, -1);
    before = result_count;
    receive(74, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    app_control_apply_pi_arm();
    expect_result(before, 74, 6, -1);
    assert(moves == executed);

    /* A duplicate still in the queue must not replace the original payload */
    before = result_count;
    receive(78, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    receive(78, 99, 0);
    assert(result_count == before);
    move_status = ARM_OK;
    app_control_apply_pi_arm();
    expect_result(before, 78, 0, ARM_OK);

    /* Failed UART sends retry without needing another downlink command */
    transport_ok = false;
    move_status = ARM_SERVO_FAILED;
    receive(79, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    app_control_apply_pi_arm();
    transport_ok = true;
    before = result_count;
    pi_comms_process();
    expect_result(before, 79, 4, ARM_SERVO_FAILED);

    /* Failed UART sends remain replayable */
    transport_ok = false;
    move_status = ARM_NO_SOLUTION;
    receive(80, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    app_control_apply_pi_arm();
    executed = moves;
    transport_ok = true;
    before = result_count;
    receive(80, BINARY_FRAME_PI_ARM_MODE_POSE_5D, 0);
    expect_result(before, 80, 1, ARM_NO_SOLUTION);
    app_control_apply_pi_arm();
    assert(moves == executed);
    puts("PASS arm command results: mapping, immediate failures, timeout, cancellation, duplicate replay");
}
