#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "binary_frame.h"
#include "../src/app/app_runtime.c"

static AppFsmStateId state = APP_FSM_STATE_AUTO_PI;
static bool ready = true;
static uint8_t result_payload[7];
static unsigned result_count;
static uint32_t now_ms(void) { return 1000u; }
static bool write_frame(const char* bytes, uint32_t size) {
    const uint8_t* data = (const uint8_t*)bytes;
    if(size >= 10u && data[5] == 0x28u) {
        assert(size == 17u);
        memcpy(result_payload, data + 8, 7);
        result_count++;
    }
    return true;
}
static bool is_ready(void) { return ready; }
static bool always_ready(void) { return true; }
const struct ArmInterface arm_interface = { .is_ready = is_ready };
const struct ChassisInterface chassis_interface = { .is_ready = always_ready };
const struct OdomInterface odom_interface = { .is_ready = always_ready };
AppFsmStateId app_fsm_get_state(void) { return state; }
AppManualMode app_fsm_get_manual_mode(void) { return APP_MANUAL_MODE_CHASSIS_PC_ARM; }
const AppFault* app_fsm_get_fault(void) { return NULL; }
bool app_fsm_raise_fault(const AppFault* fault) { (void)fault; state = APP_FSM_STATE_FAULT; return true; }
bool app_fsm_post(AppFsmEventId event) { (void)event; return true; }
void app_fsm_process(void) {}
bool app_fsm_has_fault(void) { return false; }
void pc_comms_clear_master_joints(void) {}
AppControlResult app_control_stop_all(void) { return APP_CONTROL_RESULT_OK; }
AppControlResult app_control_stop_arm(void) { return APP_CONTROL_RESULT_OK; }
AppControlResult app_control_brake_chassis(void) { return APP_CONTROL_RESULT_OK; }
void remote_clear_pending_auto_start_event(void) {}
void asr_comms_clear_pending_auto_start_event(void) {}
void chassis_yaw_hold_reset(void) {}

static void receive(uint16_t seq) {
    uint8_t payload[38] = {0}, bytes[64];
    uint16_t len;
    binary_frame_write_u32_le(payload, 1000u);
    payload[4] = BINARY_FRAME_PI_CONTROL_MASK_ARM_VALID;
    payload[5] = BINARY_FRAME_PI_ARM_MODE_POSE_5D;
    binary_frame_write_u16_le(payload + 6, seq);
    assert(binary_frame_pack(BINARY_FRAME_MSG_PI_CONTROL, 1, 0, payload, sizeof(payload), bytes, sizeof(bytes), &len));
    for(unsigned i = 0; i < len; ++i) { pi_comms_on_rx_byte(bytes[i]); }
    pi_comms_process();
}
static void expect(uint16_t seq, int32_t status) {
    assert(result_count == 1u && "runtime gate must return result before discarding command");
    assert(binary_frame_read_u16_le(result_payload) == seq);
    assert(result_payload[2] == 6u);
    assert(binary_frame_read_i32_le(result_payload + 3) == status);
    assert(!pi_comms_has_pending_arm_control());
}
int main(void) {
    PiCommsConfig config = { .port_ops = { .write = write_frame, .now_ms = now_ms } };
    delay_ms_init(now_ms);
    assert(pi_comms_init(&config) == PI_COMMS_STATUS_OK);
    s_auto_start_latched = true;
    ready = false;
    receive(101);
    assert(!app_runtime_apply_safety());
    expect(101, ARM_NOT_INITIALIZED);
    ready = true;
    const AppFsmStateId states[] = {APP_FSM_STATE_IDLE, APP_FSM_STATE_MANUAL, APP_FSM_STATE_FAULT, APP_FSM_STATE_ESTOP, APP_FSM_STATE_FINISHED};
    for(unsigned i = 0; i < sizeof(states) / sizeof(states[0]); ++i) {
        state = states[i];
        result_count = 0;
        receive((uint16_t)(110u + i));
        (void)app_runtime_apply_safety();
        expect((uint16_t)(110u + i), -1);
    }
    puts("PASS arm command runtime gates: not-ready and all non-AutoPi modes");
}
