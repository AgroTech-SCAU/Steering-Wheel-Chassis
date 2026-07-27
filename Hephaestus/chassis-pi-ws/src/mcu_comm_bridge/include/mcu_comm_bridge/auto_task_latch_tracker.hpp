#pragma once

#include <cstdint>

namespace mcu_comm_bridge {

enum class AutoTaskTransition {
    None,
    Start,
    Reset,
};

struct AutoTaskUpdate {
    AutoTaskTransition transition = AutoTaskTransition::None;
    bool latched = false;
    bool start_pending = false;
    bool first_observation = false;
};

class AutoTaskLatchTracker {
public:
    explicit AutoTaskLatchTracker(uint8_t auto_pi_state = 2u);

    AutoTaskUpdate update(bool auto_start_latched, uint8_t app_state);
    void reset_local_state();

private:
    uint8_t auto_pi_state_ = 2u;
    bool initialized_ = false;
    bool last_latched_ = false;
    bool start_emitted_for_current_latch_ = false;
};

}  // namespace mcu_comm_bridge
