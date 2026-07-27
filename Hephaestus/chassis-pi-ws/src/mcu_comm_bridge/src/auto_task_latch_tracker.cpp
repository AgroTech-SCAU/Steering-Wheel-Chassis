#include "mcu_comm_bridge/auto_task_latch_tracker.hpp"

namespace mcu_comm_bridge {

AutoTaskLatchTracker::AutoTaskLatchTracker(uint8_t auto_pi_state)
    : auto_pi_state_(auto_pi_state) {}

AutoTaskUpdate AutoTaskLatchTracker::update(bool auto_start_latched, uint8_t app_state) {
    AutoTaskUpdate update;
    update.latched = auto_start_latched;

    if(!initialized_) {
        initialized_ = true;
        last_latched_ = auto_start_latched;
        update.first_observation = true;

        if(auto_start_latched) {
            if(app_state == auto_pi_state_) {
                start_emitted_for_current_latch_ = true;
                update.transition = AutoTaskTransition::Start;
            }
            else {
                update.start_pending = true;
            }
        }
        return update;
    }

    if(last_latched_ && !auto_start_latched) {
        last_latched_ = false;
        start_emitted_for_current_latch_ = false;
        update.transition = AutoTaskTransition::Reset;
        return update;
    }

    if(!last_latched_ && auto_start_latched) {
        last_latched_ = true;
    }

    if(auto_start_latched && !start_emitted_for_current_latch_) {
        if(app_state == auto_pi_state_) {
            start_emitted_for_current_latch_ = true;
            update.transition = AutoTaskTransition::Start;
        }
        else {
            update.start_pending = true;
        }
    }

    if(!auto_start_latched) {
        last_latched_ = false;
        start_emitted_for_current_latch_ = false;
    }

    return update;
}

void AutoTaskLatchTracker::reset_local_state() {
    initialized_ = false;
    last_latched_ = false;
    start_emitted_for_current_latch_ = false;
}

}  // namespace mcu_comm_bridge
