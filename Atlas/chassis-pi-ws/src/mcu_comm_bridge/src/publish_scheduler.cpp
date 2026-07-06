#include "mcu_comm_bridge/publish_scheduler.hpp"

namespace mcu_comm_bridge {

FixedRatePublishScheduler::FixedRatePublishScheduler(int64_t max_reuse_age_ms)
    : max_reuse_age_ms_(max_reuse_age_ms) {
}

void FixedRatePublishScheduler::set_max_reuse_age_ms(int64_t max_reuse_age_ms) {
    max_reuse_age_ms_ = max_reuse_age_ms;
}

void FixedRatePublishScheduler::reset() {
    has_valid_ = false;
    latest_receive_time_ms_ = 0;
    latest_version_ = 0;
    published_version_ = 0;
    reused_latest_version_ = false;
}

void FixedRatePublishScheduler::note_valid(uint64_t receive_time_ms) {
    has_valid_ = true;
    latest_receive_time_ms_ = receive_time_ms;
    ++latest_version_;
    reused_latest_version_ = false;
}

PublishOutcome FixedRatePublishScheduler::on_tick(uint64_t now_ms) {
    if(!has_valid_) {
        return PublishOutcome::NoValidData;
    }

    if(latest_version_ != published_version_) {
        published_version_ = latest_version_;
        reused_latest_version_ = false;
        return PublishOutcome::Fresh;
    }

    if(reused_latest_version_) {
        return PublishOutcome::StaleDrop;
    }

    if(now_ms >= latest_receive_time_ms_ &&
       static_cast<int64_t>(now_ms - latest_receive_time_ms_) <= max_reuse_age_ms_) {
        reused_latest_version_ = true;
        return PublishOutcome::Reused;
    }

    return PublishOutcome::StaleDrop;
}

}  // namespace mcu_comm_bridge
