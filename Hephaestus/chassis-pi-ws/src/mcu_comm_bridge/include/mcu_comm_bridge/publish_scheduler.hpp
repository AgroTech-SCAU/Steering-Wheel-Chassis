#pragma once

#include <cstdint>

namespace mcu_comm_bridge {

enum class PublishOutcome {
    Fresh,
    Reused,
    StaleDrop,
    NoValidData,
};

class FixedRatePublishScheduler {
public:
    explicit FixedRatePublishScheduler(int64_t max_reuse_age_ms = 0);

    void set_max_reuse_age_ms(int64_t max_reuse_age_ms);
    void reset();
    void note_valid(uint64_t receive_time_ms);
    PublishOutcome on_tick(uint64_t now_ms);

private:
    int64_t max_reuse_age_ms_;
    bool has_valid_ = false;
    uint64_t latest_receive_time_ms_ = 0;
    uint64_t latest_version_ = 0;
    uint64_t published_version_ = 0;
    bool reused_latest_version_ = false;
};

}  // namespace mcu_comm_bridge
