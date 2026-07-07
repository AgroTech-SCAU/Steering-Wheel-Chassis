#include "mcu_comm_bridge/arm_state_codec.hpp"
#include "mcu_comm_bridge/auto_task_latch_tracker.hpp"
#include "mcu_comm_bridge/binary_frame.hpp"
#include "mcu_comm_bridge/mcu_status_codec.hpp"
#include "mcu_comm_bridge/publish_scheduler.hpp"
#include "mcu_comm_bridge/serial_port.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cinttypes>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mcu_comm_bridge/msg/auto_task_event.hpp"
#include "mcu_comm_bridge/msg/mcu_status.hpp"
#include "mcu_comm_bridge/srv/estop.hpp"
#include "mcu_comm_bridge/srv/report_mission_result.hpp"
#include "mcu_comm_bridge/srv/set_arm_joints.hpp"
#include "mcu_comm_bridge/srv/set_arm_orientation.hpp"
#include "mcu_comm_bridge/srv/set_arm_pose.hpp"
#include "mcu_comm_bridge/srv/set_arm_position.hpp"
#include "mcu_comm_bridge/srv/set_yaw_target.hpp"

namespace mcu_comm_bridge {
namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr uint8_t PI_CONTROL_MASK_CHASSIS_VALID = 1u << 0;
constexpr uint8_t PI_CONTROL_MASK_ARM_VALID = 1u << 1;
constexpr uint8_t PI_CONTROL_MASK_BRAKE_REQUEST = 1u << 3;
constexpr uint8_t PI_ARM_MODE_NONE = 0u;
constexpr uint8_t PI_ARM_MODE_JOINTS = 1u;
constexpr uint8_t PI_ARM_MODE_POSE_5D = 2u;
constexpr uint8_t PI_ARM_MODE_POSITION = 3u;
constexpr uint8_t PI_ARM_MODE_ORIENTATION_2D = 4u;
constexpr uint8_t PI_YAW_ACTION_HOLD_ENABLE = 1u;
constexpr uint8_t PI_YAW_ACTION_HOLD_DISABLE = 2u;
constexpr uint8_t PI_YAW_ACTION_TARGET_SET = 3u;

enum class ReusedMessageStampMode {
    PreserveSource,
    PublishNow,
};

struct ImuSample {
    uint32_t stamp_ms = 0;
    uint16_t status_flags = 0;
    uint16_t sequence_count = 0;
    int32_t acc_x_mm_s2 = 0;
    int32_t acc_y_mm_s2 = 0;
    int32_t acc_z_mm_s2 = 0;
    int32_t gyro_x_urad_s = 0;
    int32_t gyro_y_urad_s = 0;
    int32_t gyro_z_urad_s = 0;
    int32_t roll_urad = 0;
    int32_t pitch_urad = 0;
    int32_t yaw_urad = 0;
};

struct OdomState {
    uint32_t stamp_ms = 0;
    uint16_t status_flags = 0;
    uint16_t reset_counter = 0;
    int32_t x_mm = 0;
    int32_t y_mm = 0;
    int32_t yaw_urad = 0;
    int32_t vx_mm_s = 0;
    int32_t vy_mm_s = 0;
    int32_t wz_urad_s = 0;
};

struct ArmState {
    uint32_t stamp_ms = 0;
    uint16_t status_flags = 0;
    uint16_t sequence_count = 0;
    int32_t q0_urad = 0;
    int32_t q1_urad = 0;
    int32_t q2_urad = 0;
    int32_t q3_urad = 0;
    int32_t q4_urad = 0;
    int32_t x_mm = 0;
    int32_t y_mm = 0;
    int32_t z_mm = 0;
};

struct CmdVelCache {
    geometry_msgs::msg::Twist twist;
    SteadyClock::time_point last_update{};
    bool has_cmd = false;
    bool timeout_brake_sent = false;
};

struct ArmCommandCache {
    uint8_t arm_mode = PI_ARM_MODE_NONE;
    std::array<int32_t, 5> arm_target{ 0, 0, 0, 0, 0 };
    uint16_t arm_speed_mrad_s = 0u;
    uint16_t command_seq = 0u;
    int repeats_remaining = 0;
    bool has_command = false;
};

struct AutoTaskContext {
    bool auto_start_triggered = false;
    bool auto_start_consumed = false;
    bool mission_active = false;
    bool mission_done = false;
    bool mission_failed = false;
    bool pending_mission_result = false;
    bool pending_auto_control_resend = false;
};

template <typename T>
struct LatestValidCache {
    T sample{};
    rclcpp::Time source_stamp{ 0, 0, RCL_ROS_TIME };
    rclcpp::Time published_stamp{ 0, 0, RCL_ROS_TIME };
    SteadyClock::time_point receive_tp{};
    bool has_value = false;
    bool has_published = false;
};

struct BridgeStats {
    std::atomic<uint64_t> read_call_count{ 0 };
    std::atomic<uint64_t> read_zero_count{ 0 };
    std::atomic<uint64_t> read_error_count{ 0 };
    std::atomic<uint64_t> read_bytes_count{ 0 };
    std::atomic<uint64_t> max_read_batch_size{ 0 };

    std::atomic<uint64_t> queue_drop_count{ 0 };
    std::atomic<uint64_t> queue_peak_depth{ 0 };
    std::atomic<uint64_t> termios_mismatch_count{ 0 };

    std::atomic<uint64_t> imu_rx_valid{ 0 };
    std::atomic<uint64_t> odom_rx_valid{ 0 };
    std::atomic<uint64_t> arm_state_valid{ 0 };
    std::atomic<uint64_t> arm_joint_valid{ 0 };
    std::atomic<uint64_t> arm_pose_valid{ 0 };
    std::atomic<uint64_t> arm_pose_invalid_quaternion{ 0 };
    std::atomic<uint64_t> arm_state_bad_length{ 0 };
    std::atomic<uint64_t> status_rx_valid{ 0 };
    std::atomic<uint64_t> status_auto_start_invalid{ 0 };
    std::atomic<uint64_t> imu_rx_crc_rejected{ 0 };
    std::atomic<uint64_t> odom_rx_crc_rejected{ 0 };
    std::atomic<uint64_t> arm_rx_crc_rejected{ 0 };
    std::atomic<uint64_t> parser_crc_error_unknown_msg{ 0 };

    std::atomic<uint64_t> imu_publish_total{ 0 };
    std::atomic<uint64_t> imu_publish_fresh{ 0 };
    std::atomic<uint64_t> imu_publish_reused{ 0 };
    std::atomic<uint64_t> imu_publish_stale_drop{ 0 };
    std::atomic<uint64_t> imu_publish_no_valid_data{ 0 };

    std::atomic<uint64_t> odom_publish_total{ 0 };
    std::atomic<uint64_t> odom_publish_fresh{ 0 };
    std::atomic<uint64_t> odom_publish_reused{ 0 };
    std::atomic<uint64_t> odom_publish_stale_drop{ 0 };
    std::atomic<uint64_t> odom_publish_no_valid_data{ 0 };

    std::atomic<uint64_t> tx_heartbeat{ 0 };
    std::atomic<uint64_t> tx_pi_ack{ 0 };
    std::atomic<uint64_t> tx_control{ 0 };
    std::atomic<uint64_t> tx_arm_command{ 0 };
    std::atomic<uint64_t> tx_arm_command_retry{ 0 };
    std::atomic<uint64_t> tx_yaw_action{ 0 };
    std::atomic<uint64_t> tx_mission_event{ 0 };
    std::atomic<uint64_t> tx_estop{ 0 };
    std::atomic<uint64_t> arm_service_accepted{ 0 };
    std::atomic<uint64_t> arm_service_rejected{ 0 };
    std::atomic<uint64_t> mission_result_accepted{ 0 };
    std::atomic<uint64_t> mission_result_rejected{ 0 };
    std::atomic<uint64_t> tx_fail{ 0 };
};

struct BridgeStatsSnapshot {
    uint64_t read_call_count = 0;
    uint64_t read_zero_count = 0;
    uint64_t read_error_count = 0;
    uint64_t read_bytes_count = 0;
    uint64_t max_read_batch_size = 0;
    uint64_t queue_drop_count = 0;
    uint64_t queue_peak_depth = 0;
    uint64_t queue_depth = 0;
    uint64_t termios_mismatch_count = 0;
    uint64_t imu_rx_valid = 0;
    uint64_t odom_rx_valid = 0;
    uint64_t arm_state_valid = 0;
    uint64_t arm_joint_valid = 0;
    uint64_t arm_pose_valid = 0;
    uint64_t arm_pose_invalid_quaternion = 0;
    uint64_t arm_state_bad_length = 0;
    uint64_t status_rx_valid = 0;
    uint64_t status_auto_start_invalid = 0;
    uint64_t imu_rx_crc_rejected = 0;
    uint64_t odom_rx_crc_rejected = 0;
    uint64_t arm_rx_crc_rejected = 0;
    uint64_t parser_crc_error_unknown_msg = 0;
    uint64_t imu_publish_total = 0;
    uint64_t imu_publish_fresh = 0;
    uint64_t imu_publish_reused = 0;
    uint64_t imu_publish_stale_drop = 0;
    uint64_t imu_publish_no_valid_data = 0;
    uint64_t odom_publish_total = 0;
    uint64_t odom_publish_fresh = 0;
    uint64_t odom_publish_reused = 0;
    uint64_t odom_publish_stale_drop = 0;
    uint64_t odom_publish_no_valid_data = 0;
    uint64_t tx_heartbeat = 0;
    uint64_t tx_pi_ack = 0;
    uint64_t tx_control = 0;
    uint64_t tx_arm_command = 0;
    uint64_t tx_arm_command_retry = 0;
    uint64_t tx_yaw_action = 0;
    uint64_t tx_mission_event = 0;
    uint64_t tx_estop = 0;
    uint64_t arm_service_accepted = 0;
    uint64_t arm_service_rejected = 0;
    uint64_t mission_result_accepted = 0;
    uint64_t mission_result_rejected = 0;
    uint64_t tx_fail = 0;
    ParserStats parser{};
};

geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

int16_t clamp_to_i16(double value) {
    const double min_v = static_cast<double>(std::numeric_limits<int16_t>::min());
    const double max_v = static_cast<double>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(std::llround(std::clamp(value, min_v, max_v)));
}

int32_t clamp_to_i32(double value) {
    const double min_v = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double max_v = static_cast<double>(std::numeric_limits<int32_t>::max());
    return static_cast<int32_t>(std::llround(std::clamp(value, min_v, max_v)));
}

int16_t m_s_to_mm_s_i16(double value) {
    return clamp_to_i16(value * 1000.0);
}

int16_t rad_s_to_mrad_s_i16(double value) {
    return clamp_to_i16(value * 1000.0);
}

int32_t rad_to_urad_i32(double value) {
    return clamp_to_i32(value * 1000000.0);
}

bool finite_non_negative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool try_m_to_mm_i32(double value, int32_t* out) {
    const double scaled = value * 1000.0;
    if(!std::isfinite(value) || !std::isfinite(scaled)) {
        return false;
    }
    if(scaled < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
       scaled > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    if(out != nullptr) {
        *out = static_cast<int32_t>(std::llround(scaled));
    }
    return true;
}

bool try_rad_to_urad_i32(double value, int32_t* out) {
    const double scaled = value * 1000000.0;
    if(!std::isfinite(value) || !std::isfinite(scaled)) {
        return false;
    }
    if(scaled < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
       scaled > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    if(out != nullptr) {
        *out = static_cast<int32_t>(std::llround(scaled));
    }
    return true;
}

bool try_rad_s_to_mrad_s_u16(double value, uint16_t* out) {
    const double scaled = value * 1000.0;
    if(!std::isfinite(value) || !std::isfinite(scaled) || scaled < 0.0 ||
       scaled > static_cast<double>(std::numeric_limits<uint16_t>::max())) {
        return false;
    }
    if(out != nullptr) {
        *out = static_cast<uint16_t>(std::llround(scaled));
    }
    return true;
}

double safe_rate(uint64_t delta, double elapsed_sec) {
    return elapsed_sec > 0.0 ? static_cast<double>(delta) / elapsed_sec : 0.0;
}

uint64_t steady_now_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(SteadyClock::now().time_since_epoch()).count());
}

rclcpp::Time strictly_increasing_stamp(
    const rclcpp::Time& candidate,
    const rclcpp::Time& last_published,
    bool has_published) {
    if(!has_published || candidate.nanoseconds() > last_published.nanoseconds()) {
        return candidate;
    }
    return rclcpp::Time(last_published.nanoseconds() + 1, candidate.get_clock_type());
}

const char* app_state_name(uint8_t app_state) {
    switch(app_state) {
        case ::mcu_comm_bridge::msg::McuStatus::STATE_IDLE:
            return "Idle";
        case ::mcu_comm_bridge::msg::McuStatus::STATE_MANUAL:
            return "Manual";
        case ::mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI:
            return "AutoPi";
        case ::mcu_comm_bridge::msg::McuStatus::STATE_FAULT:
            return "Fault";
        case ::mcu_comm_bridge::msg::McuStatus::STATE_ESTOP:
            return "Estop";
        case ::mcu_comm_bridge::msg::McuStatus::STATE_FINISHED:
            return "Finished";
        default:
            return "Unknown";
    }
}

const char* mission_result_name(uint8_t result) {
    switch(result) {
        case ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE:
            return "DONE";
        case ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL:
            return "FAIL";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

class McuCommBridgeNode : public rclcpp::Node {
public:
    McuCommBridgeNode()
        : Node("mcu_comm_bridge_node"),
          parser_(256u, 4096u),
          imu_publish_scheduler_(100),
          odom_publish_scheduler_(200) {
        load_parameters();
        next_arm_command_seq_ = static_cast<uint16_t>(steady_now_ms() & 0xFFFFu);
        parser_.set_max_body_len(static_cast<uint16_t>(max_body_len_));
        parser_.set_raw_buffer_capacity(parser_raw_buffer_capacity_);
        imu_publish_scheduler_.set_max_reuse_age_ms(imu_max_reuse_age_ms_);
        odom_publish_scheduler_.set_max_reuse_age_ms(odom_max_reuse_age_ms_);

        create_ros_interfaces();
        open_serial();

        running_.store(true);
        rx_thread_ = std::thread(&McuCommBridgeNode::rx_loop, this);
        dispatch_thread_ = std::thread(&McuCommBridgeNode::dispatch_loop, this);

        heartbeat_timer_ = create_rate_timer(heartbeat_rate_hz_, [this]() { send_heartbeat(); });
        stats_timer_ = create_rate_timer(stats_rate_hz_, [this]() { print_stats(); });
        control_timer_ = create_rate_timer(control_rate_hz_, [this]() { control_timer_callback(); });
        imu_publish_timer_ = create_rate_timer(imu_publish_rate_hz_, [this]() { imu_publish_timer_callback(); });
        odom_publish_timer_ = create_rate_timer(odom_publish_rate_hz_, [this]() { odom_publish_timer_callback(); });
        termios_verify_timer_ = create_rate_timer(1.0 / verify_termios_period_s_, [this]() { verify_termios_callback(); });

        RCLCPP_INFO(
            get_logger(),
            "mcu_comm_bridge started: port=%s baudrate=%d imu_pub=%.1fHz odom_pub=%.1fHz raw_buffer=%zu queue=%zu",
            port_.c_str(), baudrate_, imu_publish_rate_hz_, odom_publish_rate_hz_,
            parser_raw_buffer_capacity_, frame_queue_capacity_);
    }

    ~McuCommBridgeNode() override {
        running_.store(false);
        rx_queue_cv_.notify_all();
        if(rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if(dispatch_thread_.joinable()) {
            dispatch_thread_.join();
        }
        serial_.close();
    }

private:
    void load_parameters() {
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = declare_parameter<int>("baudrate", 1000000);
        heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 1.0);
        stats_rate_hz_ = declare_parameter<double>("stats_rate_hz", 1.0);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);

        imu_publish_rate_hz_ = read_positive_rate("imu_publish_rate_hz", 100.0);
        odom_publish_rate_hz_ = read_positive_rate("odom_publish_rate_hz", 50.0);
        imu_max_reuse_age_ms_ = read_non_negative_ms("imu_max_reuse_age_ms", 100);
        odom_max_reuse_age_ms_ = read_non_negative_ms("odom_max_reuse_age_ms", 200);
        verify_termios_period_s_ = read_positive_rate("verify_termios_period_s", 5.0);

        const auto stamp_mode = declare_parameter<std::string>("reused_message_stamp_mode", "preserve_source");
        if(stamp_mode == "publish_now") {
            reused_message_stamp_mode_ = ReusedMessageStampMode::PublishNow;
            RCLCPP_WARN(
                get_logger(),
                "reused_message_stamp_mode=publish_now repackages old measurements with new timestamps; keep this non-default");
        }
        else {
            reused_message_stamp_mode_ = ReusedMessageStampMode::PreserveSource;
        }

        max_body_len_ = declare_parameter<int>("max_body_len", 256);
        parser_raw_buffer_capacity_ = static_cast<size_t>(read_non_negative_ms("parser_raw_buffer_capacity", 4096));
        frame_queue_capacity_ = static_cast<size_t>(read_non_negative_ms("frame_queue_capacity", 512));
        if(parser_raw_buffer_capacity_ == 0u) {
            parser_raw_buffer_capacity_ = 4096u;
        }
        if(frame_queue_capacity_ == 0u) {
            frame_queue_capacity_ = 512u;
        }

        auto_ack_start_sensor_event_ = declare_parameter<bool>("auto_ack_start_sensor_event", true);
        log_latest_sample_ = declare_parameter<bool>("log_latest_sample", false);

        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
        arm_joint_state_topic_ = declare_parameter<std::string>("arm_joint_state_topic", "/arm/joint_states");
        arm_pose_topic_ = declare_parameter<std::string>("arm_pose_topic", "/arm/pose");
        arm_pose_position_topic_ = declare_parameter<std::string>("arm_pose_position_topic", "/arm/pose_position");
        mcu_status_topic_ = declare_parameter<std::string>("mcu_status_topic", "/mcu/status");
        auto_task_event_topic_ = declare_parameter<std::string>("auto_task_event_topic", "/mcu/auto_task_event");
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/motor_cmd_vel");
        brake_service_ = declare_parameter<std::string>("brake_service", "/mcu/set_brake");
        estop_service_ = declare_parameter<std::string>("estop_service", "/mcu/estop");
        arm_joints_service_ = declare_parameter<std::string>("arm_joints_service", "/mcu/set_arm_joints");
        arm_pose_service_ = declare_parameter<std::string>("arm_pose_service", "/mcu/set_arm_pose");
        arm_position_service_ = declare_parameter<std::string>("arm_position_service", "/mcu/set_arm_position");
        arm_orientation_service_ = declare_parameter<std::string>("arm_orientation_service", "/mcu/set_arm_orientation");
        yaw_hold_service_ = declare_parameter<std::string>("yaw_hold_service", "/mcu/set_yaw_hold");
        yaw_target_service_ = declare_parameter<std::string>("yaw_target_service", "/mcu/set_yaw_target");
        mission_result_service_ = declare_parameter<std::string>("mission_result_service", "/mcu/report_mission_result");

        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
        arm_frame_id_ = declare_parameter<std::string>("arm_frame_id", "arm_base_link");
        publish_tf_ = declare_parameter<bool>("publish_tf", true);

        cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 200);
        send_brake_on_cmd_timeout_ = declare_parameter<bool>("send_brake_on_cmd_timeout", true);
        max_vx_m_s_ = declare_parameter<double>("max_vx_m_s", 1.5);
        max_vy_m_s_ = declare_parameter<double>("max_vy_m_s", 1.5);
        max_wz_rad_s_ = declare_parameter<double>("max_wz_rad_s", 1.0);
        arm_command_repeat_count_ = declare_parameter<int>("arm_command_repeat_count", 3);
        mission_event_repeat_count_ = read_repeat_count("mission_event_repeat_count", 3, 10);
        repeat_estop_count_ = declare_parameter<int>("repeat_estop_count", 3);
    }

    double read_positive_rate(const char* name, double fallback) {
        const double value = declare_parameter<double>(name, fallback);
        if(value > 0.0) {
            return value;
        }
        RCLCPP_WARN(get_logger(), "%s must be > 0, fallback to %.3f", name, fallback);
        return fallback;
    }

    int read_non_negative_ms(const char* name, int fallback) {
        const int value = declare_parameter<int>(name, fallback);
        if(value >= 0) {
            return value;
        }
        RCLCPP_WARN(get_logger(), "%s must be >= 0, fallback to %d", name, fallback);
        return fallback;
    }

    int read_repeat_count(const char* name, int fallback, int max_value) {
        const int value = declare_parameter<int>(name, fallback);
        if(value >= 1 && value <= max_value) {
            return value;
        }
        RCLCPP_WARN(get_logger(), "%s must be in [1, %d], fallback to %d", name, max_value, fallback);
        return fallback;
    }

    rclcpp::TimerBase::SharedPtr create_rate_timer(double rate_hz, std::function<void()> callback) {
        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        return create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::move(callback));
    }

    void create_ros_interfaces() {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
        arm_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(arm_joint_state_topic_, rclcpp::QoS(20));
        arm_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(arm_pose_topic_, rclcpp::QoS(20));
        arm_pose_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(arm_pose_position_topic_, rclcpp::QoS(20));
        mcu_status_pub_ = create_publisher<::mcu_comm_bridge::msg::McuStatus>(
            mcu_status_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        auto_task_event_pub_ = create_publisher<::mcu_comm_bridge::msg::AutoTaskEvent>(
            auto_task_event_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

        if(publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { handle_cmd_vel(*msg); });

        brake_srv_ = create_service<std_srvs::srv::SetBool>(
            brake_service_,
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                handle_set_brake(request, response);
            });

        arm_joints_srv_ = create_service<::mcu_comm_bridge::srv::SetArmJoints>(
            arm_joints_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::SetArmJoints::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::SetArmJoints::Response> response) {
                handle_set_arm_joints(request, response);
            });

        arm_pose_srv_ = create_service<::mcu_comm_bridge::srv::SetArmPose>(
            arm_pose_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::SetArmPose::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::SetArmPose::Response> response) {
                handle_set_arm_pose(request, response);
            });

        arm_position_srv_ = create_service<::mcu_comm_bridge::srv::SetArmPosition>(
            arm_position_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::SetArmPosition::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::SetArmPosition::Response> response) {
                handle_set_arm_position(request, response);
            });

        arm_orientation_srv_ = create_service<::mcu_comm_bridge::srv::SetArmOrientation>(
            arm_orientation_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::SetArmOrientation::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::SetArmOrientation::Response> response) {
                handle_set_arm_orientation(request, response);
            });

        yaw_hold_srv_ = create_service<std_srvs::srv::SetBool>(
            yaw_hold_service_,
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                handle_set_yaw_hold(request, response);
            });

        yaw_target_srv_ = create_service<::mcu_comm_bridge::srv::SetYawTarget>(
            yaw_target_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Response> response) {
                handle_set_yaw_target(request, response);
            });

        mission_result_srv_ = create_service<::mcu_comm_bridge::srv::ReportMissionResult>(
            mission_result_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Response> response) {
                handle_report_mission_result(request, response);
            });

        estop_srv_ = create_service<::mcu_comm_bridge::srv::Estop>(
            estop_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::Estop::Request> request,
                    std::shared_ptr<::mcu_comm_bridge::srv::Estop::Response> response) {
                handle_estop(request, response);
            });
    }

    void open_serial() {
        RCLCPP_INFO(get_logger(), "--------------------------------------------------");
        try {
            serial_.open(port_, baudrate_);
            RCLCPP_INFO(get_logger(), "serial opened exclusively: %s @ %d", port_.c_str(), baudrate_);
        }
        catch(const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "failed to open serial %s: %s", port_.c_str(), e.what());
            throw;
        }
        RCLCPP_INFO(get_logger(), "--------------------------------------------------");
    }

    void rx_loop() {
        std::array<uint8_t, 1024> buf{};

        while(rclcpp::ok() && running_.load()) {
            stats_.read_call_count++;
            const int n = serial_.read_some(buf.data(), buf.size());

            if(n < 0) {
                const int err = serial_.last_read_errno();
                stats_.read_error_count++;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "serial read failed on %s: errno=%d (%s)", port_.c_str(), err, std::strerror(err));
                enter_serial_error_state("serial read error");
                break;
            }

            if(n == 0) {
                stats_.read_zero_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            stats_.read_bytes_count += static_cast<uint64_t>(n);
            update_max_atomic(stats_.max_read_batch_size, static_cast<uint64_t>(n));

            std::vector<Frame> frames;
            std::vector<ParserErrorEvent> error_events;
            {
                std::lock_guard<std::mutex> lock(parser_mutex_);
                frames = parser_.feed(buf.data(), static_cast<size_t>(n));
                error_events = parser_.take_error_events();
            }

            process_parser_errors(error_events);
            for(Frame& frame : frames) {
                enqueue_frame(std::move(frame));
            }
        }
    }

    void process_parser_errors(const std::vector<ParserErrorEvent>& error_events) {
        for(const ParserErrorEvent& event : error_events) {
            if(event.kind == ParserErrorKind::CrcError) {
                if(event.msg_id.has_value() && event.msg_id.value() == MSG_MCU_IMU) {
                    stats_.imu_rx_crc_rejected++;
                }
                else if(event.msg_id.has_value() && event.msg_id.value() == MSG_MCU_ODOM) {
                    stats_.odom_rx_crc_rejected++;
                }
                else if(event.msg_id.has_value() && event.msg_id.value() == MSG_MCU_ARM_STATE) {
                    stats_.arm_rx_crc_rejected++;
                }
                else {
                    stats_.parser_crc_error_unknown_msg++;
                }
            }
            else if(event.kind == ParserErrorKind::RawBufferOverflow) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "parser raw buffer overflow, oldest bytes dropped");
            }
            else if(event.kind == ParserErrorKind::KnownMessageBadLength &&
                    event.msg_id.has_value() &&
                    event.msg_id.value() == MSG_MCU_ARM_STATE) {
                stats_.arm_state_bad_length++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ARM_STATE dropped: expected payload=%u bytes", PAYLOAD_MCU_ARM_STATE_LEN);
            }
        }
    }

    void enqueue_frame(Frame&& frame) {
        bool dropped_oldest = false;
        {
            std::lock_guard<std::mutex> lock(rx_queue_mutex_);
            if(rx_queue_.size() >= frame_queue_capacity_) {
                rx_queue_.pop_front();
                stats_.queue_drop_count++;
                dropped_oldest = true;
            }

            rx_queue_.push_back(std::move(frame));
            update_max_atomic(stats_.queue_peak_depth, static_cast<uint64_t>(rx_queue_.size()));
        }

        if(dropped_oldest) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "frame queue full, dropping oldest frame");
        }
        rx_queue_cv_.notify_one();
    }

    void dispatch_loop() {
        while(rclcpp::ok()) {
            Frame frame;
            {
                std::unique_lock<std::mutex> lock(rx_queue_mutex_);
                rx_queue_cv_.wait(lock, [this]() {
                    return !running_.load() || !rx_queue_.empty();
                });

                if(!running_.load() && rx_queue_.empty()) {
                    break;
                }

                frame = std::move(rx_queue_.front());
                rx_queue_.pop_front();
            }

            handle_frame(frame);
        }
    }

    void handle_frame(const Frame& frame) {
        try {
            switch(frame.msg_id) {
                case MSG_MCU_IMU:
                    handle_imu(frame);
                    break;
                case MSG_MCU_ODOM:
                    handle_odom(frame);
                    break;
                case MSG_MCU_ARM_STATE:
                    handle_arm_state(frame);
                    break;
                case MSG_MCU_STATUS:
                    handle_status(frame);
                    break;
                case MSG_MCU_START_SENSOR_EVENT:
                    handle_start_sensor_event(frame);
                    break;
                case MSG_MCU_ACK:
                    break;
                case MSG_MCU_FAULT_EVENT:
                    break;
                default:
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "unknown MCU msg_id=0x%02X payload_len=%zu", frame.msg_id, frame.payload.size());
                    break;
            }
        }
        catch(const std::exception& e) {
            RCLCPP_WARN(get_logger(), "failed to handle msg_id=0x%02X: %s", frame.msg_id, e.what());
        }
    }

    void handle_imu(const Frame& frame) {
        ImuSample sample;
        sample.stamp_ms = read_u32_le(frame.payload, 0);
        sample.status_flags = read_u16_le(frame.payload, 4);
        sample.sequence_count = read_u16_le(frame.payload, 6);
        sample.acc_x_mm_s2 = read_i32_le(frame.payload, 8);
        sample.acc_y_mm_s2 = read_i32_le(frame.payload, 12);
        sample.acc_z_mm_s2 = read_i32_le(frame.payload, 16);
        sample.gyro_x_urad_s = read_i32_le(frame.payload, 20);
        sample.gyro_y_urad_s = read_i32_le(frame.payload, 24);
        sample.gyro_z_urad_s = read_i32_le(frame.payload, 28);
        sample.roll_urad = read_i32_le(frame.payload, 32);
        sample.pitch_urad = read_i32_le(frame.payload, 36);
        sample.yaw_urad = read_i32_le(frame.payload, 40);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_imu_.sample = sample;
            latest_imu_.source_stamp = now();
            latest_imu_.published_stamp = rclcpp::Time{ 0, 0, get_clock()->get_clock_type() };
            latest_imu_.receive_tp = SteadyClock::now();
            latest_imu_.has_value = true;
            latest_imu_.has_published = false;
            imu_publish_scheduler_.note_valid(steady_now_ms());
        }
        stats_.imu_rx_valid++;
    }

    void handle_odom(const Frame& frame) {
        OdomState odom;
        odom.stamp_ms = read_u32_le(frame.payload, 0);
        odom.status_flags = read_u16_le(frame.payload, 4);
        odom.reset_counter = read_u16_le(frame.payload, 6);
        odom.x_mm = read_i32_le(frame.payload, 8);
        odom.y_mm = read_i32_le(frame.payload, 12);
        odom.yaw_urad = read_i32_le(frame.payload, 16);
        odom.vx_mm_s = read_i32_le(frame.payload, 20);
        odom.vy_mm_s = read_i32_le(frame.payload, 24);
        odom.wz_urad_s = read_i32_le(frame.payload, 28);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_odom_.sample = odom;
            latest_odom_.source_stamp = now();
            latest_odom_.published_stamp = rclcpp::Time{ 0, 0, get_clock()->get_clock_type() };
            latest_odom_.receive_tp = SteadyClock::now();
            latest_odom_.has_value = true;
            latest_odom_.has_published = false;
            odom_publish_scheduler_.note_valid(steady_now_ms());
        }
        stats_.odom_rx_valid++;
    }

    void handle_arm_state(const Frame& frame) {
        DecodedArmState arm;
        const ArmStateDecodeError decode_error = decode_arm_state_payload(frame.payload, &arm);
        if(decode_error == ArmStateDecodeError::BadLength) {
            stats_.arm_state_bad_length++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ARM_STATE dropped: invalid payload length=%zu", frame.payload.size());
            return;
        }

        const auto stamp = now();
        publish_arm_state(arm, stamp);
        stats_.arm_state_valid++;
        if(arm.joint_valid) {
            stats_.arm_joint_valid++;
        }
        if(arm.pose_valid) {
            stats_.arm_pose_valid++;
        }
        else if(arm.pose_flag_set && decode_error != ArmStateDecodeError::None) {
            stats_.arm_pose_invalid_quaternion++;
        }
    }

    ::mcu_comm_bridge::msg::McuStatus make_status_msg(
        const DecodedMcuStatus& status,
        const rclcpp::Time& stamp) const {
        ::mcu_comm_bridge::msg::McuStatus msg;
        msg.header.stamp = stamp;
        msg.mcu_stamp_ms = status.stamp_ms;
        msg.app_state = status.app_state;
        msg.manual_mode = status.manual_mode;
        msg.ready_flags = status.ready_flags;
        msg.online_flags = status.online_flags;
        msg.fault_source = status.fault_source;
        msg.fault_level = status.fault_level;
        msg.fault_code = status.fault_code;
        msg.auto_start_latched = status.auto_start_latched;
        return msg;
    }

    void publish_auto_task_event(uint8_t event, const DecodedMcuStatus& status, const rclcpp::Time& stamp) {
        ::mcu_comm_bridge::msg::AutoTaskEvent msg;
        msg.header.stamp = stamp;
        msg.event = event;
        msg.mcu_stamp_ms = status.stamp_ms;
        msg.app_state = status.app_state;
        msg.auto_start_latched = status.auto_start_latched;
        auto_task_event_pub_->publish(msg);
    }

    void log_status_summary(const DecodedMcuStatus& status) {
        const uint8_t has_fault = (status.online_flags & (1u << 3)) != 0u ? 1u : 0u;
        RCLCPP_INFO(
            get_logger(),
            "MCU status: state=%s manual=%u auto_start=%d fault=%u source=%u level=%u code=%d",
            app_state_name(status.app_state),
            status.manual_mode,
            status.auto_start_latched ? 1 : 0,
            has_fault,
            status.fault_source,
            status.fault_level,
            status.fault_code);
    }

    void log_status_changes(const std::optional<DecodedMcuStatus>& previous, const DecodedMcuStatus& current) {
        if(!previous.has_value()) {
            log_status_summary(current);
            return;
        }

        if(previous->app_state != current.app_state) {
            RCLCPP_INFO(
                get_logger(),
                "MCU state changed: %s -> %s, auto_start=%d",
                app_state_name(previous->app_state),
                app_state_name(current.app_state),
                current.auto_start_latched ? 1 : 0);
        }

        if(previous->auto_start_latched != current.auto_start_latched) {
            RCLCPP_INFO(
                get_logger(),
                "MCU auto_start_latched changed: %d -> %d",
                previous->auto_start_latched ? 1 : 0,
                current.auto_start_latched ? 1 : 0);
        }

        if(previous->app_state != current.app_state ||
           previous->manual_mode != current.manual_mode ||
           previous->fault_source != current.fault_source ||
           previous->fault_level != current.fault_level ||
           previous->fault_code != current.fault_code ||
           previous->auto_start_latched != current.auto_start_latched) {
            log_status_summary(current);
        }
    }

    void reset_auto_task_context(bool request_brake_after_reset) {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            cmd_vel_cache_ = CmdVelCache{};
            arm_command_cache_ = ArmCommandCache{};
            brake_request_ = false;
            auto_task_reset_brake_pending_ = request_brake_after_reset;
        }

        {
            std::lock_guard<std::mutex> lock(auto_task_mutex_);
            auto_task_context_ = AutoTaskContext{};
        }
    }

    void handle_status(const Frame& frame) {
        const McuStatusDecodeResult decode_result = decode_mcu_status(frame.payload);
        if(decode_result.status == McuStatusDecodeStatus::InvalidLength) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "MCU_STATUS dropped: expected payload=%u bytes, got=%zu",
                PAYLOAD_MCU_STATUS_LEN, frame.payload.size());
            return;
        }

        const DecodedMcuStatus& status = decode_result.decoded;
        const rclcpp::Time stamp = now();

        if(decode_result.status == McuStatusDecodeStatus::InvalidAutoStartLatchedValue) {
            stats_.status_auto_start_invalid++;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "MCU_STATUS protocol anomaly: invalid auto_start_latched raw=%u, treating as latched",
                decode_result.raw_auto_start_latched);
        }

        std::optional<DecodedMcuStatus> previous_status;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if(has_status_) {
                previous_status = latest_status_;
            }
            latest_status_ = status;
            has_status_ = true;
        }

        mcu_status_pub_->publish(make_status_msg(status, stamp));
        log_status_changes(previous_status, status);

        const AutoTaskUpdate tracker_update = auto_task_latch_tracker_.update(status.auto_start_latched, status.app_state);
        if(tracker_update.first_observation && !status.auto_start_latched) {
            reset_auto_task_context(false);
            RCLCPP_INFO(get_logger(), "MCU auto task baseline established with auto_start=0; local task context cleared");
        }

        if(tracker_update.start_pending) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "MCU auto task pending: auto_start=1 but state=%s, waiting for state=AutoPi",
                app_state_name(status.app_state));
        }

        if(tracker_update.transition == AutoTaskTransition::Start) {
            {
                std::lock_guard<std::mutex> lock(auto_task_mutex_);
                auto_task_context_.auto_start_triggered = true;
                auto_task_context_.auto_start_consumed = true;
                auto_task_context_.mission_active = true;
                auto_task_context_.mission_done = false;
                auto_task_context_.mission_failed = false;
                auto_task_context_.pending_mission_result = false;
            }
            publish_auto_task_event(::mcu_comm_bridge::msg::AutoTaskEvent::EVENT_START, status, stamp);
            RCLCPP_INFO(
                get_logger(),
                "MCU auto task START detected: state=%s auto_start=%d",
                app_state_name(status.app_state),
                status.auto_start_latched ? 1 : 0);
        }
        else if(tracker_update.transition == AutoTaskTransition::Reset) {
            const bool request_brake = status.app_state != ::mcu_comm_bridge::msg::McuStatus::STATE_IDLE;
            reset_auto_task_context(request_brake);
            publish_auto_task_event(::mcu_comm_bridge::msg::AutoTaskEvent::EVENT_RESET, status, stamp);
            RCLCPP_INFO(
                get_logger(),
                "MCU auto task RESET detected: auto_start=%d, local task context cleared",
                status.auto_start_latched ? 1 : 0);
        }

        stats_.status_rx_valid++;
    }

    void handle_start_sensor_event(const Frame& frame) {
        if(auto_ack_start_sensor_event_ && ((frame.flags & FLAG_NEED_ACK) != 0u)) {
            send_pi_ack(frame.msg_id, frame.seq, 0u);
        }
    }

    void imu_publish_timer_callback() {
        PublishOutcome outcome;
        ImuSample sample{};
        rclcpp::Time stamp = now();

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            outcome = imu_publish_scheduler_.on_tick(steady_now_ms());
            if(outcome == PublishOutcome::Fresh || outcome == PublishOutcome::Reused) {
                sample = latest_imu_.sample;
                const auto candidate_stamp = now();
                stamp = strictly_increasing_stamp(candidate_stamp, latest_imu_.published_stamp, latest_imu_.has_published);
                latest_imu_.published_stamp = stamp;
                latest_imu_.has_published = true;
            }
        }

        if(outcome == PublishOutcome::NoValidData) {
            stats_.imu_publish_no_valid_data++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "imu publish skipped: no valid data yet");
            return;
        }
        if(outcome == PublishOutcome::StaleDrop) {
            stats_.imu_publish_stale_drop++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "imu publish skipped: latest valid data is stale");
            return;
        }

        publish_imu(sample, stamp);
        stats_.imu_publish_total++;
        if(outcome == PublishOutcome::Fresh) {
            stats_.imu_publish_fresh++;
        }
        else {
            stats_.imu_publish_reused++;
        }
    }

    void odom_publish_timer_callback() {
        PublishOutcome outcome;
        OdomState odom{};
        rclcpp::Time stamp = now();

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            outcome = odom_publish_scheduler_.on_tick(steady_now_ms());
            if(outcome == PublishOutcome::Fresh || outcome == PublishOutcome::Reused) {
                odom = latest_odom_.sample;
                const auto candidate_stamp = now();
                stamp = strictly_increasing_stamp(candidate_stamp, latest_odom_.published_stamp, latest_odom_.has_published);
                latest_odom_.published_stamp = stamp;
                latest_odom_.has_published = true;
            }
        }

        if(outcome == PublishOutcome::NoValidData) {
            stats_.odom_publish_no_valid_data++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom publish skipped: no valid data yet");
            return;
        }
        if(outcome == PublishOutcome::StaleDrop) {
            stats_.odom_publish_stale_drop++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom publish skipped: latest valid data is stale");
            return;
        }

        publish_odom(odom, stamp);
        stats_.odom_publish_total++;
        if(outcome == PublishOutcome::Fresh) {
            stats_.odom_publish_fresh++;
        }
        else {
            stats_.odom_publish_reused++;
        }
    }

    void publish_imu(const ImuSample& sample, const rclcpp::Time& stamp) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_id_;
        msg.orientation = quaternion_from_rpy(
            urad_to_rad(sample.roll_urad),
            urad_to_rad(sample.pitch_urad),
            urad_to_rad(sample.yaw_urad));
        msg.angular_velocity.x = urad_s_to_rad_s(sample.gyro_x_urad_s);
        msg.angular_velocity.y = urad_s_to_rad_s(sample.gyro_y_urad_s);
        msg.angular_velocity.z = urad_s_to_rad_s(sample.gyro_z_urad_s);
        msg.linear_acceleration.x = mm_s2_to_m_s2(sample.acc_x_mm_s2);
        msg.linear_acceleration.y = mm_s2_to_m_s2(sample.acc_y_mm_s2);
        msg.linear_acceleration.z = mm_s2_to_m_s2(sample.acc_z_mm_s2);
        imu_pub_->publish(msg);
    }

    void publish_odom(const OdomState& odom, const rclcpp::Time& stamp) {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_id_;
        msg.child_frame_id = base_frame_id_;
        msg.pose.pose.position.x = mm_to_m(odom.x_mm);
        msg.pose.pose.position.y = mm_to_m(odom.y_mm);
        msg.pose.pose.orientation = quaternion_from_rpy(0.0, 0.0, urad_to_rad(odom.yaw_urad));
        msg.twist.twist.linear.x = mm_s_to_m_s(odom.vx_mm_s);
        msg.twist.twist.linear.y = mm_s_to_m_s(odom.vy_mm_s);
        msg.twist.twist.angular.z = urad_s_to_rad_s(odom.wz_urad_s);
        odom_pub_->publish(msg);

        if(publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = msg.header;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = msg.pose.pose.position.x;
            tf_msg.transform.translation.y = msg.pose.pose.position.y;
            tf_msg.transform.rotation = msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    void publish_arm_state(const DecodedArmState& arm, const rclcpp::Time& stamp) {
        if((arm.status_flags & ARM_STATE_FLAG_JOINT_VALID) != 0u) {
            sensor_msgs::msg::JointState joint_msg;
            joint_msg.header.stamp = stamp;
            joint_msg.header.frame_id = arm_frame_id_;
            joint_msg.name = { "q0", "q1", "q2", "q3", "q4" };
            joint_msg.position = {
                arm.joints_rad[0],
                arm.joints_rad[1],
                arm.joints_rad[2],
                arm.joints_rad[3],
                arm.joints_rad[4],
            };
            arm_joint_state_pub_->publish(joint_msg);
        }

        if(arm.pose_valid) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = arm_frame_id_;
            pose_msg.pose.position.x = arm.position_x_m;
            pose_msg.pose.position.y = arm.position_y_m;
            pose_msg.pose.position.z = arm.position_z_m;
            pose_msg.pose.orientation.x = arm.orientation_x;
            pose_msg.pose.orientation.y = arm.orientation_y;
            pose_msg.pose.orientation.z = arm.orientation_z;
            pose_msg.pose.orientation.w = arm.orientation_w;
            arm_pose_pub_->publish(pose_msg);

            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header.stamp = stamp;
            point_msg.header.frame_id = arm_frame_id_;
            point_msg.point.x = arm.position_x_m;
            point_msg.point.y = arm.position_y_m;
            point_msg.point.z = arm.position_z_m;
            arm_pose_position_pub_->publish(point_msg);
        }
    }

    void handle_cmd_vel(const geometry_msgs::msg::Twist& msg) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        cmd_vel_cache_.twist = msg;
        cmd_vel_cache_.last_update = SteadyClock::now();
        cmd_vel_cache_.has_cmd = true;
        cmd_vel_cache_.timeout_brake_sent = false;
    }

    void handle_set_brake(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            brake_request_ = request->data;
        }

        if(request->data) {
            geometry_msgs::msg::Twist zero;
            response->success = send_pi_control(&zero, nullptr, true);
            response->message = response->success ? "brake latch enabled" : "failed to send brake frame";
            return;
        }

        response->success = true;
        response->message = "brake latch disabled";
    }

    uint16_t allocate_arm_command_seq() {
        const uint16_t seq = next_arm_command_seq_;
        next_arm_command_seq_ = static_cast<uint16_t>(next_arm_command_seq_ + 1u);
        return seq;
    }

    bool queue_arm_command(uint8_t arm_mode,
                           const std::array<int32_t, 5>& arm_target,
                           uint16_t arm_speed_mrad_s,
                           uint16_t command_seq) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        arm_command_cache_.arm_mode = arm_mode;
        arm_command_cache_.arm_target = arm_target;
        arm_command_cache_.arm_speed_mrad_s = arm_speed_mrad_s;
        arm_command_cache_.command_seq = command_seq;
        arm_command_cache_.repeats_remaining = std::max(1, arm_command_repeat_count_);
        arm_command_cache_.has_command = true;
        return true;
    }

    void handle_set_arm_joints(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetArmJoints::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetArmJoints::Response> response) {
        std::array<int32_t, 5> target{ 0, 0, 0, 0, 0 };
        uint16_t speed_mrad_s = 0u;

        for(size_t i = 0; i < target.size(); ++i) {
            if(!try_rad_to_urad_i32(request->joints_rad[i], &target[i])) {
                response->success = false;
                response->message = "joint target out of protocol range";
                stats_.arm_service_rejected++;
                return;
            }
        }

        if(!finite_non_negative(request->speed_rad_s) ||
           (request->speed_rad_s > 0.0 && !try_rad_s_to_mrad_s_u16(request->speed_rad_s, &speed_mrad_s))) {
            response->success = false;
            response->message = "invalid arm speed";
            stats_.arm_service_rejected++;
            return;
        }

        response->command_seq = allocate_arm_command_seq();
        queue_arm_command(PI_ARM_MODE_JOINTS, target, speed_mrad_s, response->command_seq);
        response->success = true;
        response->message = "arm joints command queued for transmission";
        RCLCPP_INFO(get_logger(),
                    "arm joints queued: seq=%u speed=%.3f q=[%.4f,%.4f,%.4f,%.4f,%.4f] repeats=%d",
                    static_cast<unsigned int>(response->command_seq),
                    request->speed_rad_s,
                    request->joints_rad[0],
                    request->joints_rad[1],
                    request->joints_rad[2],
                    request->joints_rad[3],
                    request->joints_rad[4],
                    std::max(1, arm_command_repeat_count_));
        stats_.arm_service_accepted++;
    }

    void handle_set_arm_pose(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetArmPose::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetArmPose::Response> response) {
        std::array<int32_t, 5> target{ 0, 0, 0, 0, 0 };
        uint16_t speed_mrad_s = 0u;

        if(!try_m_to_mm_i32(request->x_m, &target[0]) ||
           !try_m_to_mm_i32(request->y_m, &target[1]) ||
           !try_m_to_mm_i32(request->z_m, &target[2]) ||
           !try_rad_to_urad_i32(request->pitch_rad, &target[3]) ||
           !try_rad_to_urad_i32(request->yaw_rad, &target[4]) ||
           !finite_non_negative(request->speed_rad_s) ||
           (request->speed_rad_s > 0.0 && !try_rad_s_to_mrad_s_u16(request->speed_rad_s, &speed_mrad_s))) {
            response->success = false;
            response->message = "arm pose request out of protocol range";
            stats_.arm_service_rejected++;
            return;
        }

        response->command_seq = allocate_arm_command_seq();
        queue_arm_command(PI_ARM_MODE_POSE_5D, target, speed_mrad_s, response->command_seq);
        response->success = true;
        response->message = "arm pose command queued for transmission";
        RCLCPP_INFO(get_logger(),
                    "arm pose queued: seq=%u speed=%.3f xyz=[%.4f,%.4f,%.4f] pitch=%.4f yaw=%.4f repeats=%d",
                    static_cast<unsigned int>(response->command_seq),
                    request->speed_rad_s,
                    request->x_m,
                    request->y_m,
                    request->z_m,
                    request->pitch_rad,
                    request->yaw_rad,
                    std::max(1, arm_command_repeat_count_));
        stats_.arm_service_accepted++;
    }

    void handle_set_arm_position(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetArmPosition::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetArmPosition::Response> response) {
        std::array<int32_t, 5> target{ 0, 0, 0, 0, 0 };
        uint16_t speed_mrad_s = 0u;

        if(!try_m_to_mm_i32(request->x_m, &target[0]) ||
           !try_m_to_mm_i32(request->y_m, &target[1]) ||
           !try_m_to_mm_i32(request->z_m, &target[2]) ||
           !finite_non_negative(request->speed_rad_s) ||
           (request->speed_rad_s > 0.0 && !try_rad_s_to_mrad_s_u16(request->speed_rad_s, &speed_mrad_s))) {
            response->success = false;
            response->message = "arm position request out of protocol range";
            stats_.arm_service_rejected++;
            return;
        }

        response->command_seq = allocate_arm_command_seq();
        queue_arm_command(PI_ARM_MODE_POSITION, target, speed_mrad_s, response->command_seq);
        response->success = true;
        response->message = "arm position command queued for transmission";
        RCLCPP_INFO(get_logger(),
                    "arm position queued: seq=%u speed=%.3f xyz=[%.4f,%.4f,%.4f] repeats=%d",
                    static_cast<unsigned int>(response->command_seq),
                    request->speed_rad_s,
                    request->x_m,
                    request->y_m,
                    request->z_m,
                    std::max(1, arm_command_repeat_count_));
        stats_.arm_service_accepted++;
    }

    void handle_set_arm_orientation(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetArmOrientation::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetArmOrientation::Response> response) {
        std::array<int32_t, 5> target{ 0, 0, 0, 0, 0 };
        uint16_t speed_mrad_s = 0u;

        if(!try_rad_to_urad_i32(request->pitch_rad, &target[0]) ||
           !try_rad_to_urad_i32(request->yaw_rad, &target[1]) ||
           !finite_non_negative(request->speed_rad_s) ||
           (request->speed_rad_s > 0.0 && !try_rad_s_to_mrad_s_u16(request->speed_rad_s, &speed_mrad_s))) {
            response->success = false;
            response->message = "arm orientation request out of protocol range";
            stats_.arm_service_rejected++;
            return;
        }

        response->command_seq = allocate_arm_command_seq();
        queue_arm_command(PI_ARM_MODE_ORIENTATION_2D, target, speed_mrad_s, response->command_seq);
        response->success = true;
        response->message = "arm orientation command queued for transmission";
        RCLCPP_INFO(get_logger(),
                    "arm orientation queued: seq=%u speed=%.3f pitch=%.4f yaw=%.4f repeats=%d",
                    static_cast<unsigned int>(response->command_seq),
                    request->speed_rad_s,
                    request->pitch_rad,
                    request->yaw_rad,
                    std::max(1, arm_command_repeat_count_));
        stats_.arm_service_accepted++;
    }

    void handle_set_yaw_hold(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const bool sent = send_yaw_action(request->data ? PI_YAW_ACTION_HOLD_ENABLE : PI_YAW_ACTION_HOLD_DISABLE, 0.0);
        response->success = sent;
        response->message = sent ? "yaw hold updated" : "failed to send yaw hold action";
    }

    void handle_set_yaw_target(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Response> response) {
        const bool sent = send_yaw_action(PI_YAW_ACTION_TARGET_SET, request->yaw_rad);
        response->success = sent;
        response->message = sent ? "yaw target sent" : "failed to send yaw target";
    }

    void handle_estop(
        const std::shared_ptr<::mcu_comm_bridge::srv::Estop::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::Estop::Response> response) {
        int sent_count = 0;
        for(int i = 0; i < std::max(1, repeat_estop_count_); ++i) {
            if(send_estop(request->reason)) {
                ++sent_count;
            }
        }
        response->success = sent_count > 0;
        response->message = response->success ? "estop sent" : "failed to send estop";
    }

    void handle_report_mission_result(
        const std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Response> response) {
        try {
            handle_report_mission_result_impl(*request, response);
        }
        catch(const std::exception& e) {
            response->success = false;
            response->message = "mission result handling failed";
            response->sent_count = 0u;
            stats_.mission_result_rejected++;
            RCLCPP_WARN(get_logger(), "PI mission result handling failed: %s", e.what());
        }
    }

    void handle_report_mission_result_impl(
        const ::mcu_comm_bridge::srv::ReportMissionResult::Request& request,
        std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Response> response) {
        uint8_t event = 0u;
        int16_t code = 0;
        if(request.result == ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE) {
            event = PI_MISSION_EVENT_DONE;
            code = 0;
        }
        else if(request.result == ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL) {
            event = PI_MISSION_EVENT_FAIL;
            code = request.code;
        }
        else {
            reject_mission_result(response, "unsupported mission result", "unsupported mission result");
            return;
        }

        std::optional<DecodedMcuStatus> status;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if(has_status_) {
                status = latest_status_;
            }
        }

        if(!status.has_value()) {
            reject_mission_result(response, "MCU status is not available", "MCU status is not available");
            return;
        }
        if(status->app_state != ::mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI) {
            reject_mission_result(
                response,
                "MCU is not in AutoPi",
                std::string("MCU state=") + app_state_name(status->app_state));
            return;
        }
        if(!status->auto_start_latched) {
            reject_mission_result(response, "auto task is not latched", "auto task is not latched");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(auto_task_mutex_);
            if(!auto_task_context_.mission_active) {
                if(auto_task_context_.mission_done || auto_task_context_.mission_failed) {
                    reject_mission_result(response, "mission result already reported", "result already reported");
                }
                else {
                    reject_mission_result(response, "no active mission", "no active mission");
                }
                return;
            }
            if(auto_task_context_.mission_done || auto_task_context_.mission_failed) {
                reject_mission_result(response, "mission result already reported", "result already reported");
                return;
            }
            if(auto_task_context_.pending_mission_result) {
                reject_mission_result(
                    response,
                    "mission result report is already in progress",
                    "mission result report is already in progress");
                return;
            }
            auto_task_context_.pending_mission_result = true;
        }

        int sent_count = 0;
        for(int i = 0; i < mission_event_repeat_count_; ++i) {
            if(send_mission_event(event, code)) {
                ++sent_count;
            }
        }

        {
            std::lock_guard<std::mutex> lock(auto_task_mutex_);
            auto_task_context_.pending_mission_result = false;
            if(sent_count > 0) {
                auto_task_context_.mission_active = false;
                auto_task_context_.mission_done =
                    request.result == ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE;
                auto_task_context_.mission_failed =
                    request.result == ::mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL;
            }
            else {
                auto_task_context_.mission_active = true;
                auto_task_context_.mission_done = false;
                auto_task_context_.mission_failed = false;
            }
        }

        response->success = sent_count > 0;
        response->message = response->success ? "mission result sent" : "failed to send mission result";
        response->sent_count = static_cast<uint8_t>(std::clamp(sent_count, 0, 255));

        if(response->success) {
            stats_.mission_result_accepted++;
            RCLCPP_INFO(
                get_logger(),
                "PI mission result sent: result=%s code=%d sent=%d",
                mission_result_name(request.result),
                static_cast<int>(code),
                sent_count);
        }
        else {
            stats_.mission_result_rejected++;
            RCLCPP_WARN(
                get_logger(),
                "PI mission result rejected: failed to send result=%s code=%d",
                mission_result_name(request.result),
                static_cast<int>(code));
        }
    }

    void reject_mission_result(
        std::shared_ptr<::mcu_comm_bridge::srv::ReportMissionResult::Response> response,
        const std::string& response_message,
        const std::string& log_reason) {
        response->success = false;
        response->message = response_message;
        response->sent_count = 0u;
        stats_.mission_result_rejected++;
        RCLCPP_WARN(get_logger(), "PI mission result rejected: %s", log_reason.c_str());
    }

    void control_timer_callback() {
        geometry_msgs::msg::Twist cmd;
        bool brake = false;
        bool has_chassis = false;
        ArmCommandCache arm_snapshot;
        bool has_arm = false;
        bool reset_brake_snapshot = false;

        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if(auto_task_reset_brake_pending_) {
                reset_brake_snapshot = true;
                brake = true;
                has_chassis = true;
            }
            else if(brake_request_) {
                brake = true;
                has_chassis = true;
            }
            else if(cmd_vel_cache_.has_cmd) {
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    SteadyClock::now() - cmd_vel_cache_.last_update).count();
                if(elapsed_ms <= cmd_vel_timeout_ms_) {
                    cmd = cmd_vel_cache_.twist;
                    has_chassis = true;
                    cmd_vel_cache_.timeout_brake_sent = false;
                }
                else if(send_brake_on_cmd_timeout_ && !cmd_vel_cache_.timeout_brake_sent) {
                    brake = true;
                    has_chassis = true;
                    cmd_vel_cache_.timeout_brake_sent = true;
                }
            }

            if(arm_command_cache_.has_command && arm_command_cache_.repeats_remaining > 0) {
                arm_snapshot = arm_command_cache_;
                has_arm = true;
            }
        }

        if(!has_chassis && !has_arm) {
            return;
        }

        if(has_chassis && brake) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
        }
        else if(has_chassis) {
            cmd.linear.x = std::clamp(cmd.linear.x, -max_vx_m_s_, max_vx_m_s_);
            cmd.linear.y = std::clamp(cmd.linear.y, -max_vy_m_s_, max_vy_m_s_);
            cmd.angular.z = std::clamp(cmd.angular.z, -max_wz_rad_s_, max_wz_rad_s_);
        }

        if(send_pi_control(has_chassis ? &cmd : nullptr, has_arm ? &arm_snapshot : nullptr, brake)) {
            if(has_arm) {
                std::lock_guard<std::mutex> lock(control_mutex_);
                if(arm_command_cache_.has_command &&
                   arm_command_cache_.command_seq == arm_snapshot.command_seq &&
                   arm_command_cache_.repeats_remaining > 0) {
                    arm_command_cache_.repeats_remaining--;
                    if(arm_command_cache_.repeats_remaining <= 0) {
                        arm_command_cache_.has_command = false;
                    }
                }
            }
            if(reset_brake_snapshot) {
                std::lock_guard<std::mutex> lock(control_mutex_);
                auto_task_reset_brake_pending_ = false;
            }
        }
    }

    void send_heartbeat() {
        if(send_frame(MSG_PI_HEARTBEAT, 0u, {})) {
            stats_.tx_heartbeat++;
        }
    }

    bool send_pi_control(const geometry_msgs::msg::Twist* cmd,
                         const ArmCommandCache* arm_command,
                         bool brake_request) {
        std::vector<uint8_t> payload(PAYLOAD_PI_CONTROL_LEN, 0u);
        write_u32_le(payload, 0, ros_now_ms_u32());

        if(cmd != nullptr) {
            payload[4] |= PI_CONTROL_MASK_CHASSIS_VALID;
            write_i16_le(payload, 8, m_s_to_mm_s_i16(cmd->linear.x));
            write_i16_le(payload, 10, m_s_to_mm_s_i16(cmd->linear.y));
            write_i16_le(payload, 12, rad_s_to_mrad_s_i16(cmd->angular.z));
        }
        if(brake_request) {
            payload[4] |= PI_CONTROL_MASK_BRAKE_REQUEST;
        }

        payload[5] = PI_ARM_MODE_NONE;
        write_u16_le(payload, 6, 0u);
        if(arm_command != nullptr && arm_command->has_command && arm_command->repeats_remaining > 0) {
            payload[4] |= PI_CONTROL_MASK_ARM_VALID;
            payload[5] = arm_command->arm_mode;
            write_u16_le(payload, 6, arm_command->command_seq);
            for(size_t i = 0; i < arm_command->arm_target.size(); ++i) {
                write_i32_le(payload, 14u + static_cast<size_t>(i * 4u), arm_command->arm_target[i]);
            }
            write_u16_le(payload, 34, arm_command->arm_speed_mrad_s);
        }
        write_u16_le(payload, 36, 0u);

        if(send_frame(MSG_PI_CONTROL, 0u, payload)) {
            stats_.tx_control++;
            if(arm_command != nullptr && arm_command->has_command && arm_command->repeats_remaining > 0) {
                stats_.tx_arm_command++;
                if(arm_command->repeats_remaining < std::max(1, arm_command_repeat_count_)) {
                    stats_.tx_arm_command_retry++;
                }
            }
            return true;
        }
        return false;
    }

    bool send_yaw_action(uint8_t action, double target_yaw_rad) {
        std::vector<uint8_t> payload(PAYLOAD_PI_YAW_ACTION_LEN, 0u);
        write_u32_le(payload, 0, ros_now_ms_u32());
        payload[4] = action;
        write_i32_le(payload, 8, rad_to_urad_i32(target_yaw_rad));

        if(send_frame(MSG_PI_YAW_ACTION, 0u, payload)) {
            stats_.tx_yaw_action++;
            return true;
        }
        return false;
    }

    bool send_mission_event(uint8_t event, int16_t code) {
        std::vector<uint8_t> payload(PAYLOAD_PI_MISSION_EVENT_LEN, 0u);
        write_u32_le(payload, 0, ros_now_ms_u32());
        payload[4] = event;
        payload[5] = 0u;
        write_i16_le(payload, 6, code);

        if(send_frame(MSG_PI_MISSION_EVENT, 0u, payload)) {
            stats_.tx_mission_event++;
            return true;
        }
        return false;
    }

    bool send_estop(uint8_t reason) {
        std::vector<uint8_t> payload(PAYLOAD_PI_ESTOP_LEN, 0u);
        write_u32_le(payload, 0, ros_now_ms_u32());
        payload[4] = reason;

        if(send_frame(MSG_PI_ESTOP, 0u, payload)) {
            stats_.tx_estop++;
            return true;
        }
        return false;
    }

    void send_pi_ack(uint8_t ack_msg_id, uint8_t ack_seq, uint16_t code) {
        std::vector<uint8_t> payload(PAYLOAD_PI_ACK_LEN, 0u);
        payload[0] = ack_msg_id;
        payload[1] = ack_seq;
        write_u16_le(payload, 2, code);
        if(send_frame(MSG_PI_ACK, 0u, payload)) {
            stats_.tx_pi_ack++;
        }
    }

    bool send_frame(uint8_t msg_id, uint8_t flags, const std::vector<uint8_t>& payload) {
        if(serial_faulted_.load()) {
            stats_.tx_fail++;
            return false;
        }

        std::lock_guard<std::mutex> lock(tx_mutex_);
        const auto frame = pack_frame(msg_id, next_tx_seq(), flags, payload);
        if(!serial_.write_all(frame)) {
            stats_.tx_fail++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "serial write failed");
            return false;
        }
        return true;
    }

    uint8_t next_tx_seq() {
        return tx_seq_.fetch_add(1);
    }

    uint32_t ros_now_ms_u32() {
        const int64_t ms = now().nanoseconds() / 1000000LL;
        return static_cast<uint32_t>(ms & 0xFFFFFFFFLL);
    }

    void verify_termios_callback() {
        if(serial_faulted_.load()) {
            return;
        }

        std::string reason;
        if(serial_.verify_configuration(&reason)) {
            return;
        }

        stats_.termios_mismatch_count++;
        RCLCPP_ERROR(get_logger(), "serial termios mismatch detected: %s", reason.c_str());

        try {
            serial_.reapply_configuration();
        }
        catch(const std::exception& e) {
            enter_serial_error_state(std::string("failed to reapply termios: ") + e.what());
            return;
        }

        if(!serial_.verify_configuration(&reason)) {
            enter_serial_error_state(std::string("termios verification failed after reapply: ") + reason);
        }
    }

    void enter_serial_error_state(const std::string& reason) {
        if(serial_faulted_.exchange(true)) {
            return;
        }

        running_.store(false);
        rx_queue_cv_.notify_all();
        serial_.close();
        RCLCPP_FATAL(get_logger(), "serial error state entered: %s", reason.c_str());
    }

    BridgeStatsSnapshot snapshot_stats() {
        BridgeStatsSnapshot snapshot;
        snapshot.read_call_count = stats_.read_call_count.load();
        snapshot.read_zero_count = stats_.read_zero_count.load();
        snapshot.read_error_count = stats_.read_error_count.load();
        snapshot.read_bytes_count = stats_.read_bytes_count.load();
        snapshot.max_read_batch_size = stats_.max_read_batch_size.load();
        snapshot.queue_drop_count = stats_.queue_drop_count.load();
        snapshot.queue_peak_depth = stats_.queue_peak_depth.load();
        snapshot.termios_mismatch_count = stats_.termios_mismatch_count.load();
        snapshot.imu_rx_valid = stats_.imu_rx_valid.load();
        snapshot.odom_rx_valid = stats_.odom_rx_valid.load();
        snapshot.arm_state_valid = stats_.arm_state_valid.load();
        snapshot.arm_joint_valid = stats_.arm_joint_valid.load();
        snapshot.arm_pose_valid = stats_.arm_pose_valid.load();
        snapshot.arm_pose_invalid_quaternion = stats_.arm_pose_invalid_quaternion.load();
        snapshot.arm_state_bad_length = stats_.arm_state_bad_length.load();
        snapshot.status_rx_valid = stats_.status_rx_valid.load();
        snapshot.status_auto_start_invalid = stats_.status_auto_start_invalid.load();
        snapshot.imu_rx_crc_rejected = stats_.imu_rx_crc_rejected.load();
        snapshot.odom_rx_crc_rejected = stats_.odom_rx_crc_rejected.load();
        snapshot.arm_rx_crc_rejected = stats_.arm_rx_crc_rejected.load();
        snapshot.parser_crc_error_unknown_msg = stats_.parser_crc_error_unknown_msg.load();
        snapshot.imu_publish_total = stats_.imu_publish_total.load();
        snapshot.imu_publish_fresh = stats_.imu_publish_fresh.load();
        snapshot.imu_publish_reused = stats_.imu_publish_reused.load();
        snapshot.imu_publish_stale_drop = stats_.imu_publish_stale_drop.load();
        snapshot.imu_publish_no_valid_data = stats_.imu_publish_no_valid_data.load();
        snapshot.odom_publish_total = stats_.odom_publish_total.load();
        snapshot.odom_publish_fresh = stats_.odom_publish_fresh.load();
        snapshot.odom_publish_reused = stats_.odom_publish_reused.load();
        snapshot.odom_publish_stale_drop = stats_.odom_publish_stale_drop.load();
        snapshot.odom_publish_no_valid_data = stats_.odom_publish_no_valid_data.load();
        snapshot.tx_heartbeat = stats_.tx_heartbeat.load();
        snapshot.tx_pi_ack = stats_.tx_pi_ack.load();
        snapshot.tx_control = stats_.tx_control.load();
        snapshot.tx_arm_command = stats_.tx_arm_command.load();
        snapshot.tx_arm_command_retry = stats_.tx_arm_command_retry.load();
        snapshot.tx_yaw_action = stats_.tx_yaw_action.load();
        snapshot.tx_mission_event = stats_.tx_mission_event.load();
        snapshot.tx_estop = stats_.tx_estop.load();
        snapshot.arm_service_accepted = stats_.arm_service_accepted.load();
        snapshot.arm_service_rejected = stats_.arm_service_rejected.load();
        snapshot.mission_result_accepted = stats_.mission_result_accepted.load();
        snapshot.mission_result_rejected = stats_.mission_result_rejected.load();
        snapshot.tx_fail = stats_.tx_fail.load();

        {
            std::lock_guard<std::mutex> lock(parser_mutex_);
            snapshot.parser = parser_.stats();
        }
        {
            std::lock_guard<std::mutex> lock(rx_queue_mutex_);
            snapshot.queue_depth = rx_queue_.size();
        }
        return snapshot;
    }

    void print_stats() {
        const BridgeStatsSnapshot snapshot = snapshot_stats();
        const auto now_tp = SteadyClock::now();
        const double elapsed_sec = last_stats_time_.time_since_epoch().count() == 0
            ? 0.0
            : std::chrono::duration<double>(now_tp - last_stats_time_).count();
        last_stats_time_ = now_tp;

        RCLCPP_INFO(get_logger(), "--------------------------------------------------");
        RCLCPP_INFO(
            get_logger(),
            "RAW rx_bytes_hz=%.1f read_calls_hz=%.1f read_zero_hz=%.1f read_error_hz=%.1f valid_frames_hz=%.1f queue_drop_hz=%.1f queue_depth=%" PRIu64 " max_batch=%" PRIu64,
            safe_rate(snapshot.read_bytes_count - last_stats_snapshot_.read_bytes_count, elapsed_sec),
            safe_rate(snapshot.read_call_count - last_stats_snapshot_.read_call_count, elapsed_sec),
            safe_rate(snapshot.read_zero_count - last_stats_snapshot_.read_zero_count, elapsed_sec),
            safe_rate(snapshot.read_error_count - last_stats_snapshot_.read_error_count, elapsed_sec),
            safe_rate(snapshot.parser.rx_frames - last_stats_snapshot_.parser.rx_frames, elapsed_sec),
            safe_rate(snapshot.queue_drop_count - last_stats_snapshot_.queue_drop_count, elapsed_sec),
            snapshot.queue_depth,
            snapshot.max_read_batch_size);

        RCLCPP_INFO(
            get_logger(),
            "PARSER crc_err_hz=%.1f len_err_hz=%.1f version_err_hz=%.1f known_bad_len_hz=%.1f resync_hz=%.1f overflow_hz=%.1f",
            safe_rate(snapshot.parser.crc_error - last_stats_snapshot_.parser.crc_error, elapsed_sec),
            safe_rate(snapshot.parser.len_error - last_stats_snapshot_.parser.len_error, elapsed_sec),
            safe_rate(snapshot.parser.version_error - last_stats_snapshot_.parser.version_error, elapsed_sec),
            safe_rate(snapshot.parser.known_msg_bad_length - last_stats_snapshot_.parser.known_msg_bad_length, elapsed_sec),
            safe_rate(snapshot.parser.resync - last_stats_snapshot_.parser.resync, elapsed_sec),
            safe_rate(snapshot.parser.raw_buffer_overflow - last_stats_snapshot_.parser.raw_buffer_overflow, elapsed_sec));

        RCLCPP_INFO(
            get_logger(),
            "RX valid_hz imu=%.1f odom=%.1f arm=%.1f status=%.1f status_auto_start_invalid=%.1f crc_reject imu=%.1f odom=%.1f arm=%.1f unknown=%.1f",
            safe_rate(snapshot.imu_rx_valid - last_stats_snapshot_.imu_rx_valid, elapsed_sec),
            safe_rate(snapshot.odom_rx_valid - last_stats_snapshot_.odom_rx_valid, elapsed_sec),
            safe_rate(snapshot.arm_state_valid - last_stats_snapshot_.arm_state_valid, elapsed_sec),
            safe_rate(snapshot.status_rx_valid - last_stats_snapshot_.status_rx_valid, elapsed_sec),
            safe_rate(snapshot.status_auto_start_invalid - last_stats_snapshot_.status_auto_start_invalid, elapsed_sec),
            safe_rate(snapshot.imu_rx_crc_rejected - last_stats_snapshot_.imu_rx_crc_rejected, elapsed_sec),
            safe_rate(snapshot.odom_rx_crc_rejected - last_stats_snapshot_.odom_rx_crc_rejected, elapsed_sec),
            safe_rate(snapshot.arm_rx_crc_rejected - last_stats_snapshot_.arm_rx_crc_rejected, elapsed_sec),
            safe_rate(snapshot.parser_crc_error_unknown_msg - last_stats_snapshot_.parser_crc_error_unknown_msg, elapsed_sec));

        RCLCPP_INFO(
            get_logger(),
            "ARM_STATE joint_valid=%.1f pose_valid=%.1f pose_invalid=%.1f bad_length=%.1f",
            safe_rate(snapshot.arm_joint_valid - last_stats_snapshot_.arm_joint_valid, elapsed_sec),
            safe_rate(snapshot.arm_pose_valid - last_stats_snapshot_.arm_pose_valid, elapsed_sec),
            safe_rate(snapshot.arm_pose_invalid_quaternion - last_stats_snapshot_.arm_pose_invalid_quaternion, elapsed_sec),
            safe_rate(snapshot.arm_state_bad_length - last_stats_snapshot_.arm_state_bad_length, elapsed_sec));

        RCLCPP_INFO(
            get_logger(),
            "PUB imu=%.1f fresh=%.1f reused=%.1f stale_drop=%.1f no_valid=%.1f",
            safe_rate(snapshot.imu_publish_total - last_stats_snapshot_.imu_publish_total, elapsed_sec),
            safe_rate(snapshot.imu_publish_fresh - last_stats_snapshot_.imu_publish_fresh, elapsed_sec),
            safe_rate(snapshot.imu_publish_reused - last_stats_snapshot_.imu_publish_reused, elapsed_sec),
            safe_rate(snapshot.imu_publish_stale_drop - last_stats_snapshot_.imu_publish_stale_drop, elapsed_sec),
            safe_rate(snapshot.imu_publish_no_valid_data - last_stats_snapshot_.imu_publish_no_valid_data, elapsed_sec));

        RCLCPP_INFO(
            get_logger(),
            "PUB odom=%.1f fresh=%.1f reused=%.1f stale_drop=%.1f no_valid=%.1f",
            safe_rate(snapshot.odom_publish_total - last_stats_snapshot_.odom_publish_total, elapsed_sec),
            safe_rate(snapshot.odom_publish_fresh - last_stats_snapshot_.odom_publish_fresh, elapsed_sec),
            safe_rate(snapshot.odom_publish_reused - last_stats_snapshot_.odom_publish_reused, elapsed_sec),
            safe_rate(snapshot.odom_publish_stale_drop - last_stats_snapshot_.odom_publish_stale_drop, elapsed_sec),
            safe_rate(snapshot.odom_publish_no_valid_data - last_stats_snapshot_.odom_publish_no_valid_data, elapsed_sec));

        RCLCPP_INFO(
            get_logger(),
            "TX ctrl=%.1f arm=%.1f arm_retry=%.1f arm_accept=%.1f arm_reject=%.1f yaw=%.1f mission=%.1f mission_accept=%.1f mission_reject=%.1f estop=%.1f fail=%.1f",
            safe_rate(snapshot.tx_control - last_stats_snapshot_.tx_control, elapsed_sec),
            safe_rate(snapshot.tx_arm_command - last_stats_snapshot_.tx_arm_command, elapsed_sec),
            safe_rate(snapshot.tx_arm_command_retry - last_stats_snapshot_.tx_arm_command_retry, elapsed_sec),
            safe_rate(snapshot.arm_service_accepted - last_stats_snapshot_.arm_service_accepted, elapsed_sec),
            safe_rate(snapshot.arm_service_rejected - last_stats_snapshot_.arm_service_rejected, elapsed_sec),
            safe_rate(snapshot.tx_yaw_action - last_stats_snapshot_.tx_yaw_action, elapsed_sec),
            safe_rate(snapshot.tx_mission_event - last_stats_snapshot_.tx_mission_event, elapsed_sec),
            safe_rate(snapshot.mission_result_accepted - last_stats_snapshot_.mission_result_accepted, elapsed_sec),
            safe_rate(snapshot.mission_result_rejected - last_stats_snapshot_.mission_result_rejected, elapsed_sec),
            safe_rate(snapshot.tx_estop - last_stats_snapshot_.tx_estop, elapsed_sec),
            safe_rate(snapshot.tx_fail - last_stats_snapshot_.tx_fail, elapsed_sec));

        last_stats_snapshot_ = snapshot;

        RCLCPP_INFO(get_logger(), "--------------------------------------------------");

        if(!log_latest_sample_) {
            return;
        }

        RCLCPP_INFO(get_logger(), "--------------------------------------------------");

        std::lock_guard<std::mutex> lock(data_mutex_);
        if(latest_imu_.has_value) {
            RCLCPP_INFO(get_logger(), "latest imu seq=%u stamp_ms=%u", latest_imu_.sample.sequence_count, latest_imu_.sample.stamp_ms);
        }
        if(latest_odom_.has_value) {
            RCLCPP_INFO(get_logger(), "latest odom stamp_ms=%u reset=%u", latest_odom_.sample.stamp_ms, latest_odom_.sample.reset_counter);
        }
        if(has_status_) {
            RCLCPP_INFO(
                get_logger(),
                "latest status stamp_ms=%u state=%s manual=%u auto_start=%d source=%u level=%u code=%d",
                latest_status_.stamp_ms,
                app_state_name(latest_status_.app_state),
                latest_status_.manual_mode,
                latest_status_.auto_start_latched ? 1 : 0,
                latest_status_.fault_source,
                latest_status_.fault_level,
                latest_status_.fault_code);
        }

        RCLCPP_INFO(get_logger(), "--------------------------------------------------");
    }

    static void update_max_atomic(std::atomic<uint64_t>& target, uint64_t value) {
        uint64_t current = target.load();
        while(value > current && !target.compare_exchange_weak(current, value)) {
        }
    }

    std::string port_;
    int baudrate_ = 1000000;
    double heartbeat_rate_hz_ = 1.0;
    double stats_rate_hz_ = 1.0;
    double control_rate_hz_ = 50.0;
    double imu_publish_rate_hz_ = 100.0;
    double odom_publish_rate_hz_ = 50.0;
    double verify_termios_period_s_ = 5.0;
    int imu_max_reuse_age_ms_ = 100;
    int odom_max_reuse_age_ms_ = 200;
    int max_body_len_ = 256;
    size_t parser_raw_buffer_capacity_ = 4096u;
    size_t frame_queue_capacity_ = 512u;
    bool auto_ack_start_sensor_event_ = true;
    bool log_latest_sample_ = false;
    ReusedMessageStampMode reused_message_stamp_mode_ = ReusedMessageStampMode::PreserveSource;

    std::string odom_topic_ = "/odom";
    std::string imu_topic_ = "/imu";
    std::string arm_joint_state_topic_ = "/arm/joint_states";
    std::string arm_pose_topic_ = "/arm/pose";
    std::string arm_pose_position_topic_ = "/arm/pose_position";
    std::string mcu_status_topic_ = "/mcu/status";
    std::string auto_task_event_topic_ = "/mcu/auto_task_event";
    std::string cmd_vel_topic_ = "/motor_cmd_vel";
    std::string brake_service_ = "/mcu/set_brake";
    std::string estop_service_ = "/mcu/estop";
    std::string arm_joints_service_ = "/mcu/set_arm_joints";
    std::string arm_pose_service_ = "/mcu/set_arm_pose";
    std::string arm_position_service_ = "/mcu/set_arm_position";
    std::string arm_orientation_service_ = "/mcu/set_arm_orientation";
    std::string yaw_hold_service_ = "/mcu/set_yaw_hold";
    std::string yaw_target_service_ = "/mcu/set_yaw_target";
    std::string mission_result_service_ = "/mcu/report_mission_result";
    std::string odom_frame_id_ = "odom";
    std::string base_frame_id_ = "base_footprint";
    std::string imu_frame_id_ = "imu_link";
    std::string arm_frame_id_ = "arm_base_link";
    bool publish_tf_ = true;

    int cmd_vel_timeout_ms_ = 200;
    bool send_brake_on_cmd_timeout_ = true;
    double max_vx_m_s_ = 1.5;
    double max_vy_m_s_ = 1.5;
    double max_wz_rad_s_ = 1.0;
    int arm_command_repeat_count_ = 3;
    int mission_event_repeat_count_ = 3;
    int repeat_estop_count_ = 3;

    SerialPort serial_;
    BinaryFrameParser parser_;
    FixedRatePublishScheduler imu_publish_scheduler_;
    FixedRatePublishScheduler odom_publish_scheduler_;
    BridgeStats stats_;
    BridgeStatsSnapshot last_stats_snapshot_;
    SteadyClock::time_point last_stats_time_{};

    std::atomic<bool> running_{ false };
    std::atomic<bool> serial_faulted_{ false };
    std::thread rx_thread_;
    std::thread dispatch_thread_;
    std::atomic<uint8_t> tx_seq_{ 0 };

    std::mutex tx_mutex_;
    std::mutex parser_mutex_;
    std::mutex data_mutex_;
    std::mutex rx_queue_mutex_;
    std::condition_variable rx_queue_cv_;
    std::deque<Frame> rx_queue_;

    LatestValidCache<ImuSample> latest_imu_;
    LatestValidCache<OdomState> latest_odom_;
    DecodedMcuStatus latest_status_{};
    bool has_status_ = false;

    std::mutex control_mutex_;
    std::mutex auto_task_mutex_;
    CmdVelCache cmd_vel_cache_;
    bool brake_request_ = false;
    bool auto_task_reset_brake_pending_ = false;
    ArmCommandCache arm_command_cache_;
    uint16_t next_arm_command_seq_ = 0u;
    AutoTaskContext auto_task_context_{};
    AutoTaskLatchTracker auto_task_latch_tracker_{
        static_cast<uint8_t>(::mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI)
    };

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr arm_pose_position_pub_;
    rclcpp::Publisher<::mcu_comm_bridge::msg::McuStatus>::SharedPtr mcu_status_pub_;
    rclcpp::Publisher<::mcu_comm_bridge::msg::AutoTaskEvent>::SharedPtr auto_task_event_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetArmJoints>::SharedPtr arm_joints_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetArmPose>::SharedPtr arm_pose_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetArmPosition>::SharedPtr arm_position_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetArmOrientation>::SharedPtr arm_orientation_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr yaw_hold_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetYawTarget>::SharedPtr yaw_target_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::ReportMissionResult>::SharedPtr mission_result_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::Estop>::SharedPtr estop_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr imu_publish_timer_;
    rclcpp::TimerBase::SharedPtr odom_publish_timer_;
    rclcpp::TimerBase::SharedPtr termios_verify_timer_;
};

}  // namespace mcu_comm_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcu_comm_bridge::McuCommBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
