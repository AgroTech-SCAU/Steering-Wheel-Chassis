#include "mcu_comm_bridge/binary_frame.hpp"
#include "mcu_comm_bridge/serial_port.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cinttypes>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "geometry_msgs/msg/point_stamped.hpp" // IWYU pragma: keep
#include "geometry_msgs/msg/twist.hpp" // IWYU pragma: keep
#include "nav_msgs/msg/odometry.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/imu.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/joint_state.hpp" // IWYU pragma: keep
#include "mcu_comm_bridge/srv/estop.hpp" // IWYU pragma: keep
#include "mcu_comm_bridge/srv/set_yaw_target.hpp" // IWYU pragma: keep
#include "std_srvs/srv/set_bool.hpp" // IWYU pragma: keep
#include "tf2_ros/transform_broadcaster.h" // IWYU pragma: keep

namespace mcu_comm_bridge {
namespace {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// ! ========================= 控 制 常 量 ========================= ! //

constexpr uint8_t PI_CONTROL_MASK_CHASSIS_VALID = 1u << 0; /**< PI_CONTROL: vx/vy/wz 有效 */
constexpr uint8_t PI_CONTROL_MASK_BRAKE_REQUEST = 1u << 3; /**< PI_CONTROL: 请求底盘刹车 */
constexpr uint8_t PI_ARM_MODE_NONE = 0u;                   /**< PI_CONTROL: 本节点不控制机械臂 */

constexpr uint8_t PI_YAW_ACTION_HOLD_ENABLE = 1u;          /**< PI_YAW_ACTION: 开启 yaw hold */
constexpr uint8_t PI_YAW_ACTION_HOLD_DISABLE = 2u;         /**< PI_YAW_ACTION: 关闭 yaw hold */
constexpr uint8_t PI_YAW_ACTION_TARGET_SET = 3u;           /**< PI_YAW_ACTION: 设置目标 yaw */
constexpr size_t RX_QUEUE_CAPACITY = 512u;

// ! ========================= 数 据 结 构 ========================= ! //

/**
 * @brief MCU_IMU 解析后的 IMU 样本
 *
 * MCU_IMU 为 100Hz 高频周期帧，payload 内部单位保持协议定点格式
 * 三轴角度来自 MCU 融合姿态，其中 yaw 是融合底盘后的航向角
 */
struct ImuSample {
    uint32_t stamp_ms = 0;          /**< MCU 时间戳，ms */
    uint16_t status_flags = 0;      /**< IMU 状态标志 */
    uint16_t sequence_count = 0;    /**< IMU 样本计数 */

    int32_t acc_x_mm_s2 = 0;        /**< x 轴加速度，mm/s^2 */
    int32_t acc_y_mm_s2 = 0;        /**< y 轴加速度，mm/s^2 */
    int32_t acc_z_mm_s2 = 0;        /**< z 轴加速度，mm/s^2 */

    int32_t gyro_x_urad_s = 0;      /**< x 轴角速度，urad/s */
    int32_t gyro_y_urad_s = 0;      /**< y 轴角速度，urad/s */
    int32_t gyro_z_urad_s = 0;      /**< z 轴角速度，urad/s */

    int32_t roll_urad = 0;          /**< 融合 roll，urad */
    int32_t pitch_urad = 0;         /**< 融合 pitch，urad */
    int32_t yaw_urad = 0;           /**< 融合 yaw，urad */
};

/**
 * @brief MCU_ODOM 解析后的底盘局部里程计状态
 *
 * x/y/yaw 属于 odom 坐标系下的底盘位姿
 * vx/vy/wz 属于 base_footprint 坐标系下的底盘速度
 */
struct OdomState {
    uint32_t stamp_ms = 0;          /**< MCU 时间戳，ms */
    uint16_t status_flags = 0;      /**< ODOM 状态标志 */
    uint16_t reset_counter = 0;     /**< 里程计重置计数 */

    int32_t x_mm = 0;               /**< odom 系 x，mm */
    int32_t y_mm = 0;               /**< odom 系 y，mm */
    int32_t yaw_urad = 0;           /**< odom 系 yaw，urad */

    int32_t vx_mm_s = 0;            /**< base_footprint 系 vx，mm/s */
    int32_t vy_mm_s = 0;            /**< base_footprint 系 vy，mm/s */
    int32_t wz_urad_s = 0;          /**< base_footprint 系 wz，urad/s */
};

/**
 * @brief MCU_ARM_STATE 解析后的机械臂状态
 *
 * q0~q4 为当前机械臂关节角，xyz 为 MCU 根据当前关节角正解得到的末端位置
 */
struct ArmState {
    uint32_t stamp_ms = 0;          /**< MCU 时间戳，ms */
    uint16_t status_flags = 0;      /**< 机械臂状态有效位 */
    uint16_t sequence_count = 0;    /**< 机械臂状态计数 */

    int32_t q0_urad = 0;            /**< q0，urad */
    int32_t q1_urad = 0;            /**< q1，urad */
    int32_t q2_urad = 0;            /**< q2，urad */
    int32_t q3_urad = 0;            /**< q3，urad */
    int32_t q4_urad = 0;            /**< q4，urad */

    int32_t x_mm = 0;               /**< 末端 x，mm */
    int32_t y_mm = 0;               /**< 末端 y，mm */
    int32_t z_mm = 0;               /**< 末端 z，mm */
};

/**
 * @brief MCU_STATUS 解析后的状态快照
 */
struct McuStatus {
    uint32_t stamp_ms = 0;
    uint8_t app_state = 0;
    uint8_t manual_mode = 0;
    uint8_t ready_flags = 0;
    uint8_t online_flags = 0;
    uint8_t fault_source = 0;
    uint8_t fault_level = 0;
    int16_t fault_code = 0;
};

/**
 * @brief MCU_START_SENSOR_EVENT 解析后的事件
 */
struct StartSensorEvent {
    uint32_t stamp_ms = 0;
    uint8_t sensor_id = 0;
    uint8_t event_type = 0;
    uint16_t event_value = 0;
};

/**
 * @brief MCU_FAULT_EVENT 解析后的故障事件
 */
struct FaultEvent {
    uint32_t stamp_ms = 0;
    uint8_t fault_source = 0;
    uint8_t fault_level = 0;
    int16_t fault_code = 0;
};

/**
 * @brief 导航速度命令缓存
 *
 * /motor_cmd_vel 来自 competition_fsm，是 Nav2 /cmd_vel 经过任务仲裁后的最终底盘命令
 */
struct CmdVelCache {
    geometry_msgs::msg::Twist twist;
    SteadyClock::time_point last_update{};
    bool has_cmd = false;
    bool timeout_brake_sent = false;
};

/**
 * @brief 桥接节点统计信息
 */
struct BridgeStats {
    std::atomic<uint64_t> rx_imu{ 0 };
    std::atomic<uint64_t> rx_odom{ 0 };
    std::atomic<uint64_t> rx_arm_state{ 0 };
    std::atomic<uint64_t> rx_status{ 0 };
    std::atomic<uint64_t> rx_start_sensor_event{ 0 };
    std::atomic<uint64_t> rx_mcu_ack{ 0 };
    std::atomic<uint64_t> rx_fault_event{ 0 };
    std::atomic<uint64_t> rx_unknown{ 0 };
    std::atomic<uint64_t> rx_bad_payload_len{ 0 };
    std::atomic<uint64_t> tx_heartbeat{ 0 };
    std::atomic<uint64_t> tx_pi_ack{ 0 };
    std::atomic<uint64_t> tx_control{ 0 };
    std::atomic<uint64_t> tx_yaw_action{ 0 };
    std::atomic<uint64_t> tx_estop{ 0 };
    std::atomic<uint64_t> tx_fail{ 0 };
    std::atomic<uint64_t> rx_queue_drop{ 0 };
    std::atomic<uint64_t> rx_queue_peak_depth{ 0 };
};

struct BridgeStatsSnapshot {
    uint64_t rx_imu = 0;
    uint64_t rx_odom = 0;
    uint64_t rx_arm_state = 0;
    uint64_t rx_status = 0;
    uint64_t rx_start_sensor_event = 0;
    uint64_t rx_mcu_ack = 0;
    uint64_t rx_fault_event = 0;
    uint64_t rx_unknown = 0;
    uint64_t rx_bad_payload_len = 0;
    uint64_t tx_heartbeat = 0;
    uint64_t tx_pi_ack = 0;
    uint64_t tx_control = 0;
    uint64_t tx_yaw_action = 0;
    uint64_t tx_estop = 0;
    uint64_t tx_fail = 0;
    uint64_t rx_queue_drop = 0;
    uint64_t rx_queue_peak_depth = 0;
    uint64_t rx_queue_depth = 0;
    ParserStats parser{};
};

/**
 * @brief 将 roll/pitch/yaw 转为四元数
 */
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

/**
 * @brief double 限幅并转换为 int16_t
 */
int16_t clamp_to_i16(double value) {
    const double min_v = static_cast<double>(std::numeric_limits<int16_t>::min());
    const double max_v = static_cast<double>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(std::llround(std::clamp(value, min_v, max_v)));
}

/**
 * @brief m/s 转 mm/s，用于 PI_CONTROL
 */
int16_t m_s_to_mm_s_i16(double value) {
    return clamp_to_i16(value * 1000.0);
}

/**
 * @brief rad/s 转 mrad/s，用于 PI_CONTROL 的 wz
 */
int16_t rad_s_to_mrad_s_i16(double value) {
    return clamp_to_i16(value * 1000.0);
}

/**
 * @brief double 限幅并转换为 int32_t
 */
int32_t clamp_to_i32(double value) {
    const double min_v = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double max_v = static_cast<double>(std::numeric_limits<int32_t>::max());
    return static_cast<int32_t>(std::llround(std::clamp(value, min_v, max_v)));
}

/**
 * @brief rad 转 urad，用于 PI_YAW_ACTION 的 target_yaw
 */
int32_t rad_to_urad_i32(double value) {
    return clamp_to_i32(value * 1000000.0);
}

}  // namespace

/**
 * @brief ROS2 节点：MCU 通信桥接
 *
 * 节点面向 Atlas/navigation_system：
 * 1. 读取 MCU_IMU / MCU_ODOM / MCU_ARM_STATE，发布 /imu、/odom、机械臂关节角与末端位置，并可发布 odom -> base_footprint TF
 * 2. 订阅 competition_fsm 输出的 /motor_cmd_vel，周期性打包为 PI_CONTROL 下发给 MCU
 * 3. 提供底盘一次性服务：刹车、急停、yaw hold、yaw target
 * 4. 继续维护 Pi 心跳、START_SENSOR_EVENT ACK、串口解析统计
 */
class McuCommBridgeNode : public rclcpp::Node {
public:
    /**
     * @brief 构造并启动节点线程、发布器、订阅器与定时器
     */
    McuCommBridgeNode()
        : Node("mcu_comm_bridge_node"),
        parser_(static_cast<uint16_t>(declare_parameter<int>("max_body_len", 256))) {
        load_parameters();
        create_ros_interfaces();
        open_serial();

        running_.store(true);
        rx_thread_ = std::thread(&McuCommBridgeNode::rx_loop, this);
        dispatch_thread_ = std::thread(&McuCommBridgeNode::dispatch_loop, this);

        const auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_rate_hz_);
        heartbeat_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
            [this]() { send_heartbeat(); });

        const auto stats_period = std::chrono::duration<double>(1.0 / stats_rate_hz_);
        stats_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(stats_period),
            [this]() { print_stats(); });

        const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
            [this]() { control_timer_callback(); });

        RCLCPP_INFO(
            get_logger(),
            "mcu_comm_bridge started: port=%s baudrate=%d odom_topic=%s imu_topic=%s cmd_vel_topic=%s",
            port_.c_str(), baudrate_, odom_topic_.c_str(), imu_topic_.c_str(), cmd_vel_topic_.c_str());
    }

    /**
     * @brief 清理并停止后台线程和串口
     */
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
    // ! ========================= 初 始 化 ========================= ! //

    /**
     * @brief 读取 ROS 参数
     */
    void load_parameters() {
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = declare_parameter<int>("baudrate", 1000000);
        heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 1.0);
        stats_rate_hz_ = declare_parameter<double>("stats_rate_hz", 1.0);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
        auto_ack_start_sensor_event_ = declare_parameter<bool>("auto_ack_start_sensor_event", true);
        log_latest_sample_ = declare_parameter<bool>("log_latest_sample", true);

        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
        arm_joint_state_topic_ = declare_parameter<std::string>("arm_joint_state_topic", "/arm/joint_states");
        arm_fk_topic_ = declare_parameter<std::string>("arm_fk_topic", "/arm/fk_position");
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/motor_cmd_vel");
        brake_service_ = declare_parameter<std::string>("brake_service", "/mcu/set_brake");
        estop_service_ = declare_parameter<std::string>("estop_service", "/mcu/estop");
        yaw_hold_service_ = declare_parameter<std::string>("yaw_hold_service", "/mcu/set_yaw_hold");
        yaw_target_service_ = declare_parameter<std::string>("yaw_target_service", "/mcu/set_yaw_target");

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
        repeat_estop_count_ = declare_parameter<int>("repeat_estop_count", 3);
    }

    /**
     * @brief 创建 ROS 发布器、订阅器和 TF 广播器
     */
    void create_ros_interfaces() {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
        arm_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(arm_joint_state_topic_, rclcpp::QoS(20));
        arm_fk_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(arm_fk_topic_, rclcpp::QoS(20));

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

        estop_srv_ = create_service<::mcu_comm_bridge::srv::Estop>(
            estop_service_,
            [this](const std::shared_ptr<::mcu_comm_bridge::srv::Estop::Request> request,
                std::shared_ptr<::mcu_comm_bridge::srv::Estop::Response> response) {
                    handle_estop(request, response);
            });
    }

    /**
     * @brief 打开并配置串口
     */
    void open_serial() {
        try {
            serial_.open(port_, baudrate_);
            RCLCPP_INFO(get_logger(), "serial opened: %s @ %d", port_.c_str(), baudrate_);
        }
        catch(const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "failed to open serial: %s", e.what());
            throw;
        }
    }

    BridgeStatsSnapshot snapshot_stats() {
        BridgeStatsSnapshot snapshot;
        snapshot.rx_imu = stats_.rx_imu.load();
        snapshot.rx_odom = stats_.rx_odom.load();
        snapshot.rx_arm_state = stats_.rx_arm_state.load();
        snapshot.rx_status = stats_.rx_status.load();
        snapshot.rx_start_sensor_event = stats_.rx_start_sensor_event.load();
        snapshot.rx_mcu_ack = stats_.rx_mcu_ack.load();
        snapshot.rx_fault_event = stats_.rx_fault_event.load();
        snapshot.rx_unknown = stats_.rx_unknown.load();
        snapshot.rx_bad_payload_len = stats_.rx_bad_payload_len.load();
        snapshot.tx_heartbeat = stats_.tx_heartbeat.load();
        snapshot.tx_pi_ack = stats_.tx_pi_ack.load();
        snapshot.tx_control = stats_.tx_control.load();
        snapshot.tx_yaw_action = stats_.tx_yaw_action.load();
        snapshot.tx_estop = stats_.tx_estop.load();
        snapshot.tx_fail = stats_.tx_fail.load();
        snapshot.rx_queue_drop = stats_.rx_queue_drop.load();
        snapshot.rx_queue_peak_depth = stats_.rx_queue_peak_depth.load();
        {
            std::lock_guard<std::mutex> lock(parser_mutex_);
            snapshot.parser = parser_.stats();
        }
        {
            std::lock_guard<std::mutex> lock(rx_queue_mutex_);
            snapshot.rx_queue_depth = rx_queue_.size();
        }
        return snapshot;
    }

    void enqueue_frame(Frame&& frame) {
        bool dropped_oldest = false;
        {
            std::lock_guard<std::mutex> lock(rx_queue_mutex_);
            if(rx_queue_.size() >= RX_QUEUE_CAPACITY) {
                rx_queue_.pop_front();
                stats_.rx_queue_drop++;
                dropped_oldest = true;
            }

            rx_queue_.push_back(std::move(frame));
            const uint64_t depth = rx_queue_.size();
            uint64_t peak = stats_.rx_queue_peak_depth.load();
            while(depth > peak &&
                  !stats_.rx_queue_peak_depth.compare_exchange_weak(peak, depth)) {
            }
        }

        if(dropped_oldest) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "rx queue full, dropping oldest frame");
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

    // ! ========================= 串 口 接 收 ========================= ! //

    /**
     * @brief 接收线程主循环：从串口读取字节并交给解析器
     */
    void rx_loop() {
        std::array<uint8_t, 256> buf{};

        while(rclcpp::ok() && running_.load()) {
            const int n = serial_.read_some(buf.data(), buf.size());
            if(n < 0) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "serial read failed");
                std::this_thread::sleep_for(20ms);
                continue;
            }

            if(n == 0) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            std::vector<Frame> frames;
            frames.reserve(4u);
            {
                std::lock_guard<std::mutex> lock(parser_mutex_);
                for(int i = 0; i < n; ++i) {
                    auto frame = parser_.feed(buf[static_cast<size_t>(i)]);
                    if(frame.has_value()) {
                        frames.push_back(std::move(frame.value()));
                    }
                }
            }

            for(Frame& frame : frames) {
                enqueue_frame(std::move(frame));
            }
        }
    }

    /**
     * @brief 分发已解析帧
     */
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
                    handle_mcu_ack(frame);
                    break;
                case MSG_MCU_FAULT_EVENT:
                    handle_fault_event(frame);
                    break;
                default:
                    stats_.rx_unknown++;
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "unknown MCU msg_id=0x%02X payload_len=%zu", frame.msg_id, frame.payload.size());
                    break;
            }
        }
        catch(const std::exception& e) {
            stats_.rx_bad_payload_len++;
            RCLCPP_WARN(get_logger(), "failed to handle msg_id=0x%02X: %s", frame.msg_id, e.what());
        }
    }

    /**
     * @brief 处理 MCU_IMU 并发布 /imu
     *
     * ROS 话题名按导航需求固定默认 /imu，而不是 /imu/data
     */
    void handle_imu(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_IMU_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

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
            latest_imu_ = sample;
            has_imu_ = true;
        }

        publish_imu(sample);
        stats_.rx_imu++;
    }

    /**
     * @brief 处理 MCU_ODOM 并发布 /odom 和 odom -> base_footprint TF
     */
    void handle_odom(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_ODOM_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

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
            latest_odom_ = odom;
            has_odom_ = true;
        }

        publish_odom(odom);
        stats_.rx_odom++;
    }

    /**
     * @brief 处理 MCU_ARM_STATE 并发布机械臂关节角和末端位置
     */
    void handle_arm_state(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_ARM_STATE_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

        ArmState arm_state;
        arm_state.stamp_ms = read_u32_le(frame.payload, 0);
        arm_state.status_flags = read_u16_le(frame.payload, 4);
        arm_state.sequence_count = read_u16_le(frame.payload, 6);
        arm_state.q0_urad = read_i32_le(frame.payload, 8);
        arm_state.q1_urad = read_i32_le(frame.payload, 12);
        arm_state.q2_urad = read_i32_le(frame.payload, 16);
        arm_state.q3_urad = read_i32_le(frame.payload, 20);
        arm_state.q4_urad = read_i32_le(frame.payload, 24);
        arm_state.x_mm = read_i32_le(frame.payload, 28);
        arm_state.y_mm = read_i32_le(frame.payload, 32);
        arm_state.z_mm = read_i32_le(frame.payload, 36);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_arm_state_ = arm_state;
            has_arm_state_ = true;
        }

        publish_arm_state(arm_state);
        stats_.rx_arm_state++;
    }

    /**
     * @brief 处理 MCU_STATUS 并更新状态缓存
     */
    void handle_status(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_STATUS_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

        McuStatus status;
        status.stamp_ms = read_u32_le(frame.payload, 0);
        status.app_state = frame.payload[4];
        status.manual_mode = frame.payload[5];
        status.ready_flags = frame.payload[6];
        status.online_flags = frame.payload[7];
        status.fault_source = frame.payload[8];
        status.fault_level = frame.payload[9];
        status.fault_code = read_i16_le(frame.payload, 10);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_status_ = status;
            has_status_ = true;
        }

        stats_.rx_status++;
    }

    /**
     * @brief 处理传感器启动事件并根据配置自动应答
     */
    void handle_start_sensor_event(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_START_SENSOR_EVENT_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

        StartSensorEvent event;
        event.stamp_ms = read_u32_le(frame.payload, 0);
        event.sensor_id = frame.payload[4];
        event.event_type = frame.payload[5];
        event.event_value = read_u16_le(frame.payload, 6);

        stats_.rx_start_sensor_event++;

        RCLCPP_INFO(
            get_logger(),
            "start sensor event: stamp=%u sensor=%u type=%u value=%u seq=%u flags=0x%02X",
            event.stamp_ms, event.sensor_id, event.event_type, event.event_value, frame.seq, frame.flags);

        if(auto_ack_start_sensor_event_ && ((frame.flags & FLAG_NEED_ACK) != 0u)) {
            send_pi_ack(frame.msg_id, frame.seq, 0u);
        }
    }

    /**
     * @brief 处理 MCU ACK
     */
    void handle_mcu_ack(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_ACK_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

        const uint8_t ack_msg_id = frame.payload[0];
        const uint8_t ack_seq = frame.payload[1];
        const uint16_t code = read_u16_le(frame.payload, 2);
        stats_.rx_mcu_ack++;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "MCU_ACK: ack_msg_id=0x%02X ack_seq=%u code=%u", ack_msg_id, ack_seq, code);
    }

    /**
     * @brief 处理 MCU 故障事件
     */
    void handle_fault_event(const Frame& frame) {
        if(frame.payload.size() != PAYLOAD_MCU_FAULT_EVENT_LEN) {
            stats_.rx_bad_payload_len++;
            return;
        }

        FaultEvent event;
        event.stamp_ms = read_u32_le(frame.payload, 0);
        event.fault_source = frame.payload[4];
        event.fault_level = frame.payload[5];
        event.fault_code = read_i16_le(frame.payload, 6);
        stats_.rx_fault_event++;

        RCLCPP_WARN(
            get_logger(),
            "fault event: stamp=%u source=%u level=%u code=%d",
            event.stamp_ms, event.fault_source, event.fault_level, event.fault_code);
    }

    // ! ========================= ROS 发 布 ========================= ! //

    /**
     * @brief 将 MCU_IMU 转为 sensor_msgs/Imu 并发布到 /imu
     */
    void publish_imu(const ImuSample& sample) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = imu_frame_id_;

        const double roll = urad_to_rad(sample.roll_urad);
        const double pitch = urad_to_rad(sample.pitch_urad);
        const double yaw = urad_to_rad(sample.yaw_urad);
        msg.orientation = quaternion_from_rpy(roll, pitch, yaw);

        msg.angular_velocity.x = urad_s_to_rad_s(sample.gyro_x_urad_s);
        msg.angular_velocity.y = urad_s_to_rad_s(sample.gyro_y_urad_s);
        msg.angular_velocity.z = urad_s_to_rad_s(sample.gyro_z_urad_s);

        msg.linear_acceleration.x = mm_s2_to_m_s2(sample.acc_x_mm_s2);
        msg.linear_acceleration.y = mm_s2_to_m_s2(sample.acc_y_mm_s2);
        msg.linear_acceleration.z = mm_s2_to_m_s2(sample.acc_z_mm_s2);

        // 协方差先给保守固定值，后续可根据 IMU 标定结果改为参数
        msg.orientation_covariance[0] = 0.05;
        msg.orientation_covariance[4] = 0.05;
        msg.orientation_covariance[8] = 0.10;
        msg.angular_velocity_covariance[0] = 0.02;
        msg.angular_velocity_covariance[4] = 0.02;
        msg.angular_velocity_covariance[8] = 0.02;
        msg.linear_acceleration_covariance[0] = 0.20;
        msg.linear_acceleration_covariance[4] = 0.20;
        msg.linear_acceleration_covariance[8] = 0.20;

        imu_pub_->publish(msg);
    }

    /**
     * @brief 将 MCU_ODOM 转为 nav_msgs/Odometry 并发布 /odom
     *
     * frame_id 使用 odom，child_frame_id 使用 base_footprint，符合当前导航 TF 链路
     */
    void publish_odom(const OdomState& odom) {
        const rclcpp::Time stamp = now();
        const double x = mm_to_m(odom.x_mm);
        const double y = mm_to_m(odom.y_mm);
        const double yaw = urad_to_rad(odom.yaw_urad);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_id_;
        msg.child_frame_id = base_frame_id_;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = quaternion_from_rpy(0.0, 0.0, yaw);

        msg.twist.twist.linear.x = mm_s_to_m_s(odom.vx_mm_s);
        msg.twist.twist.linear.y = mm_s_to_m_s(odom.vy_mm_s);
        msg.twist.twist.angular.z = urad_s_to_rad_s(odom.wz_urad_s);

        // 二维底盘里程计：x/y/yaw 和 vx/vy/wz 为主要有效量
        msg.pose.covariance[0] = 0.02;
        msg.pose.covariance[7] = 0.02;
        msg.pose.covariance[35] = 0.05;
        msg.twist.covariance[0] = 0.05;
        msg.twist.covariance[7] = 0.05;
        msg.twist.covariance[35] = 0.05;

        odom_pub_->publish(msg);

        if(publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    /**
     * @brief 将 MCU_ARM_STATE 转为 ROS 标准消息并发布
     *
     * q0~q4 发布为 sensor_msgs/JointState，末端 xyz 发布为 geometry_msgs/PointStamped
     * 如果状态标志显示字段无效，则不发布对应消息，但仍缓存原始帧
     */
    void publish_arm_state(const ArmState& arm_state) {
        const rclcpp::Time stamp = now();

        if((arm_state.status_flags & ARM_STATE_FLAG_JOINT_VALID) != 0u) {
            sensor_msgs::msg::JointState joint_msg;
            joint_msg.header.stamp = stamp;
            joint_msg.header.frame_id = arm_frame_id_;
            joint_msg.name = { "q0", "q1", "q2", "q3", "q4" };
            joint_msg.position = {
                urad_to_rad(arm_state.q0_urad),
                urad_to_rad(arm_state.q1_urad),
                urad_to_rad(arm_state.q2_urad),
                urad_to_rad(arm_state.q3_urad),
                urad_to_rad(arm_state.q4_urad),
            };
            arm_joint_state_pub_->publish(joint_msg);
        }

        if((arm_state.status_flags & ARM_STATE_FLAG_FK_VALID) != 0u) {
            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header.stamp = stamp;
            point_msg.header.frame_id = arm_frame_id_;
            point_msg.point.x = mm_to_m(arm_state.x_mm);
            point_msg.point.y = mm_to_m(arm_state.y_mm);
            point_msg.point.z = mm_to_m(arm_state.z_mm);
            arm_fk_pub_->publish(point_msg);
        }
    }

    // ! ========================= ROS 订 阅 与 服 务 ========================= ! //

    /**
     * @brief 缓存 /motor_cmd_vel
     *
     * 周期性底盘速度命令使用 topic，回调只缓存最新速度，不直接高频写串口
     */
    void handle_cmd_vel(const geometry_msgs::msg::Twist& msg) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        cmd_vel_cache_.twist = msg;
        cmd_vel_cache_.last_update = SteadyClock::now();
        cmd_vel_cache_.has_cmd = true;
        cmd_vel_cache_.timeout_brake_sent = false;
    }

    /**
     * @brief 设置底盘刹车锁存
     *
     * 刹车属于一次性请求入口，使用 service 修改本节点的 brake latch
     * 当 latch 为 true 时，控制定时器会持续发送零速 + brake_request 的 PI_CONTROL
     */
    void handle_set_brake(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            {
                std::lock_guard<std::mutex> lock(control_mutex_);
                brake_request_ = request->data;
            }

            if(request->data) {
                geometry_msgs::msg::Twist zero;
                const bool sent = send_pi_control(zero, true);
                response->success = sent;
                response->message = sent ? "brake latch enabled and brake frame sent" : "failed to send brake frame";
            }
            else {
                response->success = true;
                response->message = "brake latch disabled";
            }
    }

    /**
     * @brief 开关 yaw hold
     *
     * yaw hold 是一次性状态切换，使用 service 直接下发 PI_YAW_ACTION
     */
    void handle_set_yaw_hold(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const uint8_t action = request->data ? PI_YAW_ACTION_HOLD_ENABLE : PI_YAW_ACTION_HOLD_DISABLE;
        const bool sent = send_yaw_action(action, 0.0);
        response->success = sent;
        response->message = sent ? (request->data ? "yaw hold enabled" : "yaw hold disabled") : "failed to send yaw hold action";
    }

    /**
     * @brief 设置目标 yaw
     *
     * 目标 yaw 单位为 rad，协议中转换为 urad 后通过 PI_YAW_ACTION 下发
     */
    void handle_set_yaw_target(
        const std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::SetYawTarget::Response> response) {
        const bool sent = send_yaw_action(PI_YAW_ACTION_TARGET_SET, request->yaw_rad);
        response->success = sent;
        response->message = sent ? "yaw target sent" : "failed to send yaw target";
    }

    /**
     * @brief 发送急停
     *
     * 急停是一次性安全事件，使用 service 触发后立即重复下发 PI_ESTOP
     */
    void handle_estop(
        const std::shared_ptr<::mcu_comm_bridge::srv::Estop::Request> request,
        std::shared_ptr<::mcu_comm_bridge::srv::Estop::Response> response) {
        const uint8_t reason = request->reason;
        const int repeat = std::max(1, repeat_estop_count_);
        int sent_count = 0;
        for(int i = 0; i < repeat; ++i) {
            if(send_estop(reason)) {
                ++sent_count;
            }
        }
        response->success = sent_count > 0;
        response->message = response->success ? "estop sent" : "failed to send estop";
    }

    /**
     * @brief 控制定时器：把缓存的导航速度命令转换成 PI_CONTROL
     */
    void control_timer_callback() {
        geometry_msgs::msg::Twist cmd;
        bool send_control = false;
        bool brake = false;

        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if(brake_request_) {
                send_control = true;
                brake = true;
            }
            else if(cmd_vel_cache_.has_cmd) {
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    SteadyClock::now() - cmd_vel_cache_.last_update).count();
                if(elapsed_ms <= cmd_vel_timeout_ms_) {
                    cmd = cmd_vel_cache_.twist;
                    send_control = true;
                    cmd_vel_cache_.timeout_brake_sent = false;
                }
                else if(send_brake_on_cmd_timeout_ && !cmd_vel_cache_.timeout_brake_sent) {
                    send_control = true;
                    brake = true;
                    cmd_vel_cache_.timeout_brake_sent = true;
                }
            }
        }

        if(!send_control) {
            return;
        }

        if(brake) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
        }
        else {
            cmd.linear.x = std::clamp(cmd.linear.x, -max_vx_m_s_, max_vx_m_s_);
            cmd.linear.y = std::clamp(cmd.linear.y, -max_vy_m_s_, max_vy_m_s_);
            cmd.angular.z = std::clamp(cmd.angular.z, -max_wz_rad_s_, max_wz_rad_s_);
        }

        send_pi_control(cmd, brake);
    }

    // ! ========================= 协 议 发 送 ========================= ! //

    /**
     * @brief 发送 PI_HEARTBEAT 给 MCU
     */
    void send_heartbeat() {
        if(send_frame(MSG_PI_HEARTBEAT, 0u, {})) {
            stats_.tx_heartbeat++;
        }
    }

    /**
     * @brief 发送 PI_CONTROL
     *
     * 本节点只下发底盘连续控制，不在该帧中填充机械臂关节目标
     */
    bool send_pi_control(const geometry_msgs::msg::Twist& cmd, bool brake_request) {
        std::vector<uint8_t> payload(PAYLOAD_PI_CONTROL_LEN, 0u);
        const uint32_t stamp_ms = ros_now_ms_u32();

        write_u32_le(payload, 0, stamp_ms);
        payload[4] = PI_CONTROL_MASK_CHASSIS_VALID;
        if(brake_request) {
            payload[4] |= PI_CONTROL_MASK_BRAKE_REQUEST;
        }
        payload[5] = PI_ARM_MODE_NONE;
        write_u16_le(payload, 6, 0u);
        write_i16_le(payload, 8, m_s_to_mm_s_i16(cmd.linear.x));
        write_i16_le(payload, 10, m_s_to_mm_s_i16(cmd.linear.y));
        write_i16_le(payload, 12, rad_s_to_mrad_s_i16(cmd.angular.z));
        // q0~q4、arm_speed、reserved2 保持 0，表示本节点不控制机械臂

        if(send_frame(MSG_PI_CONTROL, 0u, payload)) {
            stats_.tx_control++;
            return true;
        }
        return false;
    }

    /**
     * @brief 发送 PI_YAW_ACTION
     */
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

    /**
     * @brief 发送 PI_ESTOP
     */
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

    /**
     * @brief 发送 PI_ACK 给 MCU
     */
    void send_pi_ack(uint8_t ack_msg_id, uint8_t ack_seq, uint16_t code) {
        std::vector<uint8_t> payload(PAYLOAD_PI_ACK_LEN, 0u);
        payload[0] = ack_msg_id;
        payload[1] = ack_seq;
        write_u16_le(payload, 2, code);
        if(send_frame(MSG_PI_ACK, 0u, payload)) {
            stats_.tx_pi_ack++;
        }
    }

    /**
     * @brief 打包并发送协议帧
     */
    bool send_frame(uint8_t msg_id, uint8_t flags, const std::vector<uint8_t>& payload) {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        const uint8_t seq = next_tx_seq();
        const auto frame = pack_frame(msg_id, seq, flags, payload);
        if(!serial_.write_all(frame)) {
            stats_.tx_fail++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "serial write failed");
            return false;
        }
        return true;
    }

    /**
     * @brief 获取并递增发送序列号
     */
    uint8_t next_tx_seq() {
        return tx_seq_.fetch_add(1);
    }

    /**
     * @brief 获取当前 ROS 时间并截断为协议 uint32 ms
     */
    uint32_t ros_now_ms_u32() {
        const int64_t ms = now().nanoseconds() / 1000000LL;
        return static_cast<uint32_t>(ms & 0xFFFFFFFFLL);
    }

    // ! ========================= 统 计 日 志 ========================= ! //

    /**
     * @brief 周期性打印统计数据和最近样本
     */
    void print_stats() {
        const BridgeStatsSnapshot snapshot = snapshot_stats();
        const SteadyClock::time_point now_tp = SteadyClock::now();
        double elapsed_sec = 0.0;

        if(last_stats_time_.time_since_epoch().count() != 0) {
            elapsed_sec = std::chrono::duration<double>(now_tp - last_stats_time_).count();
        }
        last_stats_time_ = now_tp;

        const double imu_hz = elapsed_sec > 0.0 ? static_cast<double>(snapshot.rx_imu - last_stats_snapshot_.rx_imu) / elapsed_sec : 0.0;
        const double odom_hz = elapsed_sec > 0.0 ? static_cast<double>(snapshot.rx_odom - last_stats_snapshot_.rx_odom) / elapsed_sec : 0.0;
        const double arm_hz = elapsed_sec > 0.0 ? static_cast<double>(snapshot.rx_arm_state - last_stats_snapshot_.rx_arm_state) / elapsed_sec : 0.0;
        const double status_hz = elapsed_sec > 0.0 ? static_cast<double>(snapshot.rx_status - last_stats_snapshot_.rx_status) / elapsed_sec : 0.0;
        const double crc_err_per_sec = elapsed_sec > 0.0 ? static_cast<double>(snapshot.parser.crc_error - last_stats_snapshot_.parser.crc_error) / elapsed_sec : 0.0;
        const double resync_per_sec = elapsed_sec > 0.0 ? static_cast<double>(snapshot.parser.resync - last_stats_snapshot_.parser.resync) / elapsed_sec : 0.0;
        const double queue_drop_per_sec = elapsed_sec > 0.0 ? static_cast<double>(snapshot.rx_queue_drop - last_stats_snapshot_.rx_queue_drop) / elapsed_sec : 0.0;

        RCLCPP_INFO(
            get_logger(),
            "stats: imu=%" PRIu64 " odom=%" PRIu64 " arm=%" PRIu64 " status=%" PRIu64
            " start_evt=%" PRIu64 " ack_rx=%" PRIu64 " fault=%" PRIu64
            " unknown=%" PRIu64 " bad_len=%" PRIu64 " tx_hb=%" PRIu64
            " tx_ack=%" PRIu64 " tx_ctrl=%" PRIu64 " tx_yaw=%" PRIu64 " tx_estop=%" PRIu64 " tx_fail=%" PRIu64
            " queue_depth=%" PRIu64 " queue_peak=%" PRIu64 " queue_drop=%" PRIu64
            " parser_frames=%" PRIu64 " rx_bytes=%" PRIu64 " crc_err=%" PRIu64 " len_err=%" PRIu64 " ver_err=%" PRIu64 " resync=%" PRIu64
            " imu_hz=%.1f odom_hz=%.1f arm_hz=%.1f status_hz=%.1f crc_err_s=%.1f resync_s=%.1f queue_drop_s=%.1f",
            snapshot.rx_imu, snapshot.rx_odom, snapshot.rx_arm_state, snapshot.rx_status,
            snapshot.rx_start_sensor_event, snapshot.rx_mcu_ack, snapshot.rx_fault_event,
            snapshot.rx_unknown, snapshot.rx_bad_payload_len, snapshot.tx_heartbeat,
            snapshot.tx_pi_ack, snapshot.tx_control, snapshot.tx_yaw_action, snapshot.tx_estop, snapshot.tx_fail,
            snapshot.rx_queue_depth, snapshot.rx_queue_peak_depth, snapshot.rx_queue_drop,
            snapshot.parser.rx_frames, snapshot.parser.rx_bytes, snapshot.parser.crc_error, snapshot.parser.len_error, snapshot.parser.version_error, snapshot.parser.resync,
            imu_hz, odom_hz, arm_hz, status_hz, crc_err_per_sec, resync_per_sec, queue_drop_per_sec);

        last_stats_snapshot_ = snapshot;

        if(!log_latest_sample_) {
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        if(has_imu_) {
            RCLCPP_INFO(
                get_logger(),
                "latest imu: stamp=%u acc=[%.3f %.3f %.3f]m/s2 gyro=[%.3f %.3f %.3f]rad/s rpy=[%.3f %.3f %.3f]rad flags=0x%04X seq=%u",
                latest_imu_.stamp_ms,
                mm_s2_to_m_s2(latest_imu_.acc_x_mm_s2),
                mm_s2_to_m_s2(latest_imu_.acc_y_mm_s2),
                mm_s2_to_m_s2(latest_imu_.acc_z_mm_s2),
                urad_s_to_rad_s(latest_imu_.gyro_x_urad_s),
                urad_s_to_rad_s(latest_imu_.gyro_y_urad_s),
                urad_s_to_rad_s(latest_imu_.gyro_z_urad_s),
                urad_to_rad(latest_imu_.roll_urad),
                urad_to_rad(latest_imu_.pitch_urad),
                urad_to_rad(latest_imu_.yaw_urad),
                latest_imu_.status_flags,
                latest_imu_.sequence_count);
        }

        if(has_odom_) {
            RCLCPP_INFO(
                get_logger(),
                "latest odom: stamp=%u pose=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] flags=0x%04X reset=%u",
                latest_odom_.stamp_ms,
                mm_to_m(latest_odom_.x_mm),
                mm_to_m(latest_odom_.y_mm),
                urad_to_rad(latest_odom_.yaw_urad),
                mm_s_to_m_s(latest_odom_.vx_mm_s),
                mm_s_to_m_s(latest_odom_.vy_mm_s),
                urad_s_to_rad_s(latest_odom_.wz_urad_s),
                latest_odom_.status_flags,
                latest_odom_.reset_counter);
        }

        if(has_arm_state_) {
            RCLCPP_INFO(
                get_logger(),
                "latest arm: stamp=%u q=[%.3f %.3f %.3f %.3f %.3f] xyz=[%.3f %.3f %.3f] flags=0x%04X seq=%u",
                latest_arm_state_.stamp_ms,
                urad_to_rad(latest_arm_state_.q0_urad),
                urad_to_rad(latest_arm_state_.q1_urad),
                urad_to_rad(latest_arm_state_.q2_urad),
                urad_to_rad(latest_arm_state_.q3_urad),
                urad_to_rad(latest_arm_state_.q4_urad),
                mm_to_m(latest_arm_state_.x_mm),
                mm_to_m(latest_arm_state_.y_mm),
                mm_to_m(latest_arm_state_.z_mm),
                latest_arm_state_.status_flags,
                latest_arm_state_.sequence_count);
        }

        if(has_status_) {
            RCLCPP_INFO(
                get_logger(),
                "latest status: stamp=%u app=%u manual=%u ready=0x%02X online=0x%02X fault_src=%u fault_level=%u fault_code=%d",
                latest_status_.stamp_ms,
                latest_status_.app_state,
                latest_status_.manual_mode,
                latest_status_.ready_flags,
                latest_status_.online_flags,
                latest_status_.fault_source,
                latest_status_.fault_level,
                latest_status_.fault_code);
        }
    }

    // ! ========================= 成 员 变 量 ========================= ! //

    std::string port_;
    int baudrate_ = 1000000;
    double heartbeat_rate_hz_ = 1.0;
    double stats_rate_hz_ = 1.0;
    double control_rate_hz_ = 50.0;
    bool auto_ack_start_sensor_event_ = true;
    bool log_latest_sample_ = true;

    std::string odom_topic_ = "/odom";
    std::string imu_topic_ = "/imu";
    std::string arm_joint_state_topic_ = "/arm/joint_states";
    std::string arm_fk_topic_ = "/arm/fk_position";
    std::string cmd_vel_topic_ = "/motor_cmd_vel";
    std::string brake_service_ = "/mcu/set_brake";
    std::string estop_service_ = "/mcu/estop";
    std::string yaw_hold_service_ = "/mcu/set_yaw_hold";
    std::string yaw_target_service_ = "/mcu/set_yaw_target";
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
    int repeat_estop_count_ = 3;

    SerialPort serial_;
    BinaryFrameParser parser_;
    BridgeStats stats_;
    BridgeStatsSnapshot last_stats_snapshot_;
    SteadyClock::time_point last_stats_time_{};

    std::atomic<bool> running_{ false };
    std::thread rx_thread_;
    std::thread dispatch_thread_;
    std::atomic<uint8_t> tx_seq_{ 0 };

    std::mutex tx_mutex_;
    std::mutex parser_mutex_;
    std::mutex data_mutex_;
    std::mutex rx_queue_mutex_;
    std::condition_variable rx_queue_cv_;
    std::deque<Frame> rx_queue_;
    bool has_imu_ = false;
    bool has_odom_ = false;
    bool has_arm_state_ = false;
    bool has_status_ = false;
    ImuSample latest_imu_;
    OdomState latest_odom_;
    ArmState latest_arm_state_;
    McuStatus latest_status_;

    std::mutex control_mutex_;
    CmdVelCache cmd_vel_cache_;
    bool brake_request_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr arm_fk_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr yaw_hold_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::SetYawTarget>::SharedPtr yaw_target_srv_;
    rclcpp::Service<::mcu_comm_bridge::srv::Estop>::SharedPtr estop_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mcu_comm_bridge

/**
 * @brief 程序入口，初始化 rclcpp 并启动节点
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcu_comm_bridge::McuCommBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
