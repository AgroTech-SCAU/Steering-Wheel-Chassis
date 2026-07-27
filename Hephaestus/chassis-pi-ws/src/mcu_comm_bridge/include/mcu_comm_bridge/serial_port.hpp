#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>
#include <vector>

namespace mcu_comm_bridge {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void open(const std::string& device, int baudrate);
    void close();
    bool is_open() const;

    int read_some(uint8_t* data, size_t size);
    int last_read_errno() const;
    bool write_all(const std::vector<uint8_t>& data);

    bool verify_configuration(std::string* reason = nullptr) const;
    void reapply_configuration();

private:
    static speed_t baud_to_speed(int baudrate);

    void configure_or_throw();
    void close_unlocked();
    bool read_termios(termios* tty) const;
    bool matches_expected_termios(const termios& tty, std::string* reason) const;

    int fd_ = -1;
    int baudrate_ = 0;
    speed_t expected_speed_ = 0;
    std::string device_;
    mutable int last_read_errno_ = 0;
    mutable std::mutex state_mutex_;
    mutable std::mutex write_mutex_;
};

}  // namespace mcu_comm_bridge
