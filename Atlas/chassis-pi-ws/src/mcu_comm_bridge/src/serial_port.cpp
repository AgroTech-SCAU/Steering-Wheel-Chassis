#include "mcu_comm_bridge/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace mcu_comm_bridge {

/**
 * @brief 析构函数，关闭串口
 */
SerialPort::~SerialPort() {
    close();
}

/**
 * @brief 打开串口设备并配置 termios
 *
 * @param device 串口设备路径
 * @param baudrate 波特率
 * @throws std::system_error 打开或设置串口失败
 */
void SerialPort::open(const std::string& device, int baudrate) {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "open serial device failed: " + device);
    }

    termios tty{};
    if(tcgetattr(fd_, &tty) != 0) {
        const int saved_errno = errno;
        close();
        throw std::system_error(saved_errno, std::generic_category(), "tcgetattr failed");
    }

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    const speed_t speed = baud_to_speed(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if(tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const int saved_errno = errno;
        close();
        throw std::system_error(saved_errno, std::generic_category(), "tcsetattr failed");
    }

    tcflush(fd_, TCIOFLUSH);
}

/**
 * @brief 关闭串口（如果已打开）
 */
void SerialPort::close() {
    if(fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

/**
 * @brief 返回串口是否已打开
 *
 * @return true 已打开
 * @return false 未打开
 */
bool SerialPort::is_open() const {
    return fd_ >= 0;
}

/**
 * @brief 从串口读取数据（非阻塞）
 *
 * 如果没有数据返回 0；若发生错误返回 -1
 *
 * @param data 指向接收缓冲区
 * @param size 缓冲区大小（最大读取字节数）
 * @return int 实际读取字节数，0 表示无数据，-1 表示错误
 */
int SerialPort::read_some(uint8_t* data, size_t size) {
    if(fd_ < 0) {
        return -1;
    }

    const ssize_t n = ::read(fd_, data, size);
    if(n < 0) {
        if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return 0;
        }
        return -1;
    }
    return static_cast<int>(n);
}

/**
 * @brief 将整个缓冲区写入串口直到写完或遇到不可恢复错误
 *
 * 本函数对写操作使用互斥锁以保证线程安全
 *
 * @param data 要写入的数据
 * @return true 写入成功
 * @return false 写入失败
 */
bool SerialPort::write_all(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(write_mutex_);

    if(fd_ < 0) {
        return false;
    }

    size_t written = 0;
    while(written < data.size()) {
        const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
        if(n < 0) {
            if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        if(n == 0) {
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

/**
 * @brief 将整数波特率映射为 termios 的 speed_t 值
 *
 * @param baudrate 波特率，例如 115200
 * @return speed_t 对应的 termios 常量
 * @throws std::invalid_argument 不支持的波特率
 */
speed_t SerialPort::baud_to_speed(int baudrate) {
    switch(baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
        default:
            throw std::invalid_argument("unsupported baudrate: " + std::to_string(baudrate));
    }
}

}  // namespace mcu_comm_bridge
