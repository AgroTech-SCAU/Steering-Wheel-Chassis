#include "mcu_comm_bridge/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/ioctl.h>
#include <unistd.h>

namespace mcu_comm_bridge {
namespace {

bool raw_mode_matches(const termios& tty) {
    return (tty.c_iflag & static_cast<tcflag_t>(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF | IXANY)) == 0 &&
           (tty.c_oflag & static_cast<tcflag_t>(OPOST)) == 0 &&
           (tty.c_lflag & static_cast<tcflag_t>(ECHO | ECHONL | ICANON | IEXTEN | ISIG)) == 0;
}

}  // namespace

SerialPort::~SerialPort() {
    close();
}

void SerialPort::open(const std::string& device, int baudrate) {
    close();

    std::lock_guard<std::mutex> lock(state_mutex_);

    try {
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(fd_ < 0) {
            throw std::system_error(errno, std::generic_category(), "open serial device failed: " + device);
        }

        if(::ioctl(fd_, TIOCEXCL) != 0) {
            throw std::system_error(errno, std::generic_category(), "TIOCEXCL failed for serial device: " + device);
        }

        device_ = device;
        baudrate_ = baudrate;
        expected_speed_ = baud_to_speed(baudrate);
        configure_or_throw();

        std::string reason;
        termios verify{};
        if(!read_termios(&verify)) {
            throw std::system_error(errno, std::generic_category(), "tcgetattr verify failed for " + device_);
        }
        if(!matches_expected_termios(verify, &reason)) {
            throw std::runtime_error("serial termios verify failed for " + device + ": " + reason);
        }
    }
    catch(...) {
        close_unlocked();
        throw;
    }
}

void SerialPort::close() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    close_unlocked();
}

void SerialPort::close_unlocked() {
    if(fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    last_read_errno_ = 0;
}

bool SerialPort::is_open() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return fd_ >= 0;
}

int SerialPort::read_some(uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if(fd_ < 0) {
        last_read_errno_ = EBADF;
        return -1;
    }

    const ssize_t n = ::read(fd_, data, size);
    if(n < 0) {
        last_read_errno_ = errno;
        if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return 0;
        }
        return -1;
    }

    last_read_errno_ = 0;
    return static_cast<int>(n);
}

int SerialPort::last_read_errno() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_read_errno_;
}

bool SerialPort::write_all(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> write_lock(write_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    if(fd_ < 0) {
        return false;
    }

    size_t written = 0u;
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

bool SerialPort::verify_configuration(std::string* reason) const {
    std::lock_guard<std::mutex> lock(state_mutex_);

    termios tty{};
    if(!read_termios(&tty)) {
        if(reason != nullptr) {
            *reason = std::strerror(errno);
        }
        return false;
    }

    return matches_expected_termios(tty, reason);
}

void SerialPort::reapply_configuration() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    configure_or_throw();
}

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

void SerialPort::configure_or_throw() {
    termios tty{};
    if(!read_termios(&tty)) {
        throw std::system_error(errno, std::generic_category(), "tcgetattr failed for " + device_);
    }

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARODD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if(::cfsetispeed(&tty, expected_speed_) != 0) {
        throw std::system_error(errno, std::generic_category(), "cfsetispeed failed for " + device_);
    }
    if(::cfsetospeed(&tty, expected_speed_) != 0) {
        throw std::system_error(errno, std::generic_category(), "cfsetospeed failed for " + device_);
    }
    if(::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcsetattr failed for " + device_);
    }
    if(::tcflush(fd_, TCIOFLUSH) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcflush failed for " + device_);
    }
}

bool SerialPort::read_termios(termios* tty) const {
    if(fd_ < 0) {
        errno = EBADF;
        return false;
    }
    if(::tcgetattr(fd_, tty) != 0) {
        return false;
    }
    return true;
}

bool SerialPort::matches_expected_termios(const termios& tty, std::string* reason) const {
    const auto fail = [reason](const std::string& text) {
        if(reason != nullptr) {
            *reason = text;
        }
        return false;
    };

    if(::cfgetispeed(&tty) != expected_speed_) {
        return fail("input baudrate mismatch");
    }
    if(::cfgetospeed(&tty) != expected_speed_) {
        return fail("output baudrate mismatch");
    }
    if((tty.c_cflag & CSIZE) != CS8) {
        return fail("character size is not CS8");
    }
    if((tty.c_cflag & PARENB) != 0u) {
        return fail("parity is enabled");
    }
    if((tty.c_cflag & CSTOPB) != 0u) {
        return fail("stop bits are not 1");
    }
    if((tty.c_cflag & CRTSCTS) != 0u) {
        return fail("hardware flow control is enabled");
    }
    if((tty.c_iflag & static_cast<tcflag_t>(IXON | IXOFF | IXANY)) != 0u) {
        return fail("software flow control is enabled");
    }
    if(!raw_mode_matches(tty)) {
        return fail("raw mode is not active");
    }

    return true;
}

}  // namespace mcu_comm_bridge
