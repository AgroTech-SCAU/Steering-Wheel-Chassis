#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <termios.h>

namespace mcu_comm_bridge {

/**
 * @brief Linux 串口封装类
 *
 * 仅封装 POSIX 串口的打开/关闭/读写，不包含协议处理
 */
class SerialPort {
public:
    SerialPort() = default; /**< 默认构造 */

    /**
     * @brief 析构时自动关闭串口（如果已打开）
     */
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /**
     * @brief 打开串口设备并设置波特率
     * @param device 串口设备路径，例如 `/dev/ttyS0`
     * @param baudrate 波特率，例如 115200
     * @throws std::system_error 打开或配置失败
     */
    void open(const std::string& device, int baudrate);

    /**
     * @brief 关闭串口（若已打开）
     */
    void close();

    /**
     * @brief 检查串口是否已打开
     * @return true 已打开
     */
    bool is_open() const;

    /**
     * @brief 非阻塞读取最多 `size` 字节
     * @param data 指向接收缓冲区
     * @param size 缓冲区大小
     * @return 成功读取的字节数；0 表示当前无数据；-1 表示错误
     */
    int read_some(uint8_t* data, size_t size);

    /**
     * @brief 将整个缓冲区写入串口（阻塞直至写完或发生错误）
     * @param data 要写入的数据
     * @return true 写入成功；false 写入失败
     */
    bool write_all(const std::vector<uint8_t>& data);

private:
    /**
     * @brief 将整数波特率映射为 `termios` 的 `speed_t`
     * @param baudrate 波特率
     * @return 对应的 `speed_t` 值
     * @throws std::invalid_argument 不支持的波特率
     */
    static speed_t baud_to_speed(int baudrate);

    int fd_ = -1;                       /**< POSIX 文件描述符，-1 表示未打开 */
    mutable std::mutex write_mutex_;    /**< 写操作互斥以保证线程安全 */
};

}  // namespace mcu_comm_bridge
