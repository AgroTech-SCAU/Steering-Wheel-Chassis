#ifndef _remote_h_
#define _remote_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 遥控器解析后的底盘控制指令
 *
 * 该结构是遥控服务对外暴露的速度命令快照；
 * online 字段表示该命令是否来自有效遥控链路
 *
 * @param vx 底盘 x 方向目标线速度，单位 m/s
 * @param vy 底盘 y 方向目标线速度，单位 m/s
 * @param wz 底盘 z 轴目标角速度，单位 rad/s
 * @param online 遥控器链路是否在线
 */
typedef struct {
    float vx;
    float vy;
    float wz;
    bool online;
} RemoteCommand;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化遥控服务内部状态
 *
 * 函数会清空最近一次遥控命令；
 * 接收机底层初始化由装配层单独负责
 */
void remote_init(void);

/**
 * @brief 执行一次遥控服务轮询
 *
 * 该函数会读取 i.BUS 遥控器数据，完成通道映射、失联保护和刹车逻辑，
 * 并将结果写入底盘速度指令
 */
void remote_process(void);

/**
 * @brief 获取最近一次遥控服务输出的控制指令
 *
 * 函数会复制最近一次遥控处理结果；
 * 返回值表示该结果是否处于在线状态
 *
 * @param out 输出控制指令缓冲区
 * @return bool `true` 表示当前遥控链路在线，`false` 表示离线
 */
bool remote_get_command(RemoteCommand* out);

#endif
