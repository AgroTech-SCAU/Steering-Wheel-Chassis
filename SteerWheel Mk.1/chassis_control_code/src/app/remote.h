#ifndef _APP_REMOTE_H_
#define _APP_REMOTE_H_

/**
 * @file remote.h
 * @brief 遥控应用层接口
 */

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 遥控应用层输出的底盘命令快照
 *
 * 该结构表示应用层根据 i.BUS 输入解析后的最终速度命令；
 * `online` 用于表示本次命令是否来自有效遥控链路
 */
typedef struct {
    float vx;    /**< 底盘 x 方向目标线速度，单位 m/s */
    float vy;    /**< 底盘 y 方向目标线速度，单位 m/s */
    float wz;    /**< 底盘 z 轴目标角速度，单位 rad/s */
    bool online; /**< 遥控链路是否在线 */
} RemoteCommand;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化遥控应用层状态
 */
void remote_init(void);

/**
 * @brief 执行一次遥控应用层轮询
 */
void remote_process(void);

/**
 * @brief 获取最近一次遥控应用层输出的命令
 * @param out 输出命令缓冲区
 * @return bool `true` 表示遥控链路在线
 */
bool remote_get_command(RemoteCommand* out);

#endif
