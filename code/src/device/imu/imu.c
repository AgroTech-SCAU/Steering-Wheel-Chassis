#include "imu.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 当前绑定的具体 IMU 实例
 */
const ImuInterface* imu_instance = 0;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 绑定具体 IMU 实例
 */
ImuStatus imu_set_instance(const ImuInterface* instance) {
    if(instance == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    imu_instance = instance;
    return IMU_STATUS_OK;
}

/**
 * @brief 初始化当前绑定的 IMU 实例
 */
ImuStatus imu_init(void) {
    if(imu_instance == 0 || imu_instance->init == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    return imu_instance->init();
}

/**
 * @brief 更新当前绑定 IMU 的数据缓存
 */
ImuStatus imu_update(void) {
    if(imu_instance == 0 || imu_instance->update == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    return imu_instance->update();
}

/**
 * @brief 获取最近一次缓存的加速度
 */
ImuAcc imu_get_acc(void) {
    ImuAcc acc = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_acc == 0) {
        return acc;
    }

    return imu_instance->get_acc();
}

/**
 * @brief 获取最近一次缓存的角速度
 */
ImuGyro imu_get_gyro(void) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_gyro == 0) {
        return gyro;
    }

    return imu_instance->get_gyro();
}

/**
 * @brief 获取最近一次缓存的姿态角
 */
ImuAngle imu_get_angle(void) {
    ImuAngle angle = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_angle == 0) {
        return angle;
    }

    return imu_instance->get_angle();
}

/**
 * @brief 将状态码转换为常量字符串
 */
const char* imu_status_str(ImuStatus status) {
    if(imu_instance != 0 && imu_instance->status_str != 0) {
        return imu_instance->status_str(status);
    }

    switch(status) {
#define X(name, value) case IMU_STATUS_##name: return #name;
        IMU_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}
