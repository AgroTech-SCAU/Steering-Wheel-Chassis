#include "assemble.h"

#include "arm.h"
#include "bus_servo/bus_servo.h"
#include "bus_servo/ft_scs_servo.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_uart.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f))

// ! ========================= 变 量 声 明 ========================= ! //

#define ARM_SERVO_SPEED_RAD_S 3.14f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static inline bool servo_write(const uint8_t* data, uint16_t len) {
    return uart7_write_blocking((const char*)data, len);
}

static int servo_read(uint8_t* data, uint16_t len);
static void servo_flush_rx(void);
static SerialArmStatus build_atlas_arm_model(SerialArmModel* model);
static void tf_identity(SerialArmTransform* T);
static void tf_transl(SerialArmTransform* T, float x, float y, float z);
static void tf_set(SerialArmTransform* T, const float m[4][4]);

static const BusServoPortOps servo_port_ops = {
    .write = servo_write,
    .read = servo_read,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
    .flush_rx = servo_flush_rx,
};

static const FtScsServoConfig servo_config = {
    .ops = &servo_port_ops,
    .timeout_ms = 10,
    .retry_count = 0,
    .endian = SERVO_ENDIAN_LITTLE,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_arm(void) {
    ArmConfig arm_config;
    BusServoStatus servo_ret;
    ArmStatus arm_ret;

    log_info("ARM assemble begin");

    servo_ret = bus_servo_set_instance(&ft_scs_servo_common_instance);
    if(servo_ret != SERVO_STATUS_OK) {
        log_error("ARM bus_servo_set_instance failed: %s", bus_servo_status_str(servo_ret));
        return SYSTEM_STATUS_ERROR;
    }

    servo_ret = bus_servo.init(&servo_config);
    if(servo_ret != SERVO_STATUS_OK) {
        log_error("ARM servo init failed: %s", bus_servo.status_str(servo_ret));
        return SYSTEM_STATUS_ERROR;
    }

    arm_config = arm.default_config();
    arm_config.servo_interface = &ft_scs_servo_common_instance;
    arm_config.stop_servo = ft_scs_servo.disable_torque;
    arm_config.enable_servo = ft_scs_servo.enable_torque;
    arm_config.batch_set_pos_spd = ft_scs_sync_write_pos_spd;
    arm_config.batch_update_feedback = ft_scs_sync_read_feedback;
    arm_config.auto_move_servo_zero = true;
    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        arm_config.servo_id[i] = (uint8_t)(i + 1u);
    }
    if(build_atlas_arm_model(&arm_config.kinematic_model) != SERIAL_ARM_STATUS_SUCCESS) {
        log_error("ARM atlas kinematic model build failed");
        return SYSTEM_STATUS_ERROR;
    }
    arm_config.has_kinematic_model = true;
    arm_config.servo_zero_joints.dof = ARM_DOF;
    arm_config.servo_zero_joints.q[0] = DEG_TO_RAD(180.0f);
    arm_config.servo_zero_joints.q[1] = DEG_TO_RAD(90.0f);
    arm_config.servo_zero_joints.q[2] = DEG_TO_RAD(360.0f);
    arm_config.servo_zero_joints.q[3] = DEG_TO_RAD(180.0f);
    arm_config.servo_zero_joints.q[4] = DEG_TO_RAD(180.0f);

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        servo_ret = ft_scs_servo.write_u8(arm_config.servo_id[i], FT_SCS_SERVO_MODE, 0u);
        if(servo_ret != SERVO_STATUS_OK) {
            log_error("ARM servo mode config failed: %s", bus_servo.status_str(servo_ret));
            return SYSTEM_STATUS_ERROR;
        }
        servo_ret = ft_scs_servo.enable_torque(arm_config.servo_id[i]);
        if(servo_ret != SERVO_STATUS_OK) {
            log_error("ARM servo torque enable failed: %s", bus_servo.status_str(servo_ret));
            return SYSTEM_STATUS_ERROR;
        }
    }

    arm_ret = arm.init(&arm_config);
    if(arm_ret != arm.OK) {
        log_error("ARM service init failed: %s", arm.status_str(arm_ret));
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ARM assemble done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static int servo_read(uint8_t* data, uint16_t len) {
    if(data == 0 || len == 0u) {
        return 0;
    }

    return (HAL_UART_Receive(&huart7, data, len, 1u) == HAL_OK) ? (int)len : 0;
}

static void servo_flush_rx(void) {
    __HAL_UART_CLEAR_PEFLAG(&huart7);
    __HAL_UART_CLEAR_FEFLAG(&huart7);
    __HAL_UART_CLEAR_NEFLAG(&huart7);
    __HAL_UART_CLEAR_OREFLAG(&huart7);
    __HAL_UART_CLEAR_IDLEFLAG(&huart7);
}

static SerialArmStatus build_atlas_arm_model(SerialArmModel* model) {
    SerialArmStatus ret;
    static const float a[ARM_DOF] = {
        0.0000000000000000f,
        0.0276200491203067f,
        0.2167241256700170f,
        0.2002827243995208f,
        0.0451594898594991f,
    };
    static const float d[ARM_DOF] = {
        0.0000000000000000f,
        -0.0162679040568649f,
        -0.0192068569153542f,
        0.0014389528584892f,
        0.0000000000000000f,
    };
    static const float alpha[ARM_DOF] = {
        0.0f,
        M_PI * 0.5f,
        M_PI,
        0.0f,
        M_PI * 0.5f,
    };
    static const float q_offset[ARM_DOF] = {
        -M_PI,
        -M_PI * 0.5f,
        -3.3836013435535577f,
        -2.8616351199480290f,
        -M_PI,
    };
    static const float q_sign[ARM_DOF] = {
        -1.0f,
        -1.0f,
        1.0f,
        1.0f,
        -1.0f,
    };
    static const float tool_T[4][4] = {
        { 0.9999619259637f, -0.0000000000000f, -0.0087262032439f, 0.0000000000000f },
        { 0.0000761495224f, -0.9999619230642f, 0.0087262032439f, 0.0000000000000f },
        { -0.0087258709769f, -0.0087265354984f, -0.9999238504776f, -0.0184685931641f },
        { 0.0000000000000f, 0.0000000000000f, 0.0000000000000f, 1.0000000000000f },
    };

    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;

    ret = serial_arm.model_reset(model, ARM_DOF, SERIAL_ARM_DH_MODIFIED);
    if(ret != SERIAL_ARM_STATUS_SUCCESS)
        return ret;

    tf_transl(&model->base_T, 0.0f, 0.0f, 0.0605000000f);
    tf_set(&model->tool_T, tool_T);

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        ret = serial_arm.model_set_revolute(model, i, 0.0f, d[i], a[i], alpha[i],
                                            q_offset[i], 0.0f, 2.0f * M_PI);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            return ret;

        ret = serial_arm.model_set_joint_sign(model, i, q_sign[i]);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            return ret;
    }

    model->ik.max_iterations = 250.0f;
    model->ik.position_tolerance = 1e-4f;
    model->ik.orientation_tolerance = 2e-3f;
    model->ik.step_gain = 0.45f;
    model->ik.damping = 2e-3f;
    model->ik.numeric_eps = 1e-5f;
    model->ik.position_weight = 1.0f;
    model->ik.orientation_weight = 0.25f;

    return SERIAL_ARM_STATUS_SUCCESS;
}

static void tf_identity(SerialArmTransform* T) {
    for(uint8_t r = 0u; r < 4u; r++) {
        for(uint8_t c = 0u; c < 4u; c++) {
            T->m[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
}

static void tf_transl(SerialArmTransform* T, float x, float y, float z) {
    tf_identity(T);
    T->m[0][3] = x;
    T->m[1][3] = y;
    T->m[2][3] = z;
}

static void tf_set(SerialArmTransform* T, const float m[4][4]) {
    for(uint8_t r = 0u; r < 4u; r++) {
        for(uint8_t c = 0u; c < 4u; c++) {
            T->m[r][c] = m[r][c];
        }
    }
}
