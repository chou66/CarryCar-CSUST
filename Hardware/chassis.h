#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mecanum_kinematics.h"
#include "hwt101_uart.h"

typedef enum {
    CHASSIS_MODE_VELOCITY,
    CHASSIS_MODE_POSITION
} ChassisMode;

typedef enum {
    CHASSIS_TX_IDLE,
    CHASSIS_TX_WHEEL_1,
    CHASSIS_TX_WHEEL_2,
    CHASSIS_TX_WHEEL_3,
    CHASSIS_TX_WHEEL_4,
    CHASSIS_TX_SYNC,
    CHASSIS_TX_WAIT_POSITION,
    CHASSIS_TX_STOP_1,
    CHASSIS_TX_STOP_2,
    CHASSIS_TX_STOP_3,
    CHASSIS_TX_STOP_4
} ChassisTxState;

typedef struct {
    MecanumConfig config;
    /* 可选：绑定后由底盘命令自动门控 HWT101 停车抗漂移层。 */
    Hwt101 *imu_filter;
    ChassisMode mode;
    float target_x;       /* 速度模式：vx_mps；位置模式：dx_m */
    float target_y;       /* 速度模式：vy_mps；位置模式：dy_m */
    float target_z;       /* 速度模式：wz_radps；位置模式：dtheta_rad */
    float speed_limit;    /* 位置模式的最大轮缘等效线速度 */
    uint8_t acceleration;

    uint32_t last_command_ms;
    uint32_t watchdog_ms;

    /*
     * position_timeout_ms == 0：自动动态超时（推荐）
     * position_timeout_ms != 0：使用用户强制指定的固定超时
     */
    uint32_t position_timeout_ms;
    uint32_t active_position_timeout_ms; /* 本次动作最终采用的超时，便于调试 */
    uint32_t position_start_ms;           /* 同步启动成功的时刻 */
    uint32_t position_feedback_not_before_ms;
    uint32_t position_deadline_ms;
    uint32_t next_status_query_ms;
    uint32_t next_tx_ms;
    uint32_t tx_deadline_ms;          /* CAN 派发/停车序列最晚完成时间 */

    uint8_t status_query_id;
    uint8_t motor_mask;

    bool dirty;
    bool emergency_stop;
    bool velocity_watchdog_armed;     /* 收到速度命令后才启用软件速度看门狗 */
    bool tx_failed;                   /* CAN 连续发送失败并超过超时 */
    bool position_done;
    bool position_failed;

    ChassisMode active_mode;
    ChassisTxState tx_state;
    MecanumWheelCommand active_wheels;
} Chassis;

void chassis_init(Chassis *chassis, const MecanumConfig *config);
void chassis_bind_imu_filter(Chassis *chassis, Hwt101 *imu);
void chassis_set_velocity(Chassis *chassis, float vx_mps, float vy_mps,
                          float wz_radps, uint32_t now_ms);
void chassis_set_position(Chassis *chassis, float dx_m, float dy_m,
                          float dtheta_rad, float speed_mps, uint32_t now_ms);

/* timeout_ms=0 恢复自动动态超时；非 0 时强制固定值。 */
void chassis_set_position_timeout(Chassis *chassis, uint32_t timeout_ms);

void chassis_stop(Chassis *chassis);
bool chassis_position_done(const Chassis *chassis);
bool chassis_position_failed(const Chassis *chassis);
bool chassis_tx_failed(const Chassis *chassis);
bool chassis_busy(const Chassis *chassis);

/* Call from a fast periodic scheduler (1 ms recommended). No delay and no busy wait. */
void chassis_update(Chassis *chassis, uint32_t now_ms);
