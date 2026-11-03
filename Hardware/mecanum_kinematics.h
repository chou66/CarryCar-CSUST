#pragma once

#include <stdint.h>

/* Body frame: X right(+), Y forward/up(+), wz counter-clockwise(+). */
typedef struct {
    float wheel_radius_m;
    float half_length_m;
    float half_width_m;
    float gear_ratio;               /* motor rev / wheel rev */
    uint16_t max_motor_rpm;
    uint32_t motor_pulses_per_rev;  /* 驱动器一圈的脉冲数，如 3200 */
    float forward_pulses_per_m;     /* 实车标定；为 0 时退回理论值 */
    float strafe_pulses_per_m;      /* 实车标定；为 0 时退回理论值 */
    float rotate_pulses_per_rad;    /* 实车标定；为 0 时退回理论值 */
    uint8_t forward_dir[7];         /* 包含预留位，支持ID 1~6的安装方向 */
} MecanumConfig;

typedef struct {
    uint16_t rpm;
    uint32_t pulses;                /* 位置模式下的目标脉冲数 */
    uint8_t dir;
} WheelCommand;

typedef struct {
    WheelCommand fl; /* motor ID 1 */
    WheelCommand rl; /* motor ID 2 */
    WheelCommand rr; /* motor ID 3 */
    WheelCommand fr; /* motor ID 4 */
} MecanumWheelCommand;

/* 速度模式：返回受最大转速限制后的各轮指令 */
void mecanum_inverse(const MecanumConfig *cfg, float vx_mps, float vy_mps,
                     float wz_radps, MecanumWheelCommand *out);

/* 位置模式：根据给定绝对位移，等比例分配转速和脉冲，确保各轮同时到达 */
void mecanum_inverse_pos(const MecanumConfig *cfg, float dx_m, float dy_m,
                         float dtheta_rad, float speed_mps, MecanumWheelCommand *out);

/*
 * 有符号轮脉冲分配（不含方向位），单位：驱动器脉冲。
 * 输出顺序固定为 [FL(id1), RL(id2), RR(id3), FR(id4)]，与 MecanumWheelCommand 一致。
 *
 * 速度模式的正弦加减速剖面用它得到各轮脉冲比例，再换算成轮速比例：
 * 这样即使位置模式本身不再下发 0xFD，位移仍然沿用同一套实测脉冲标定，
 * 不会因为改成速度模式而丢失距离精度。
 */
void mecanum_inverse_pulses(const MecanumConfig *cfg,
                            float dx_m, float dy_m, float dtheta_rad,
                            float pulses[4]);
