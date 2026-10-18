#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * HWT101 航向抗漂移层。
 *
 * 不对航向角直接加死区：死区只能延迟慢漂，累计越界后仍会跳动，而且会吞掉
 * 真实的小角度误差。这里改用“底盘停车提示 + Z 轴角速度施密特门限”：
 *   - 底盘明确停车、角速度持续很小后，保持当前航向；
 *   - 底盘收到运动命令或检测到真实旋转，立即连续释放；
 *   - 行驶期间不低通航向，避免给闭环控制引入相位滞后。
 *
 * 0.8 dps 的释放门限低于当前 4 RPM 航向微调约 1.8 dps 的实际角速度，
 * 因此正常最慢转向也能及时退出保持。三个参数均可在实车数据基础上微调。
 */
#ifndef HWT101_STILL_GYRO_ENTER_DPS
#define HWT101_STILL_GYRO_ENTER_DPS       0.25f
#endif
#ifndef HWT101_STILL_GYRO_EXIT_DPS
#define HWT101_STILL_GYRO_EXIT_DPS        0.80f
#endif
#ifndef HWT101_STILL_SETTLE_MS
#define HWT101_STILL_SETTLE_MS             300U
#endif
#ifndef HWT101_GYRO_FRESH_MS
#define HWT101_GYRO_FRESH_MS               150U
#endif

typedef struct Hwt101 {
    volatile float roll_deg;
    volatile float pitch_deg;
    /* yaw_deg 保留原接口，但现在输出经过停车抗漂移处理的航向。 */
    volatile float yaw_deg;
    /* 传感器 0x53 帧的原始航向，仅供诊断，不参与任务控制。 */
    volatile float yaw_raw_deg;
    volatile float gyro_z_dps;
    volatile uint32_t last_update_ms;
    volatile uint32_t last_gyro_update_ms;

    /* Keil Watch 可直接观察的滤波状态。 */
    volatile uint8_t yaw_hold_active;
    volatile uint8_t platform_moving;
    volatile uint32_t yaw_hold_enter_count;
    volatile uint32_t yaw_hold_exit_count;

    /* 旧调试链使用的局部零点，保留兼容 VOFA/单边测试。 */
    float yaw_zero_deg;

    /* 正式 Mission 的世界零点：整场任务只建立一次。 */
    float world_zero_raw_deg;
    bool world_zero_ready;
    bool has_angle;
    bool has_gyro;

    /* 航向滤波内部连续状态。 */
    float yaw_filter_offset_deg;
    float yaw_hold_deg;
    uint32_t yaw_still_since_ms;
    bool yaw_filter_ready;
    bool yaw_still_candidate;
} Hwt101;

void hwt101_init(Hwt101 *imu);
/* Call once for every byte received by the USART RX interrupt/DMA parser. */
void hwt101_feed_byte(Hwt101 *imu, uint8_t byte, uint32_t now_ms);

/*
 * 底盘运动提示由 chassis 层维护。为避免主循环与串口中断同时写滤波状态，
 * 此函数只更新 volatile 标志；moving=true 会在下一帧航向数据到达时无跳变
 * 释放保持。moving=false 只允许开始静止判定，仍需满足角速度和稳定时间条件。
 */
void hwt101_set_platform_moving(Hwt101 *imu, bool moving);
void hwt101_set_zero(Hwt101 *imu);
float hwt101_yaw_relative_deg(const Hwt101 *imu);

/*
 * 世界航向定义（Mission/Navigation/EdgeMotion 统一使用）：
 *   0 deg   = 地图向上 (+Y)
 *   +90 deg = 地图向右 (+X)
 *   -90 deg = 地图向左 (-X)
 *   180 deg = 地图向下 (-Y)
 * 即顺时针为正。
 *
 * 当前 HWT101/旧 EdgeMotion 注释按 CCW 为正，因此默认取 -1。
 * 首次实车请手动从“向上”顺时针转到“向右”：若 world yaw 变为 +90，符号正确；
 * 若变为 -90，只改此宏为 +1.0f，其他节点角度和坐标转换不要改。
 */
#ifndef HWT101_WORLD_YAW_SIGN
#define HWT101_WORLD_YAW_SIGN (-1.0f)
#endif

void hwt101_set_world_zero(Hwt101 *imu);
void hwt101_clear_world_zero(Hwt101 *imu);
float hwt101_world_yaw_deg(const Hwt101 *imu);
bool hwt101_world_zero_ready(const Hwt101 *imu);
bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms);
