#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile float roll_deg;
    volatile float pitch_deg;
    volatile float yaw_deg;
    volatile float gyro_z_dps;
    volatile uint32_t last_update_ms;
    /* 旧调试链使用的局部零点，保留兼容 VOFA/单边测试。 */
    float yaw_zero_deg;

    /* 正式 Mission 的世界零点：整场任务只建立一次。 */
    float world_zero_raw_deg;
    bool world_zero_ready;
    bool has_angle;
} Hwt101;

void hwt101_init(Hwt101 *imu);
/* Call once for every byte received by the USART RX interrupt/DMA parser. */
void hwt101_feed_byte(Hwt101 *imu, uint8_t byte, uint32_t now_ms);
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
