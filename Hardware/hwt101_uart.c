#include "hwt101_uart.h"

static float wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

void hwt101_init(Hwt101 *imu)
{
    *imu = (Hwt101){0};
}

void hwt101_set_zero(Hwt101 *imu)
{
    imu->yaw_zero_deg = imu->yaw_deg;
}

void hwt101_feed_byte(Hwt101 *imu, uint8_t byte, uint32_t now_ms)
{
    static uint8_t frame[11];
    static uint8_t index;
    uint8_t checksum = 0U;
    int16_t x, y, z;

    if (index == 0U && byte != 0x55U) return;
    frame[index++] = byte;
    if (index < sizeof(frame)) return;
    index = 0U;
    for (uint8_t i = 0; i < 10U; ++i) checksum += frame[i];
    if (checksum != frame[10] || frame[0] != 0x55U) return;

    x = (int16_t)((uint16_t)frame[2] | ((uint16_t)frame[3] << 8));
    y = (int16_t)((uint16_t)frame[4] | ((uint16_t)frame[5] << 8));
    z = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    if (frame[1] == 0x52U) {                 /* angular velocity frame */
        imu->gyro_z_dps = z * (2000.0f / 32768.0f);
    } else if (frame[1] == 0x53U) {          /* Euler angle frame */
        imu->roll_deg = x * (180.0f / 32768.0f);
        imu->pitch_deg = y * (180.0f / 32768.0f);
        imu->yaw_deg = z * (180.0f / 32768.0f);
        imu->last_update_ms = now_ms;
        imu->has_angle = true;
    }
}

float hwt101_yaw_relative_deg(const Hwt101 *imu)
{
    return wrap_deg(imu->yaw_deg - imu->yaw_zero_deg);
}

void hwt101_set_world_zero(Hwt101 *imu)
{
    if (!imu || !imu->has_angle) return;
    imu->world_zero_raw_deg = imu->yaw_deg;
    imu->world_zero_ready = true;
}

void hwt101_clear_world_zero(Hwt101 *imu)
{
    if (!imu) return;
    imu->world_zero_raw_deg = 0.0f;
    imu->world_zero_ready = false;
}

float hwt101_world_yaw_deg(const Hwt101 *imu)
{
    if (!imu || !imu->world_zero_ready) return 0.0f;
    return wrap_deg(HWT101_WORLD_YAW_SIGN *
                    (imu->yaw_deg - imu->world_zero_raw_deg));
}

bool hwt101_world_zero_ready(const Hwt101 *imu)
{
    return (imu && imu->world_zero_ready) ? true : false;
}

bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms)
{
    return imu->has_angle && (uint32_t)(now_ms - imu->last_update_ms) <= timeout_ms;
}
