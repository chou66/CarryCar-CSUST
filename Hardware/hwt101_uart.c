#include "hwt101_uart.h"

static float wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 释放保持时重算偏置，使输出从保持值连续起步，不把静止期原始漂移一次性跳回。 */
static void release_yaw_hold(Hwt101 *imu)
{
    if (!imu || !imu->yaw_hold_active) return;

    imu->yaw_filter_offset_deg = wrap_deg(imu->yaw_hold_deg - imu->yaw_raw_deg);
    imu->yaw_deg = imu->yaw_hold_deg;
    imu->yaw_hold_active = 0U;
    imu->yaw_hold_exit_count++;
}

static void update_yaw_filter(Hwt101 *imu, float raw_yaw_deg, uint32_t now_ms)
{
    float gyro_abs;
    bool gyro_fresh;

    imu->yaw_raw_deg = raw_yaw_deg;

    if (!imu->yaw_filter_ready)
    {
        imu->yaw_filter_ready = true;
        imu->yaw_filter_offset_deg = 0.0f;
        imu->yaw_hold_deg = raw_yaw_deg;
        imu->yaw_deg = raw_yaw_deg;
        imu->yaw_still_candidate = false;
        return;
    }

    gyro_fresh = imu->has_gyro &&
                 (uint32_t)(now_ms - imu->last_gyro_update_ms) <=
                     HWT101_GYRO_FRESH_MS;
    gyro_abs = absf_local(imu->gyro_z_dps);

    /* 运动命令优先：保持层不介入行驶/转向，只保留连续偏置。 */
    if (imu->platform_moving)
    {
        release_yaw_hold(imu);
        imu->yaw_still_candidate = false;
        imu->yaw_deg = wrap_deg(raw_yaw_deg + imu->yaw_filter_offset_deg);
        return;
    }

    /* 没有新鲜角速度时禁止锁航向，避免串口缺帧被误判成静止。 */
    if (!gyro_fresh)
    {
        release_yaw_hold(imu);
        imu->yaw_still_candidate = false;
        imu->yaw_deg = wrap_deg(raw_yaw_deg + imu->yaw_filter_offset_deg);
        return;
    }

    if (imu->yaw_hold_active)
    {
        /* 施密特上门限：外力转动车体也会立即释放，不依赖底盘命令。 */
        if (gyro_abs >= HWT101_STILL_GYRO_EXIT_DPS)
        {
            release_yaw_hold(imu);
            imu->yaw_deg = wrap_deg(raw_yaw_deg + imu->yaw_filter_offset_deg);
        }
        else
        {
            imu->yaw_deg = imu->yaw_hold_deg;
        }
        return;
    }

    imu->yaw_deg = wrap_deg(raw_yaw_deg + imu->yaw_filter_offset_deg);

    /* 下门限必须连续满足一段时间，避免刚停车时误锁住减速尾巴。 */
    if (gyro_abs <= HWT101_STILL_GYRO_ENTER_DPS)
    {
        if (!imu->yaw_still_candidate)
        {
            imu->yaw_still_candidate = true;
            imu->yaw_still_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - imu->yaw_still_since_ms) >=
                 HWT101_STILL_SETTLE_MS)
        {
            imu->yaw_hold_deg = imu->yaw_deg;
            imu->yaw_hold_active = 1U;
            imu->yaw_hold_enter_count++;
            imu->yaw_still_candidate = false;
        }
    }
    else
    {
        imu->yaw_still_candidate = false;
    }
}

void hwt101_init(Hwt101 *imu)
{
    *imu = (Hwt101){0};
}

void hwt101_set_platform_moving(Hwt101 *imu, bool moving)
{
    if (!imu) return;

    /* 其余滤波状态只由 hwt101_feed_byte() 所在的串口中断上下文写入。 */
    imu->platform_moving = moving ? 1U : 0U;
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
        imu->last_gyro_update_ms = now_ms;
        imu->has_gyro = true;
    } else if (frame[1] == 0x53U) {          /* Euler angle frame */
        imu->roll_deg = x * (180.0f / 32768.0f);
        imu->pitch_deg = y * (180.0f / 32768.0f);
        update_yaw_filter(imu, z * (180.0f / 32768.0f), now_ms);
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
