#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "hwt101_uart.h"

static int s_pass;
static int s_fail;

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static int nearf_local(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static int16_t encode_axis(float value, float full_scale)
{
    float scaled = value * (32768.0f / full_scale);
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)scaled;
}

static void feed_frame(Hwt101 *imu, uint8_t type, int16_t z,
                       uint32_t now_ms)
{
    uint8_t frame[11] = {0x55U, type, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t checksum = 0U;
    uint8_t i;

    frame[6] = (uint8_t)((uint16_t)z & 0xFFU);
    frame[7] = (uint8_t)(((uint16_t)z >> 8) & 0xFFU);
    for (i = 0U; i < 10U; ++i) checksum = (uint8_t)(checksum + frame[i]);
    frame[10] = checksum;

    for (i = 0U; i < 11U; ++i)
        hwt101_feed_byte(imu, frame[i], now_ms);
}

static void feed_sample(Hwt101 *imu, float yaw_deg, float gyro_z_dps,
                        uint32_t now_ms)
{
    feed_frame(imu, 0x52U, encode_axis(gyro_z_dps, 2000.0f), now_ms);
    feed_frame(imu, 0x53U, encode_axis(yaw_deg, 180.0f), now_ms);
}

static void feed_angle_only(Hwt101 *imu, float yaw_deg, uint32_t now_ms)
{
    feed_frame(imu, 0x53U, encode_axis(yaw_deg, 180.0f), now_ms);
}

static void test_stationary_hold_and_continuous_release(void)
{
    Hwt101 imu;
    float held_yaw;

    hwt101_init(&imu);
    hwt101_set_platform_moving(&imu, false);

    feed_sample(&imu, 0.0f, 0.0f, 0U);
    feed_sample(&imu, 0.1f, 0.0f, 100U);
    feed_sample(&imu, 0.3f, 0.0f, 400U);
    check(imu.yaw_hold_active == 1U,
          "Quiet stopped chassis enters yaw hold after settle time");

    held_yaw = imu.yaw_deg;
    feed_sample(&imu, 1.2f, 0.0f, 1500U);
    check(nearf_local(imu.yaw_raw_deg, 1.2f, 0.02f) &&
          nearf_local(imu.yaw_deg, held_yaw, 0.02f),
          "Stationary fused-yaw drift is rejected while raw yaw remains visible");

    hwt101_set_platform_moving(&imu, true);
    check(imu.platform_moving == 1U &&
          nearf_local(imu.yaw_deg, held_yaw, 0.02f),
          "Motion hint crosses contexts without modifying filter internals");

    feed_sample(&imu, 2.2f, 5.0f, 1520U);
    check(imu.yaw_hold_active == 0U &&
          nearf_local(imu.yaw_deg, held_yaw, 0.04f),
          "Next IMU frame releases hold without an output jump");

    feed_sample(&imu, 3.2f, 5.0f, 1540U);
    check(nearf_local(imu.yaw_deg, held_yaw + 1.0f, 0.04f),
          "Subsequent moving yaw follows real angle delta without low-pass lag");

    hwt101_set_platform_moving(&imu, false);
    feed_sample(&imu, 3.25f, 0.0f, 1600U);
    feed_sample(&imu, 3.55f, 0.0f, 1900U);
    check(imu.yaw_hold_active == 1U && imu.yaw_hold_enter_count == 2U,
          "A later stop can acquire a new hold anchor");
}

static void test_motion_and_missing_gyro_never_lock(void)
{
    Hwt101 moving;
    Hwt101 no_gyro;

    hwt101_init(&moving);
    hwt101_set_platform_moving(&moving, true);
    feed_sample(&moving, 0.0f, 0.0f, 0U);
    feed_sample(&moving, 0.5f, 0.0f, 500U);
    feed_sample(&moving, 1.0f, 0.0f, 1000U);
    check(moving.yaw_hold_active == 0U &&
          nearf_local(moving.yaw_deg, 1.0f, 0.02f),
          "Straight motion is never mistaken for stationary yaw hold");

    hwt101_init(&no_gyro);
    feed_angle_only(&no_gyro, 0.0f, 0U);
    feed_angle_only(&no_gyro, 0.5f, 500U);
    feed_angle_only(&no_gyro, 1.0f, 1000U);
    check(no_gyro.yaw_hold_active == 0U &&
          nearf_local(no_gyro.yaw_deg, 1.0f, 0.02f),
          "Missing gyro frames fail open instead of freezing heading");
}

static void test_external_rotation_releases_hold(void)
{
    Hwt101 imu;

    hwt101_init(&imu);
    feed_sample(&imu, 10.0f, 0.0f, 0U);
    feed_sample(&imu, 10.0f, 0.0f, 100U);
    feed_sample(&imu, 10.1f, 0.0f, 400U);
    check(imu.yaw_hold_active == 1U, "Test precondition: yaw is held");

    feed_sample(&imu, 10.3f, 1.2f, 420U);
    check(imu.yaw_hold_active == 0U && imu.yaw_hold_exit_count == 1U,
          "Physical rotation above hysteresis threshold releases hold");
}

int main(void)
{
    test_stationary_hold_and_continuous_release();
    test_motion_and_missing_gyro_never_lock();
    test_external_rotation_releases_hold();

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
