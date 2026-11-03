/*
 * 底盘速度模式 + 正弦加减速剖面的 PC 侧验证。
 *
 * chassis.c 的回调被替换成记录型桩：
 *   - emm_vel() 只记录各轮的有符号目标转速；
 *   - emm_sync_start() 才把四条目标真正“生效”（与 EMM 同步启动一致）；
 *   - emm_stop() 把对应轮速清零。
 * 然后按 1 ms 步进把已生效转速对时间积分，得到每轮走的脉冲数，
 * 与位置模式标定的 forward_pulses_per_m 对比，验证距离精度。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "chassis_config.h"
#include "chassis_motion.h"

static Hwt101 s_imu;
static float s_yaw_deg;

static MecanumConfig s_cfg;
static float s_cmd_rpm[5];      /* 当前已生效的有符号转速，下标 1..4 */
static float s_pending_rpm[5];  /* 已发但还没同步的转速 */
static uint8_t s_have_pending[5];
static float s_peak_rpm[5];     /* 整个过程中绝对值最大的有符号转速 */
static uint8_t s_cmd_acc[5];    /* 已生效帧里的 EMM acc 参数 */
static uint8_t s_pending_acc[5];
static unsigned s_stop_count;
static unsigned s_sync_count;

static int s_pass;
static int s_fail;

static int nearf(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

/* ---- EMM 适配器桩 ---- */
HAL_StatusTypeDef emm_vel(uint8_t id, uint8_t dir, uint16_t rpm,
                          uint8_t acc, bool sync)
{
    float signed_rpm = (float)rpm;
    (void)sync;

    if (id < 1U || id > 4U)
        return HAL_ERROR;

    if (dir != s_cfg.forward_dir[id])
        signed_rpm = -signed_rpm;

    s_pending_rpm[id] = signed_rpm;
    s_pending_acc[id] = acc;
    s_have_pending[id] = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef emm_sync_start(void)
{
    uint8_t id;

    for (id = 1U; id <= 4U; ++id) {
        if (s_have_pending[id]) {
            s_cmd_rpm[id] = s_pending_rpm[id];
            s_cmd_acc[id] = s_pending_acc[id];
            s_have_pending[id] = 0U;
            if (fabsf(s_cmd_rpm[id]) > fabsf(s_peak_rpm[id]))
                s_peak_rpm[id] = s_cmd_rpm[id];
        }
    }
    s_sync_count++;
    return HAL_OK;
}

HAL_StatusTypeDef emm_stop(uint8_t id, bool sync)
{
    (void)sync;
    if (id >= 1U && id <= 4U)
        s_cmd_rpm[id] = 0.0f;
    s_stop_count++;
    return HAL_OK;
}

/*
 * 位置模式桩：本测试只验证速度剖面，视觉微调的 0xFD 路径不参与，
 * 这里给 chassis.c 提供最小可链接实现。
 */
HAL_StatusTypeDef emm_pos(uint8_t id, uint8_t dir, uint16_t speed,
                          uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    (void)id; (void)dir; (void)speed; (void)acc;
    (void)clk; (void)raF; (void)snF;
    return HAL_OK;
}

void emm_prepare_position_feedback(uint8_t id) { (void)id; }
bool emm_all_position_reached_after(uint8_t mask, uint32_t start_ms)
{
    (void)mask; (void)start_ms;
    return true;
}
bool emm_any_position_fault_after(uint8_t mask, uint32_t start_ms)
{
    (void)mask; (void)start_ms;
    return false;
}
HAL_StatusTypeDef emm_read_status(uint8_t id) { (void)id; return HAL_OK; }

/* ---- HWT101 桩 ---- */
void hwt101_set_platform_moving(Hwt101 *imu, bool moving)
{
    if (imu) imu->platform_moving = moving ? 1U : 0U;
}

float hwt101_world_yaw_deg(const Hwt101 *imu)
{
    (void)imu;
    return s_yaw_deg;
}

bool hwt101_world_zero_ready(const Hwt101 *imu)
{
    (void)imu;
    return true;
}

bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms)
{
    (void)imu;
    (void)now_ms;
    (void)timeout_ms;
    return true;
}

static void sim_reset(Chassis *chassis, float yaw_deg)
{
    memset(chassis, 0, sizeof(*chassis));
    chassis_config_init_mecanum(&s_cfg);
    chassis_init(chassis, &s_cfg);
    chassis_bind_imu_filter(chassis, &s_imu);

    memset(&s_imu, 0, sizeof(s_imu));
    s_imu.has_angle = true;
    s_imu.has_gyro = true;
    s_imu.world_zero_ready = true;
    s_imu.gyro_z_dps = 0.0f;
    s_imu.yaw_deg = yaw_deg;
    s_yaw_deg = yaw_deg;

    memset(s_cmd_rpm, 0, sizeof(s_cmd_rpm));
    memset(s_pending_rpm, 0, sizeof(s_pending_rpm));
    memset(s_have_pending, 0, sizeof(s_have_pending));
    memset(s_peak_rpm, 0, sizeof(s_peak_rpm));
    memset(s_cmd_acc, 0, sizeof(s_cmd_acc));
    memset(s_pending_acc, 0, sizeof(s_pending_acc));
    s_stop_count = 0U;
    s_sync_count = 0U;
}

/*
 * 跑到剖面完成或超时，返回运动时长。pulses[] 为各轮脉冲积分。
 * 每毫秒用“当前生效转速”积分：rpm * 1ms / 60000 * pulses_per_rev。
 */
static uint32_t sim_run(Chassis *chassis, uint32_t start_ms, uint32_t max_ms,
                        float pulses[5])
{
    uint32_t t;
    uint8_t id;

    memset(pulses, 0, sizeof(float) * 5U);

    for (t = 0U; t < max_ms; ++t) {
        chassis_update(chassis, start_ms + t);

        for (id = 1U; id <= 4U; ++id)
            pulses[id] += s_cmd_rpm[id] * (float)s_cfg.motor_pulses_per_rev /
                          60000.0f;

        if (!chassis_busy(chassis) &&
            (chassis_position_done(chassis) || chassis_position_failed(chassis)))
            break;
    }
    return t + 1U;
}

static void test_forward_distance_matches_calibration(void)
{
    Chassis chassis;
    float pulses[5];
    float expected = 0.9f * CHASSIS_FORWARD_PULSES_PER_M;
    float avg;
    uint32_t duration;

    sim_reset(&chassis, 0.0f);
    chassis_motion_move(&chassis, 0.0f, 0.9f, 0.0f, 0.8f, 0U);
    duration = sim_run(&chassis, 0U, 5000U, pulses);

    avg = (pulses[1] + pulses[2] + pulses[3] + pulses[4]) * 0.25f;

    printf("    0.9 m forward: %.1f pulses (expect %.1f), %.0f ms\n",
           avg, expected, (float)duration);
    check(nearf(avg, expected, expected * 0.02f),
          "Velocity profile integrates to the calibrated 0.9 m pulse count");
    check(chassis_position_done(&chassis) && !chassis_position_failed(&chassis),
          "Profile reports done after the STOP sequence");
    check(duration > 1000U && duration < 2000U,
          "Profile duration matches sine ramp + cruise for 0.9 m at 0.8 m/s");
    check(s_cmd_acc[1] == 0U && s_cmd_acc[2] == 0U &&
          s_cmd_acc[3] == 0U && s_cmd_acc[4] == 0U,
          "Profile mode always sends EMM acc=0 so the ramp is not doubled");
}

static void test_backward_drift_keeps_edge_direction(void)
{
    Chassis chassis;
    float pulses[5];
    float expected = 0.9f * CHASSIS_FORWARD_PULSES_PER_M;
    float avg;
    int8_t i;

    /*
     * 边方向为正北(+Y)，车头朝南(180°) -> edge_turn 选择后退：
     * body_dy 为负、目标航向也是 180°。世界位移必须仍然朝北，
     * 所以车体系速度必须是 -Y（四个轮子都应反转）。
     */
    sim_reset(&chassis, 180.0f);
    chassis_motion_drift_translate(&chassis, 0.0f, 1.0f, -0.9f,
                                   180.0f, 0.8f, 0U);
    sim_run(&chassis, 0U, 5000U, pulses);

    avg = (pulses[1] + pulses[2] + pulses[3] + pulses[4]) * 0.25f;
    printf("    backward drift: %.1f pulses (expect -%.1f)\n", avg, expected);

    check(nearf(avg, -expected, expected * 0.05f),
          "Backward drift travels the edge distance in the negative body axis");
    for (i = 1; i <= 4; ++i)
        check(s_peak_rpm[i] < 0.0f,
              "Backward drift commands reverse wheel rotation");
}

/*
 * 圆滑拐角的底盘衔接：直线末段只加速不减速（ramp_out=0），完成后保持
 * 满速、不发 STOP；圆弧段恒速转动速度方向；弧后直线只减速不加速。
 */
static void test_corner_arc_blend_segments(void)
{
    Chassis chassis;
    float pulses[5];
    float expected_forward = 0.25f * CHASSIS_FORWARD_PULSES_PER_M;
    float expected_arc_ms;
    float avg;
    unsigned stop_before;
    uint32_t duration;

    /* 1) 直线末段：ramp_in=1, ramp_out=0。 */
    sim_reset(&chassis, 0.0f);
    chassis_motion_drift_segment(&chassis, 0.0f, 1.0f, 0.25f, 0.0f,
                                 0.8f, 1U, 0U, 0U);
    stop_before = s_stop_count;
    duration = sim_run(&chassis, 0U, 3000U, pulses);

    avg = (pulses[1] + pulses[2] + pulses[3] + pulses[4]) * 0.25f;
    printf("    corner entry straight: %.1f pulses (expect %.1f), %u ms\n",
           avg, expected_forward, duration);

    check(nearf(avg, expected_forward, expected_forward * 0.05f),
          "Corner entry straight covers the trimmed distance");
    check(s_stop_count == stop_before,
          "Corner entry straight keeps full speed without a STOP");
    check(s_cmd_rpm[1] > 20.0f && s_cmd_rpm[3] > 20.0f,
          "Wheels are still running when the entry straight completes");

    /* 2) 圆弧：弧长 = R*pi/2，时间 = 弧长/车速。 */
    sim_reset(&chassis, 0.0f);
    chassis_motion_arc_world(&chassis, 0.0f, 90.0f, 0.35f, 0.8f, 0U);
    expected_arc_ms = 0.35f * 1.5707964f / 0.8f * 1000.0f;
    {
        uint32_t t;

        memset(pulses, 0, sizeof(pulses));
        for (t = 0U; t < 3000U; ++t) {
            uint8_t id;

            /* 理想跟随：航向按圆弧切线同步转动，PD 只修残差。 */
            s_yaw_deg = 90.0f * (float)t / expected_arc_ms;
            chassis_update(&chassis, t);

            for (id = 1U; id <= 4U; ++id)
                pulses[id] += s_cmd_rpm[id] *
                              (float)s_cfg.motor_pulses_per_rev / 60000.0f;

            if (!chassis_busy(&chassis) &&
                (chassis_position_done(&chassis) ||
                 chassis_position_failed(&chassis)))
                break;
        }
        duration = t + 1U;
    }

    printf("    corner arc: %u ms (expect %.0f ms) FL=%.0f RL=%.0f RR=%.0f FR=%.0f\n",
           duration, expected_arc_ms,
           s_peak_rpm[1], s_peak_rpm[2], s_peak_rpm[3], s_peak_rpm[4]);

    check(nearf((float)duration, expected_arc_ms, 20.0f),
          "Arc lasts exactly arc-length / speed");
    /* 顺时针转弯：左侧两轮比右侧快，且两两对称。 */
    check(s_peak_rpm[1] > s_peak_rpm[4] &&
          nearf(s_peak_rpm[1], s_peak_rpm[2], 1.0f) &&
          nearf(s_peak_rpm[3], s_peak_rpm[4], 1.0f),
          "Arc commands forward motion plus a clockwise wheel differential");

    /* 3) 弧后直线：ramp_in=0, ramp_out=1，正常减速停车。 */
    sim_reset(&chassis, 90.0f);
    chassis_motion_drift_segment(&chassis, 1.0f, 0.0f, 0.25f, 90.0f,
                                 0.8f, 0U, 1U, 0U);
    duration = sim_run(&chassis, 0U, 3000U, pulses);

    avg = (pulses[1] + pulses[2] + pulses[3] + pulses[4]) * 0.25f;
    printf("    corner exit straight: %.1f pulses (expect %.1f), %u ms\n",
           avg, expected_forward, duration);

    check(nearf(avg, expected_forward, expected_forward * 0.05f),
          "Corner exit straight covers the trimmed distance");
    check(chassis_position_done(&chassis) && !chassis_position_failed(&chassis),
          "Corner exit straight completes normally");
}

/*
 * 倒车走弧：切线从西(-90°)转到北(0°)，车头锁在切线+180°（0°->90°）。
 * 弧中点车体纯倒车 + 顺时针自转：四轮都应反转，左侧转得比右侧快。
 */
static void test_reverse_arc_keeps_heading_opposite_travel(void)
{
    Chassis chassis;
    float expected_ms;
    uint32_t mid_ms;
    uint32_t t;

    sim_reset(&chassis, 0.0f);
    chassis_motion_arc_world_ex(&chassis, -90.0f, 90.0f, 0.35f, 0.8f, 1U, 0U);

    expected_ms = 0.35f * 1.5707964f / 0.8f * 1000.0f;
    mid_ms = (uint32_t)(expected_ms * 0.5f);

    for (t = 0U; t <= mid_ms; ++t) {
        /* 理想跟随：航向 = 切线 + 180°（车头从 90° 转到 180°）。 */
        s_yaw_deg = 90.0f + 90.0f * (float)t / expected_ms;
        chassis_update(&chassis, t);
    }

    printf("    reverse arc mid: FL=%.0f RL=%.0f RR=%.0f FR=%.0f\n",
           s_cmd_rpm[1], s_cmd_rpm[2], s_cmd_rpm[3], s_cmd_rpm[4]);

    check(s_cmd_rpm[1] < -20.0f && s_cmd_rpm[2] < -20.0f &&
          s_cmd_rpm[3] < -20.0f && s_cmd_rpm[4] < -20.0f,
          "Reverse arc drives all four wheels backwards");
    check(nearf(s_cmd_rpm[1], s_cmd_rpm[2], 1.0f) &&
          nearf(s_cmd_rpm[3], s_cmd_rpm[4], 1.0f) &&
          s_cmd_rpm[4] < s_cmd_rpm[1],
          "Reverse arc keeps the clockwise wheel differential");
}

static void test_drift_rotates_world_vector_into_body(void)
{
    Chassis chassis;
    uint32_t t;

    /*
     * 边方向为正东(+X)，车头朝北(0°)但目标航向是 90°：
     * 起步瞬间世界速度是 +X，旋到车体系就是 vx>0（向右横移）；
     * 同时航向 PD 输出 CW 自转。期望 FL/RR 正转、FR/RL 反转。
     */
    sim_reset(&chassis, 0.0f);
    chassis_motion_drift_translate(&chassis, 1.0f, 0.0f, 0.9f,
                                   90.0f, 0.8f, 0U);

    /* 只跑 200 ms，观察起步阶段的车体系速度符号。 */
    for (t = 0U; t < 200U; ++t)
        chassis_update(&chassis, t);

    printf("    drift east peak rpm: FL=%.0f RL=%.0f RR=%.0f FR=%.0f\n",
           s_peak_rpm[1], s_peak_rpm[2], s_peak_rpm[3], s_peak_rpm[4]);

    check(s_peak_rpm[1] > 0.0f && s_peak_rpm[2] < 0.0f &&
          s_peak_rpm[3] > 0.0f && s_peak_rpm[4] < 0.0f,
          "World +X velocity is rotated into a strafe-right body command");
    check(fabsf(s_peak_rpm[1] + s_peak_rpm[4]) < 1.0f &&
          fabsf(s_peak_rpm[2] + s_peak_rpm[3]) < 1.0f,
          "Left/right wheels are antisymmetric for the strafe");
}

static void test_zero_move_completes_without_motion(void)
{
    Chassis chassis;

    sim_reset(&chassis, 0.0f);
    chassis_motion_translate(&chassis, 0.0f, 0.0f, 0.8f, 0U);

    check(!chassis_busy(&chassis) && chassis_position_done(&chassis),
          "Zero-distance move completes immediately without commanding motion");
    check(s_sync_count == 0U && s_stop_count == 0U,
          "Zero-distance move does not touch the CAN state machine");
}

static void test_stop_cancels_profile(void)
{
    Chassis chassis;
    uint32_t t;

    sim_reset(&chassis, 0.0f);
    chassis_motion_move(&chassis, 0.0f, 0.9f, 0.0f, 0.8f, 0U);

    for (t = 0U; t < 200U; ++t)
        chassis_update(&chassis, t);

    check(chassis.mode == CHASSIS_MODE_PROFILE && chassis.profile_running,
          "Profile is running before the abort");

    chassis_stop(&chassis);

    check(!chassis.profile_running && !chassis.profile_finishing,
          "chassis_stop cancels the active profile");
    check(chassis_position_failed(&chassis) && !chassis_position_done(&chassis),
          "Aborted profile reports failure instead of completion");

    /*
     * 直接速度命令（航向 PD 等）不属于剖面，应带上调用方设置的 EMM acc。
     * 先把 STOP 序列跑完，再下发一条纯自转速度。
     */
    for (t = 0U; t < 50U; ++t)
        chassis_update(&chassis, 200U + t);
    chassis.acceleration = 130U;
    chassis_set_velocity(&chassis, 0.0f, 0.0f, 0.3f, 250U);
    for (t = 0U; t < 20U; ++t)
        chassis_update(&chassis, 250U + t);
    check(s_cmd_acc[1] == 130U && s_cmd_acc[2] == 130U &&
          s_cmd_acc[3] == 130U && s_cmd_acc[4] == 130U,
          "Direct velocity mode keeps the caller's EMM acceleration");
}

static void test_direct_velocity_aborts_profile(void)
{
    Chassis chassis;
    uint32_t t;

    sim_reset(&chassis, 0.0f);
    chassis_motion_move(&chassis, 0.0f, 0.9f, 0.0f, 0.8f, 0U);

    for (t = 0U; t < 200U; ++t)
        chassis_update(&chassis, t);

    check(chassis.profile_running,
          "Profile is running before a direct velocity command preempts it");

    chassis_set_velocity(&chassis, 0.0f, 0.0f, 0.2f, 200U);

    check(!chassis.profile_running && !chassis.profile_finishing &&
          chassis_position_failed(&chassis) && !chassis_position_done(&chassis),
          "Preempted profile reports failure so EdgeMotion cannot wait forever");
}

int main(void)
{
    test_forward_distance_matches_calibration();
    test_backward_drift_keeps_edge_direction();
    test_drift_rotates_world_vector_into_body();
    test_corner_arc_blend_segments();
    test_reverse_arc_keeps_heading_opposite_travel();
    test_zero_move_completes_without_motion();
    test_stop_cancels_profile();
    test_direct_velocity_aborts_profile();
    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
