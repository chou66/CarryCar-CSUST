#include "edge_motion.h"
#include "app_uart.h"
#include "chassis_motion.h"
#include "hwt101_uart.h"
#include "map_graph.h"
#include "navigation.h"
#include "robot_config.h"
#include <math.h>

/*
 * 下列参数直接继承 10.4 VOFA 导航实车调试值。
 * 这次只把“已验证逻辑”迁入正式 EdgeMotion，不趁机重新调参。
 * 后续若实车需要修改，请 VOFA 回归值和正式 EdgeMotion 同步复测。
 */
#define EDGE_MOTION_MOVE_ACC                   150U
#define EDGE_MOTION_YAW_DEADBAND_DEG          0.50f
#define EDGE_MOTION_YAW_MAX_START_DEG         8.00f
#define EDGE_MOTION_YAW_CORRECT_RPM             20U
#define EDGE_MOTION_YAW_CORRECT_ACC             10U
#define EDGE_MOTION_YAW_SETTLE_MS              150U
#define EDGE_MOTION_IMU_TIMEOUT_MS             300U
#define EDGE_MOTION_YAW_CORRECT_TIMEOUT_MS    1800U

/*
 * Chassis 速度模式带 150 ms 软件看门狗。
 * 航向修正期间每 80 ms 刷新一次速度命令：
 *   - 正常运行时不会触发看门狗；
 *   - 如果 EdgeMotion_Update() 因程序异常停止执行，底盘仍会在约 150 ms 后自行停车。
 */
#define EDGE_MOTION_YAW_REFRESH_MS               80U

#define EDGE_MOTION_TWO_PI_F 6.28318530717958647692f
#define EDGE_MOTION_DEG_TO_RAD_F 0.01745329251994329577f

static uint8_t node_ok(uint8_t n)
{
    return (n >= 1U && n <= NODE_NUM) ? 1U : 0U;
}

static float edge_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int8_t edge_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

static float edge_wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

/*
 * 世界 -> 车身二维旋转。
 * 世界：+X=右，+Y=上；航向 0=上、顺时针为正。
 * 车身：+X=右，+Y=前。
 *
 * 例：yaw=-90 deg（车头向左）且世界要求向下 (0,-D)，
 * 结果为车身 (-D,0)，即小车左移。
 */
static void edge_world_to_body(float dx_world, float dy_world,
                               float world_yaw_deg,
                               float *dx_body, float *dy_body)
{
    float a = world_yaw_deg * EDGE_MOTION_DEG_TO_RAD_F;
    float c = cosf(a);
    float sn = sinf(a);

    if (dx_body) *dx_body = dx_world * c - dy_world * sn;
    if (dy_body) *dy_body = dx_world * sn + dy_world * c;
}

static void edge_save_acceleration(EdgeMotion_t *em)
{
    if (!em || !em->chassis || em->acceleration_saved)
        return;

    em->saved_acceleration = em->chassis->acceleration;
    em->acceleration_saved = 1U;
}

static void edge_restore_acceleration(EdgeMotion_t *em)
{
    if (!em || !em->chassis || !em->acceleration_saved)
        return;

    em->chassis->acceleration = em->saved_acceleration;
    em->acceleration_saved = 0U;
}

/*
 * 把 VOFA 已验证的“20 RPM 原地纠偏”换算成 Chassis 速度模式的 wz(rad/s)。
 *
 * mecanum_inverse() 纯旋转时每个轮子的转速幅值为：
 *   rpm = (half_length + half_width) * |wz|
 *         * 60 * gear_ratio / (2*pi*wheel_radius)
 *
 * 因此这里反算 wz，保证正式 EdgeMotion 仍走：
 *   chassis_set_velocity -> mecanum -> emm_vel -> CAN
 * 而不是在运动层重新直接拼 EMM 指令。
 */
static float edge_yaw_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
{
    float k;
    float factor;

    if (!chassis ||
        chassis->config.wheel_radius_m <= 0.0f ||
        chassis->config.gear_ratio <= 0.0f)
        return 0.0f;

    k = chassis->config.half_length_m + chassis->config.half_width_m;
    if (k <= 0.0f)
        return 0.0f;

    factor = 60.0f * chassis->config.gear_ratio /
             (EDGE_MOTION_TWO_PI_F * chassis->config.wheel_radius_m);

    if (factor <= 0.0f)
        return 0.0f;

    return (float)rpm / (k * factor);
}

static void edge_clear_runtime(EdgeMotion_t *em)
{
    em->from_node = 0U;
    em->to_node = 0U;
    em->settle_until_ms = 0U;
    em->yaw_correct_start_ms = 0U;
    em->next_yaw_refresh_ms = 0U;
    em->yaw_start_sign = 0;
    em->yaw_velocity_active = 0U;
}

/*
 * 结束航向速度修正。
 * chassis_motion_stop() 通过正式 Chassis 状态机下发四轮 STOP；
 * 不直接调用 VOFA 专用 Emm_V5_* API。
 */
static void edge_stop_yaw_velocity(EdgeMotion_t *em)
{
    if (!em || !em->chassis || !em->yaw_velocity_active)
        return;

    chassis_motion_stop(em->chassis);
    em->yaw_velocity_active = 0U;
}

/*
 * 当前边执行失败后的统一收口。
 *
 * Navigation 的 current_node 只允许由 EdgeReached() 推进，所以失败时不伪造位置；
 * 这里仅停止真实底盘并锁定 EDGE_MOTION_ERROR，等待上层人工/任务层恢复。
 */
static void edge_fail(EdgeMotion_t *em)
{
    if (!em || !em->chassis)
        return;

    edge_stop_yaw_velocity(em);
    chassis_motion_stop(em->chassis);
    edge_restore_acceleration(em);
    em->state = EDGE_MOTION_ERROR;
}

/*
 * 一条真实物理边只有在：
 *   位置运动完成 + HWT101 航向确认完成
 * 之后才能提交给 Navigation。
 */
static uint8_t edge_commit(EdgeMotion_t *em)
{
    if (!em || !em->hmi_nav)
        return 0U;

    if (!HMI_Navigation_EdgeReached(em->hmi_nav))
    {
        edge_fail(em);
        return 0U;
    }

    HMI_Navigation_UpdateDebugDisplay(em->hmi_nav);
    edge_restore_acceleration(em);
    edge_clear_runtime(em);
    em->state = EDGE_MOTION_IDLE;
    return 1U;
}

/*
 * 启动边末航向修正。
 *
 * world yaw 采用“顺时针为正”。
 * yaw_error > 0：实车相对边起点向 CW 漂，故命令 CCW（Chassis wz > 0）。
 * yaw_error < 0：实车相对边起点向 CCW 漂，故命令 CW（Chassis wz < 0）。
 */
static uint8_t edge_start_yaw_correction(EdgeMotion_t *em,
                                         float yaw_error_deg,
                                         uint32_t now_ms)
{
    float wz_mag;
    float wz_cmd;

    if (!em || !em->chassis)
        return 0U;

    wz_mag = edge_yaw_wz_for_rpm(em->chassis, EDGE_MOTION_YAW_CORRECT_RPM);
    if (wz_mag <= 0.0f)
        return 0U;

    em->chassis->acceleration = EDGE_MOTION_YAW_CORRECT_ACC;

    wz_cmd = (yaw_error_deg > 0.0f) ? wz_mag : -wz_mag;
    chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    em->yaw_start_sign = edge_signf(yaw_error_deg);
    em->yaw_correct_start_ms = now_ms;
    em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
    em->yaw_velocity_active = 1U;
    em->state = EDGE_MOTION_YAW_CORRECTING;
    return 1U;
}

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav)
{
    if (!em) return;

    em->chassis = chassis;
    em->hmi_nav = hmi_nav;
    em->state = EDGE_MOTION_IDLE;
    em->saved_acceleration = 0U;
    em->acceleration_saved = 0U;
    em->world_dx_m = 0.0f;
    em->world_dy_m = 0.0f;
    em->body_dx_m = 0.0f;
    em->body_dy_m = 0.0f;
    em->edge_heading_ref_deg = 0.0f;
    edge_clear_runtime(em);
}

void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms)
{
    NavigationState_t nav_state;
    Hwt101 *imu;
    float yaw_error_deg;
    int8_t yaw_sign;

    if (!em || !em->chassis || !em->hmi_nav ||
        !HMI_Navigation_IsInitialized(em->hmi_nav))
        return;

    nav_state = Navigation_GetState(&em->hmi_nav->nav);
    imu = AppUart_GetImu();

    /*
     * 真暂停必须停止物理运动。
     * 位置模式被异步打断后，驱动器剩余距离无法可靠还原，所以保持 10.4 原设计：
     * 不盲目 resume 当前边，而是进入 PAUSE_ABORT，等待上层重新定位/规划。
     */
    if (nav_state == NAV_STATE_PAUSED)
    {
        if (EdgeMotion_IsActive(em))
        {
            edge_stop_yaw_velocity(em);
            chassis_motion_stop(em->chassis);
            edge_restore_acceleration(em);
            em->state = EDGE_MOTION_PAUSE_ABORT;
        }
        return;
    }

    if (em->state == EDGE_MOTION_PAUSE_ABORT ||
        em->state == EDGE_MOTION_ERROR)
        return;

    /* 1) 正在执行当前拓扑边的平移位置运动。 */
    if (em->state == EDGE_MOTION_TRANSLATING)
    {
        if (chassis_motion_failed(em->chassis))
        {
            edge_fail(em);
            return;
        }

        if (chassis_motion_done(em->chassis))
        {
            em->state = EDGE_MOTION_SETTLE_AFTER_TRANSLATE;
            em->settle_until_ms = now_ms + EDGE_MOTION_YAW_SETTLE_MS;
        }
        return;
    }

    /* 2) 平移完成后先等车体/IMU 稳定，再读取边末 yaw。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_TRANSLATE)
    {
        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        /*
         * IMU 暂时没有新帧时保持停车等待，不推进节点。
         * 这和 10.4 VOFA 实车逻辑一致，避免“没有航向数据也当作到边”。
         */
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
            return;

        yaw_error_deg = edge_wrap_deg(hwt101_world_yaw_deg(imu) -
                                      em->edge_heading_ref_deg);

        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG)
        {
            (void)edge_commit(em);
            return;
        }

        /*
         * 大角度通常不是普通轮间误差，而是车辆被碰动、地图方向错误或 IMU 异常。
         * 此时拒绝自动原地旋转，避免在未知姿态下继续扩大错误。
         */
        if (edge_absf(yaw_error_deg) > EDGE_MOTION_YAW_MAX_START_DEG)
        {
            edge_fail(em);
            return;
        }

        if (!edge_start_yaw_correction(em, yaw_error_deg, now_ms))
            edge_fail(em);
        return;
    }

    /* 3) 低速闭环纠偏：达到死区或穿过 0 度立即停车。 */
    if (em->state == EDGE_MOTION_YAW_CORRECTING)
    {
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
        {
            edge_fail(em);
            return;
        }

        yaw_error_deg = edge_wrap_deg(hwt101_world_yaw_deg(imu) -
                                      em->edge_heading_ref_deg);
        yaw_sign = edge_signf(yaw_error_deg);

        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG ||
            (yaw_sign != 0 && yaw_sign != em->yaw_start_sign))
        {
            edge_stop_yaw_velocity(em);
            em->state = EDGE_MOTION_SETTLE_AFTER_YAW;
            em->settle_until_ms = 0U; /* 等 STOP 真正完成后再开始 150 ms 稳定计时。 */
            return;
        }

        if ((uint32_t)(now_ms - em->yaw_correct_start_ms) >
            EDGE_MOTION_YAW_CORRECT_TIMEOUT_MS)
        {
            edge_fail(em);
            return;
        }

        /*
         * 刷新 Chassis 速度看门狗；若主循环/EdgeMotion 卡死，不刷新后底盘会自行 STOP。
         */
        if ((int32_t)(now_ms - em->next_yaw_refresh_ms) >= 0)
        {
            float wz_mag = edge_yaw_wz_for_rpm(em->chassis,
                                               EDGE_MOTION_YAW_CORRECT_RPM);
            float wz_cmd = (em->yaw_start_sign > 0) ? wz_mag : -wz_mag;

            chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);
            em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
        }
        return;
    }

    /* 4) 纠偏停车完成后再稳定一次，随后才真正提交当前节点。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_YAW)
    {
        if (chassis_motion_busy(em->chassis))
            return;

        if (em->settle_until_ms == 0U)
        {
            em->settle_until_ms = now_ms + EDGE_MOTION_YAW_SETTLE_MS;
            return;
        }

        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
            return;

        (void)edge_commit(em);
        return;
    }

    /* 5) IDLE：Navigation 处于 RUNNING 时领取下一条待执行拓扑边。 */
    if (em->state != EDGE_MOTION_IDLE || nav_state != NAV_STATE_RUNNING)
        return;

    if (HMI_Navigation_GetCurrentEdge(em->hmi_nav,
                                      &em->from_node, &em->to_node))
    {
        float dx;
        float dy;

        if (!node_ok(em->from_node) || !node_ok(em->to_node) ||
            !MapGraph_IsConnected(em->from_node, em->to_node))
        {
            edge_fail(em);
            return;
        }

        /*
         * 正式 Mission 使用整场唯一世界零点，绝不在每条边重新清零 HWT101。
         * 本条边只记录“开始时的世界航向”作为保持参考。
         */
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        em->edge_heading_ref_deg = hwt101_world_yaw_deg(imu);

        dx = g_nodes[em->to_node].x - g_nodes[em->from_node].x;
        dy = g_nodes[em->to_node].y - g_nodes[em->from_node].y;
        em->world_dx_m = dx;
        em->world_dy_m = dy;

        edge_world_to_body(dx, dy, em->edge_heading_ref_deg,
                           &em->body_dx_m, &em->body_dy_m);

        edge_save_acceleration(em);
        em->chassis->acceleration = EDGE_MOTION_MOVE_ACC;

        chassis_motion_translate(em->chassis,
                                 em->body_dx_m,
                                 em->body_dy_m,
                                 ROUTE_EDGE_SPEED_MPS,
                                 now_ms);
        em->state = EDGE_MOTION_TRANSLATING;
    }
}

void EdgeMotion_EStop(EdgeMotion_t *em)
{
    if (!em || !em->chassis) return;

    edge_stop_yaw_velocity(em);
    chassis_motion_stop(em->chassis);
    edge_restore_acceleration(em);
    em->state = EDGE_MOTION_ERROR;
}

EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em)
{
    return em ? em->state : EDGE_MOTION_ERROR;
}

uint8_t EdgeMotion_IsActive(const EdgeMotion_t *em)
{
    if (!em) return 0U;

    return (em->state == EDGE_MOTION_TRANSLATING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_TRANSLATE ||
            em->state == EDGE_MOTION_YAW_CORRECTING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_YAW) ? 1U : 0U;
}
