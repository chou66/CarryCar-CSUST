#include "edge_motion.h"
#include "app_uart.h"
#include "chassis_motion.h"
#include "edge_turn.h"
#include "hwt101_uart.h"
#include "navigation.h"
#include "robot_config.h"

/*
 * 平移加速度、死区和稳停时间继承 VOFA 导航实车调试值；
 * 航向纠偏已升级为独立 PD 控制层，不再使用固定 20 RPM。
 */
#define EDGE_MOTION_MOVE_ACC                   100U
#define EDGE_MOTION_YAW_DEADBAND_DEG          0.50f
#define EDGE_MOTION_YAW_CORRECT_MAX_RPM         80U
#define EDGE_MOTION_YAW_CORRECT_MIN_RPM          8U
#define EDGE_MOTION_YAW_CORRECT_ACC             10U
#define EDGE_MOTION_YAW_SETTLE_MS              150U
#define EDGE_MOTION_IMU_TIMEOUT_MS             300U

/*
 * 世界角误差 -> 角速度的 PD 参数。
 * kp 单位约为 (deg/s)/deg；kd 用世界角速度抑制接近目标时的惯性超调。
 */
#define EDGE_MOTION_YAW_KP                     1.80f
#define EDGE_MOTION_YAW_KD                     0.35f

/* 大角度允许更长纠偏时间，但仍保留失控/机械卡死保护。 */
#define EDGE_MOTION_YAW_TIMEOUT_BASE_MS       1500U
#define EDGE_MOTION_YAW_TIMEOUT_PER_DEG_MS      35U
#define EDGE_MOTION_YAW_TIMEOUT_MAX_MS        9000U

/*
 * Chassis 速度模式带 150 ms 软件看门狗。
 * 航向修正期间每 40 ms 刷新一次速度命令并重新计算 PD 输出：
 *   - 正常运行时不会触发看门狗；
 *   - 如果 EdgeMotion_Update() 因程序异常停止执行，底盘仍会在约 150 ms 后自行停车。
 */
#define EDGE_MOTION_YAW_REFRESH_MS               40U

#define EDGE_MOTION_TWO_PI_F 6.28318530717958647692f

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
    em->yaw_correct_deadline_ms = 0U;
    em->next_yaw_refresh_ms = 0U;
    em->yaw_start_sign = 0;
    em->yaw_velocity_active = 0U;
    em->yaw_command_wz_radps = 0.0f;
    em->turn_attempt = 0U;
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
    em->yaw_command_wz_radps = 0.0f;
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

static uint32_t edge_yaw_timeout_ms(float yaw_error_deg)
{
    uint32_t timeout_ms = EDGE_MOTION_YAW_TIMEOUT_BASE_MS +
        (uint32_t)(edge_absf(yaw_error_deg) *
                   (float)EDGE_MOTION_YAW_TIMEOUT_PER_DEG_MS);

    if (timeout_ms > EDGE_MOTION_YAW_TIMEOUT_MAX_MS)
        timeout_ms = EDGE_MOTION_YAW_TIMEOUT_MAX_MS;
    return timeout_ms;
}

/*
 * 启动/重新启动边末航向修正。
 *
 * 不再用固定 RPM。heading_control 根据角度误差和世界角速度生成本周期 wz：
 * 大误差时最高 80 RPM，接近目标后逐步降到 8 RPM，并用 gyro 项提前制动。
 */
static uint8_t edge_start_yaw_correction(EdgeMotion_t *em,
                                         const Hwt101 *imu,
                                         uint32_t now_ms)
{
    float current_yaw_deg;
    float yaw_error_deg;
    float world_gyro_dps;
    float wz_cmd;

    if (!em || !em->chassis || !imu)
        return 0U;

    current_yaw_deg = hwt101_world_yaw_deg(imu);
    yaw_error_deg = EdgeTurn_WrapDeg(current_yaw_deg -
                                     em->edge_heading_ref_deg);
    world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;

    heading_set_target(&em->yaw_controller, em->edge_heading_ref_deg);
    wz_cmd = heading_update_world_cw(&em->yaw_controller,
                                     current_yaw_deg,
                                     world_gyro_dps,
                                     (float)EDGE_MOTION_YAW_REFRESH_MS / 1000.0f);
    if (wz_cmd == 0.0f)
        return 0U;

    em->chassis->acceleration = EDGE_MOTION_YAW_CORRECT_ACC;
    chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    em->yaw_start_sign = edge_signf(yaw_error_deg);
    if (em->yaw_correct_deadline_ms == 0U)
    {
        em->yaw_correct_start_ms = now_ms;
        em->yaw_correct_deadline_ms =
            now_ms + edge_yaw_timeout_ms(yaw_error_deg);
    }
    em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
    em->yaw_velocity_active = 1U;
    em->yaw_command_wz_radps = wz_cmd;
    em->state = EDGE_MOTION_YAW_CORRECTING;
    return 1U;
}

/*
 * 根据 edge_turn 的纯计算结果启动下一动作。
 *
 * 需要转向：下发最短位置转角，等待状态机复查 IMU。
 * 已对准：不再做世界->车身横移换算，固定 dx=0；dy由决策层选择
 *         +边长（前进）或-边长（后退），从执行层保证整条拓扑边只沿
 *         车身Y轴行驶，既杜绝大距离横移，也避免不必要的180°转向。
 */
static uint8_t edge_apply_turn_plan(EdgeMotion_t *em,
                                    float current_world_yaw_deg,
                                    uint32_t now_ms)
{
    EdgeTurnPlan_t plan;
    EdgeTurnOptions_t options;
    uint8_t navigation_target;

    if (!em || !em->chassis || !em->hmi_nav)
        return 0U;

    options.allow_reverse = ROUTE_ALLOW_REVERSE;
    options.reverse_min_turn_saving_deg = ROUTE_REVERSE_MIN_TURN_SAVING_DEG;
    options.arrival_yaw_valid = 0U;
    options.arrival_world_yaw_deg = 0.0f;

    /*
     * 只有to_node恰好是本次导航最终目标时，task_yaw才属于这一次到站动作。
     * 路径只是经过某个功能节点时不能套用其任务航向，否则会平白多转一次。
     */
    navigation_target = Navigation_GetTargetNode(&em->hmi_nav->nav);
    if (em->to_node == navigation_target &&
        navigation_target >= 1U && navigation_target <= NODE_NUM &&
        g_nodes[navigation_target].task_yaw_valid)
    {
        options.arrival_yaw_valid = 1U;
        options.arrival_world_yaw_deg = g_nodes[navigation_target].task_yaw_deg;
    }

    if (!EdgeTurn_Plan(em->from_node,
                       em->to_node,
                       current_world_yaw_deg,
                       ROUTE_TURN_TOLERANCE_DEG,
                       &options,
                       &plan))
        return 0U;

    em->world_dx_m = plan.world_dx_m;
    em->world_dy_m = plan.world_dy_m;
    em->body_dx_m = 0.0f;
    em->body_dy_m = plan.body_dy_m;
    em->edge_heading_ref_deg = plan.target_world_yaw_deg;
    em->turn_world_error_deg = plan.world_yaw_error_deg;
    em->turn_chassis_command_deg = plan.chassis_turn_deg;
    em->drive_backward = plan.drive_backward;

    if (plan.turn_required)
    {
        if (em->turn_attempt >= ROUTE_TURN_MAX_ATTEMPTS)
            return 0U;

        em->chassis->acceleration = ROUTE_TURN_ACC;
        chassis_motion_rotate_deg(em->chassis,
                                  plan.chassis_turn_deg,
                                  ROUTE_TURN_SPEED_MPS,
                                  now_ms);
        em->turn_attempt++;
        em->state = EDGE_MOTION_TURNING_TO_EDGE;
        return 1U;
    }

    em->chassis->acceleration = EDGE_MOTION_MOVE_ACC;
    chassis_motion_translate(em->chassis,
                             0.0f,
                             plan.body_dy_m,
                             ROUTE_EDGE_SPEED_MPS,
                             now_ms);
    em->state = EDGE_MOTION_TRANSLATING;
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
    em->turn_world_error_deg = 0.0f;
    em->turn_chassis_command_deg = 0.0f;
    em->drive_backward = 0U;
    em->turn_attempt = 0U;
    heading_init(&em->yaw_controller,
                 EDGE_MOTION_YAW_KP,
                 EDGE_MOTION_YAW_KD,
                 edge_yaw_wz_for_rpm(chassis,
                                     EDGE_MOTION_YAW_CORRECT_MAX_RPM),
                 EDGE_MOTION_YAW_DEADBAND_DEG);
    heading_set_min_output(&em->yaw_controller,
                           edge_yaw_wz_for_rpm(chassis,
                                               EDGE_MOTION_YAW_CORRECT_MIN_RPM));
    edge_clear_runtime(em);
}

void EdgeMotion_BeginRoute(EdgeMotion_t *em)
{
    if (!em)
        return;

    /* 每条边领取时都会重新计算；这里只清理上一路线的观察值。 */
    em->edge_heading_ref_deg = 0.0f;
    em->turn_world_error_deg = 0.0f;
    em->turn_chassis_command_deg = 0.0f;
    em->drive_backward = 0U;
    em->turn_attempt = 0U;
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

    /* 1) 正在把车头转到当前拓扑边方向。 */
    if (em->state == EDGE_MOTION_TURNING_TO_EDGE)
    {
        if (chassis_motion_failed(em->chassis))
        {
            edge_fail(em);
            return;
        }

        if (chassis_motion_done(em->chassis))
        {
            em->state = EDGE_MOTION_SETTLE_AFTER_TURN;
            em->settle_until_ms = now_ms + ROUTE_TURN_SETTLE_MS;
        }
        return;
    }

    /* 2) 转向后稳停并复查；有残差则最多补转 ROUTE_TURN_MAX_ATTEMPTS 次。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_TURN)
    {
        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        if (!edge_apply_turn_plan(em, hwt101_world_yaw_deg(imu), now_ms))
            edge_fail(em);
        return;
    }

    /* 3) 正在执行当前拓扑边的纯纵向位置运动（可能前进，也可能后退）。 */
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

    /* 4) 纵向平移完成后先等车体/IMU稳定，再读取边末yaw。 */
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

        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
                                         em->edge_heading_ref_deg);

        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG)
        {
            (void)edge_commit(em);
            return;
        }

        /* 任意角度都进入闭环纠偏；机械卡死仍由动态超时保护。 */
        if (!edge_start_yaw_correction(em, imu, now_ms))
            edge_fail(em);
        return;
    }

    /* 5) 自适应闭环纠偏：远处高速、近处降速，达到死区/穿零立即停车。 */
    if (em->state == EDGE_MOTION_YAW_CORRECTING)
    {
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
        {
            edge_fail(em);
            return;
        }

        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
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

        if ((int32_t)(now_ms - em->yaw_correct_deadline_ms) >= 0)
        {
            edge_fail(em);
            return;
        }

        /*
         * 刷新 Chassis 速度看门狗；若主循环/EdgeMotion 卡死，不刷新后底盘会自行 STOP。
         */
        if ((int32_t)(now_ms - em->next_yaw_refresh_ms) >= 0)
        {
            float world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
            float wz_cmd = heading_update_world_cw(
                &em->yaw_controller,
                hwt101_world_yaw_deg(imu),
                world_gyro_dps,
                (float)EDGE_MOTION_YAW_REFRESH_MS / 1000.0f);

            chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);
            em->yaw_command_wz_radps = wz_cmd;
            em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
        }
        return;
    }

    /* 6) 纠偏停车完成后再稳定一次，随后才真正提交当前节点。 */
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

        /*
         * STOP 后必须重新验角。若惯性造成超调，不提交节点，而是反向小速补偿；
         * 只有真正落入死区才允许 Navigation 前进。
         */
        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
                                         em->edge_heading_ref_deg);
        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG)
        {
            (void)edge_commit(em);
            return;
        }

        if ((int32_t)(now_ms - em->yaw_correct_deadline_ms) >= 0 ||
            !edge_start_yaw_correction(em, imu, now_ms))
            edge_fail(em);
        return;
    }

    /* 7) IDLE：Navigation 处于 RUNNING 时领取下一条待执行拓扑边。 */
    if (em->state != EDGE_MOTION_IDLE || nav_state != NAV_STATE_RUNNING)
        return;

    if (HMI_Navigation_GetCurrentEdge(em->hmi_nav,
                                      &em->from_node, &em->to_node))
    {
        float current_yaw_deg;

        /* 正式 Mission 使用整场唯一世界零点，绝不在每条边重新清零 HWT101。 */
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        current_yaw_deg = hwt101_world_yaw_deg(imu);
        edge_save_acceleration(em);
        em->turn_attempt = 0U;

        if (!edge_apply_turn_plan(em, current_yaw_deg, now_ms))
            edge_fail(em);
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

    return (em->state == EDGE_MOTION_TURNING_TO_EDGE ||
            em->state == EDGE_MOTION_SETTLE_AFTER_TURN ||
            em->state == EDGE_MOTION_TRANSLATING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_TRANSLATE ||
            em->state == EDGE_MOTION_YAW_CORRECTING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_YAW) ? 1U : 0U;
}
