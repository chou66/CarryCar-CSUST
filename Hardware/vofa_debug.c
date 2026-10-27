#include "vofa_debug.h"
#include "usart.h"
#include "app_uart.h"
#include "hwt101_uart.h"
#include "can.h"
#include "emm_v5.h"
#include "servo.h"
#include "chassis.h"
#include "chassis_config.h"
#include "chassis_motion.h"
#include "navigation.h"
#include "map_graph.h"
#include "robot_config.h"
#include "robot_app.h"
#include "k230.h"
#include "manipulator_action.h"
#include "rough_calibration.h"
#include "storage_calibration.h"

#define RX_LEN 7U
#define MANIP_TEST_RPM 3000U
#define MANIP_TEST_ACC 200U

typedef union {
    struct {
        float ch[10];
        uint32_t tail;
    } p;
    uint8_t raw[44];
} VofaPacket;

static volatile uint8_t rx_i;
static uint8_t rx[RX_LEN];
static volatile uint8_t cmd_ready;
static volatile uint8_t cmd_v, p1_v, p2_v, p3_v, p4_v;
static uint8_t last_cmd, last_result;
static uint32_t last_tx_ms;
static VofaPacket txp;
static uint8_t k230_telemetry_enable;

/*
 * ==================== Keil Watch 直接标定面板 ====================
 *
 * 这些变量不是“当前状态镜像”，而是一组可在 Keil Watch 中
 * 直接操作的调试按钮和参数。设计上特意将“参数”与“执行”分开：
 *
 *   1. 先修改 cal_dbg_ring / *_deg / *_mm 目标值；
 *   2. 再把对应的 *_req 改成 1；
 *   3. 主循环只执行一次，并立即把 *_req 自动清回 0。
 *
 * 这样即使 Watch 窗口持续刷新，也不会重复向电机下发同一动作。
 * 所有 Watch 请求最终仍转换成原 VOFA 命令，统一走 process_cmd()，
 * 因此原有的 RobotApp 接管、状态机忙判断、参数范围和低位互锁
 * 全部保留，不会因为从 Watch 调参而绕过安全检查。
 */
volatile uint8_t cal_dbg_takeover_req;
volatile uint8_t cal_dbg_rough_start_req;
volatile uint8_t cal_dbg_storage_start_req;
volatile uint8_t cal_dbg_home_confirm_req;
volatile uint8_t cal_dbg_ring;
volatile uint8_t cal_dbg_select_ring_req;
volatile float cal_dbg_carousel_deg;
volatile uint8_t cal_dbg_apply_carousel_req;
volatile float cal_dbg_slide_mm;
volatile uint8_t cal_dbg_apply_slide_req;
volatile float cal_dbg_drop_mm;
volatile uint8_t cal_dbg_apply_drop_req;
/*
 * 观察高度和放置高度分开保存，二者都是“从丝杆最高原点累计向下”的
 * 绝对距离，不是从当前位置继续下降的相对距离。
 */
volatile float cal_dbg_observe_drop_mm;
volatile float cal_dbg_place_drop_mm;
volatile uint8_t cal_dbg_apply_observe_drop_req;
volatile uint8_t cal_dbg_apply_place_drop_req;
volatile uint8_t cal_dbg_raise_only_req;
volatile uint8_t cal_dbg_slide_retract_only_req;
/* 正数读回：当前累计下降量；last_delta正数=本次向下，负数=本次向上。 */
volatile float cal_dbg_current_drop_mm;
volatile float cal_dbg_last_gimbal_delta_mm;
volatile uint8_t cal_dbg_retract_req;
volatile uint8_t cal_dbg_k230_start_req;
volatile uint8_t cal_dbg_k230_stop_req;
volatile uint8_t cal_dbg_storage_align_req;
volatile uint8_t cal_dbg_chassis_align_req;
volatile uint8_t cal_dbg_abort_req;
volatile uint8_t cal_dbg_estop_req;
volatile uint8_t cal_dbg_last_action;
volatile uint8_t cal_dbg_last_result;
volatile uint32_t cal_dbg_execute_count;

/* process_cmd()中的0x8C~0x8F会读取Watch浮点参数，实现放在文件后部。 */
static uint8_t cal_dbg_float_to_deci(float value, uint16_t *deci);

/* Independent chassis object used only by this bench/debug build.
 * It validates the same chassis_motion -> chassis -> mecanum stack
 * that navigation will use later.
 */
static Chassis s_debug_chassis;
static uint8_t formal_motion_active;

/*
 * Navigation bench objects.
 * This stage intentionally bypasses HMI only; it DOES use the real:
 *   MapGraph -> Dijkstra -> Navigation -> chassis_motion -> chassis -> CAN
 * path that the final project depends on.
 */
static Navigation_t s_debug_nav;
static uint8_t debug_nav_initialized;
static uint8_t debug_edge_active;

/* Edge execution phases. */
#define DEBUG_EDGE_IDLE                    0U
#define DEBUG_EDGE_TRANSLATING             1U
#define DEBUG_EDGE_SETTLE_AFTER_TRANSLATE  2U
#define DEBUG_EDGE_YAW_CORRECTING          3U
#define DEBUG_EDGE_SETTLE_AFTER_YAW        4U

static uint8_t debug_edge_phase;
static uint32_t debug_settle_until_ms;
static uint32_t debug_yaw_correct_start_ms;
static int8_t debug_yaw_start_sign;
static uint8_t debug_yaw_velocity_active;

static uint8_t valid_id(uint8_t id)
{
    return (id >= 1U && id <= 6U) ? 1U : 0U;
}

/*
 * 0x82成功选中新圆环后，把Watch调参值同步为该圆环的当前配置。
 *
 * cal_dbg_slide_mm会在选环后装入该环“夹爪落点”的基础值。如果
 * 操作者选中1号后忘记先改这个Watch值，0x8D会合法地伸到2号
 * 旧长度，看起来就像“自己回了2号位”。因此只在0x82真正被状态机
 * 接受后才自动装入新圆环的候选值；82被拒绝时保留原值，不伪装切换成功。
 */
static void cal_dbg_load_ring_defaults(uint8_t storage_area, uint8_t ring)
{
    cal_dbg_ring = ring;

    if (storage_area)
    {
        if (ring == 1U)
        {
            cal_dbg_carousel_deg = STORAGE_CAROUSEL_POS1_DEG;
            cal_dbg_slide_mm = STORAGE_SLIDE_POS1_MM;
        }
        else if (ring == 2U)
        {
            cal_dbg_carousel_deg = STORAGE_CAROUSEL_POS2_DEG;
            cal_dbg_slide_mm = STORAGE_SLIDE_POS2_MM;
        }
        else
        {
            cal_dbg_carousel_deg = STORAGE_CAROUSEL_POS3_DEG;
            cal_dbg_slide_mm = STORAGE_SLIDE_POS3_MM;
        }
        cal_dbg_drop_mm = STORAGE_VISION_PRE_DROP_MM;
        cal_dbg_observe_drop_mm = STORAGE_VISION_PRE_DROP_MM;
        /* 暂存区与粗加工区统一：185 mm是已验证的第一层绝对放置高度。 */
        cal_dbg_place_drop_mm = -MANIP_PICKUP_GROUND_MM;
    }
    else
    {
        if (ring == 1U)
        {
            cal_dbg_carousel_deg = ROUGH_CAROUSEL_POS1_DEG;
            cal_dbg_slide_mm = ROUGH_SLIDE_POS1_MM;
        }
        else if (ring == 2U)
        {
            cal_dbg_carousel_deg = ROUGH_CAROUSEL_POS2_DEG;
            cal_dbg_slide_mm = ROUGH_SLIDE_POS2_MM;
        }
        else
        {
            cal_dbg_carousel_deg = ROUGH_CAROUSEL_POS3_DEG;
            cal_dbg_slide_mm = ROUGH_SLIDE_POS3_MM;
        }
        cal_dbg_drop_mm = ROUGH_VISION_PRE_DROP_MM;
        cal_dbg_observe_drop_mm = ROUGH_VISION_PRE_DROP_MM;
        /* 粗加工正式抓放高度185 mm已在旧跑图中验证，可直接作为候选。 */
        cal_dbg_place_drop_mm = -MANIP_PICKUP_GROUND_MM;
    }
}

/*
 * 0x8C~0x91把P1作为“操作者认为当前选中的圆环号”。
 * 必须同时等于状态机内部的selected_ring才允许动作，防止0x82因回收未完成
 * 被拒绝后，下一条命令仍在旧的2号位上执行。P1不再允许填0；这是一道
 * 专门针对实车误操作增加的交叉校验，不依赖Watch显示是否及时刷新。
 */
static uint8_t cal_dbg_selected_ring_matches(uint8_t ring)
{
    if (ring < 1U || ring > 3U)
        return 0U;

    if (StorageCalibration_IsActive())
        return StorageCalibration_GetSelectedRing() == ring ? 1U : 0U;

    if (RoughCalibration_IsActive())
        return RoughCalibration_GetSelectedRing() == ring ? 1U : 0U;

    return 0U;
}

static uint8_t k230_direction_code(char direction)
{
    switch (direction)
    {
        case 'a': return 1U;
        case 'd': return 2U;
        case 'w': return 3U;
        case 's': return 4U;
        default:  return 0U;
    }
}

static void stop_all(void)
{
    uint8_t id;
    for (id = 1U; id <= 6U; ++id)
        Emm_V5_Stop_Now(id, false);
    hwt101_set_platform_moving(AppUart_GetImu(), false);
}


static void chassis_enable(uint8_t enable)
{
    uint8_t id;

    for (id = 1U; id <= 4U; ++id) {
        Emm_V5_En_Control(id, enable ? true : false, false);
        HAL_Delay(3U);
    }
}

/*
 * Send all four wheel velocity commands with the synchronization flag set,
 * then issue one synchronous-start command.  This prevents the first wheel
 * from starting noticeably earlier than the fourth wheel during mecanum tests.
 */
static void chassis_velocity_sync(uint8_t d1, uint8_t d2,
                                  uint8_t d3, uint8_t d4,
                                  uint16_t rpm, uint8_t acc)
{
    if (rpm > VOFA_CHASSIS_MAX_RPM)
        rpm = VOFA_CHASSIS_MAX_RPM;

    hwt101_set_platform_moving(AppUart_GetImu(), rpm != 0U);

    Emm_V5_Vel_Control(1U, d1, rpm, acc, true);
    HAL_Delay(3U);
    Emm_V5_Vel_Control(2U, d2, rpm, acc, true);
    HAL_Delay(3U);
    Emm_V5_Vel_Control(3U, d3, rpm, acc, true);
    HAL_Delay(3U);
    Emm_V5_Vel_Control(4U, d4, rpm, acc, true);
    HAL_Delay(3U);

    Emm_V5_Synchronous_motion(0U);
}

static void chassis_stop_sync(void)
{
    Emm_V5_Stop_Now(1U, true);
    HAL_Delay(3U);
    Emm_V5_Stop_Now(2U, true);
    HAL_Delay(3U);
    Emm_V5_Stop_Now(3U, true);
    HAL_Delay(3U);
    Emm_V5_Stop_Now(4U, true);
    HAL_Delay(3U);

    Emm_V5_Synchronous_motion(0U);
    hwt101_set_platform_moving(AppUart_GetImu(), false);
}


static float debug_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int8_t debug_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

/*
 * Commit the just-finished physical edge to Navigation only AFTER
 * translation and optional HWT101 heading correction are complete.
 */
static void debug_commit_edge(void)
{
    debug_edge_active = 0U;
    debug_edge_phase = DEBUG_EDGE_IDLE;

    if (!Navigation_EdgeReached(&s_debug_nav))
    {
        Navigation_Abort(&s_debug_nav);
        last_result = 7U;
        return;
    }

    if (Navigation_IsDone(&s_debug_nav))
        last_result = 6U;
}

static uint8_t debug_nav_is_running(void)
{
    return (debug_nav_initialized &&
            Navigation_GetState(&s_debug_nav) == NAV_STATE_RUNNING) ? 1U : 0U;
}

static void debug_nav_abort(void)
{
    if (debug_yaw_velocity_active)
    {
        chassis_stop_sync();
        debug_yaw_velocity_active = 0U;
    }

    if (debug_nav_initialized)
        Navigation_Abort(&s_debug_nav);

    debug_edge_active = 0U;
    debug_edge_phase = DEBUG_EDGE_IDLE;
    debug_settle_until_ms = 0U;
    debug_yaw_correct_start_ms = 0U;
    debug_yaw_start_sign = 0;
}

/*
 * Execute one Dijkstra edge at a time.
 *
 * Map coordinates:
 *   +X = right
 *   +Y = forward
 *
 * Example (measured node coordinates, 2026-10-01):
 *   1 -> 2 : dx=+0.85, dy=0      (strafe right)
 *   1 -> 4 : dx=0,     dy=+0.85  (forward)
 */
static void debug_navigation_update(uint32_t now_ms)
{
    uint8_t from_node;
    uint8_t to_node;
    float dx;
    float dy;
    float yaw_error_deg;
    int8_t yaw_sign;
    Hwt101 *imu = AppUart_GetImu();

    if (!debug_nav_initialized || formal_motion_active)
        return;

    if (debug_edge_active)
    {
        if (debug_edge_phase == DEBUG_EDGE_TRANSLATING)
        {
            if (chassis_motion_failed(&s_debug_chassis))
            {
                debug_nav_abort();
                last_result = 8U;
                return;
            }

            if (chassis_motion_done(&s_debug_chassis))
            {
                debug_edge_phase = DEBUG_EDGE_SETTLE_AFTER_TRANSLATE;
                debug_settle_until_ms = now_ms + VOFA_NAV_YAW_SETTLE_MS;
            }

            return;
        }

        if (debug_edge_phase == DEBUG_EDGE_SETTLE_AFTER_TRANSLATE)
        {
            if ((int32_t)(now_ms - debug_settle_until_ms) < 0)
                return;

            if (!hwt101_is_fresh(imu, now_ms, VOFA_NAV_IMU_TIMEOUT_MS))
            {
                last_result = 9U;
                return;
            }

            yaw_error_deg = hwt101_yaw_relative_deg(imu);

            if (debug_absf(yaw_error_deg) <= VOFA_NAV_YAW_DEADBAND_DEG)
            {
                debug_commit_edge();
                return;
            }

            /*
             * A very large error usually means the robot was physically
             * disturbed or IMU data is wrong. Do not start an uncontrolled
             * correction in that condition.
             */
            if (debug_absf(yaw_error_deg) > VOFA_NAV_YAW_MAX_START_DEG)
            {
                debug_nav_abort();
                last_result = 11U;
                return;
            }

            debug_yaw_start_sign = debug_signf(yaw_error_deg);
            debug_yaw_correct_start_ms = now_ms;

            /*
             * Positive relative yaw means the chassis drifted CCW,
             * so correct CW: all EMM direction bits = 0.
             *
             * Negative relative yaw means the chassis drifted CW,
             * so correct CCW: all EMM direction bits = 1.
             */
            if (yaw_error_deg > 0.0f)
            {
                chassis_velocity_sync(0U, 0U, 0U, 0U,
                                      VOFA_NAV_YAW_CORRECT_RPM,
                                      VOFA_NAV_YAW_CORRECT_ACC);
            }
            else
            {
                chassis_velocity_sync(1U, 1U, 1U, 1U,
                                      VOFA_NAV_YAW_CORRECT_RPM,
                                      VOFA_NAV_YAW_CORRECT_ACC);
            }

            debug_yaw_velocity_active = 1U;
            debug_edge_phase = DEBUG_EDGE_YAW_CORRECTING;
            last_result = 10U;
            return;
        }

        if (debug_edge_phase == DEBUG_EDGE_YAW_CORRECTING)
        {
            /*
             * Never allow open-loop rotation to continue if IMU frames stop.
             */
            if (!hwt101_is_fresh(imu, now_ms, VOFA_NAV_IMU_TIMEOUT_MS))
            {
                if (debug_yaw_velocity_active)
                {
                    chassis_stop_sync();
                    debug_yaw_velocity_active = 0U;
                }

                debug_nav_abort();
                last_result = 9U;
                return;
            }

            yaw_error_deg = hwt101_yaw_relative_deg(imu);
            yaw_sign = debug_signf(yaw_error_deg);

            /*
             * Stop as soon as:
             *  - yaw enters the deadband, OR
             *  - yaw crosses through zero (prevents overshoot).
             */
            if (debug_absf(yaw_error_deg) <= VOFA_NAV_YAW_DEADBAND_DEG ||
                (yaw_sign != 0 && yaw_sign != debug_yaw_start_sign))
            {
                if (debug_yaw_velocity_active)
                {
                    chassis_stop_sync();
                    debug_yaw_velocity_active = 0U;
                }

                debug_edge_phase = DEBUG_EDGE_SETTLE_AFTER_YAW;
                debug_settle_until_ms = now_ms + VOFA_NAV_YAW_SETTLE_MS;
                return;
            }

            if ((uint32_t)(now_ms - debug_yaw_correct_start_ms) >
                VOFA_NAV_YAW_CORRECT_TIMEOUT_MS)
            {
                if (debug_yaw_velocity_active)
                {
                    chassis_stop_sync();
                    debug_yaw_velocity_active = 0U;
                }

                debug_nav_abort();
                last_result = 12U;
                return;
            }

            return;
        }

        if (debug_edge_phase == DEBUG_EDGE_SETTLE_AFTER_YAW)
        {
            if ((int32_t)(now_ms - debug_settle_until_ms) < 0)
                return;

            if (!hwt101_is_fresh(imu, now_ms, VOFA_NAV_IMU_TIMEOUT_MS))
            {
                last_result = 9U;
                return;
            }

            /*
             * Closed-loop correction has already stopped on the IMU.
             * Commit the edge now and allow Navigation to launch the next one.
             */
            debug_commit_edge();
            return;
        }

        debug_nav_abort();
        last_result = 7U;
        return;
    }

    if (Navigation_GetState(&s_debug_nav) == NAV_STATE_NO_PATH ||
        Navigation_GetState(&s_debug_nav) == NAV_STATE_ERROR)
    {
        last_result = 7U;
        return;
    }

    if (Navigation_GetState(&s_debug_nav) != NAV_STATE_RUNNING)
        return;

    if (!Navigation_GetCurrentEdge(&s_debug_nav, &from_node, &to_node))
    {
        last_result = 7U;
        Navigation_Abort(&s_debug_nav);
        return;
    }

    if (!MapGraph_IsConnected(from_node, to_node))
    {
        last_result = 7U;
        Navigation_Abort(&s_debug_nav);
        return;
    }

    if (!hwt101_is_fresh(imu, now_ms, VOFA_NAV_IMU_TIMEOUT_MS))
    {
        last_result = 9U;
        return;
    }

    /*
     * Software reference only: preserve the heading present at edge start.
     */
    hwt101_set_zero(imu);

    dx = g_nodes[to_node].x - g_nodes[from_node].x;
    dy = g_nodes[to_node].y - g_nodes[from_node].y;

    s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;

    chassis_motion_translate(&s_debug_chassis,
                             dx,
                             dy,
                             ROUTE_EDGE_SPEED_MPS,
                             now_ms);

    debug_edge_active = 1U;
    debug_edge_phase = DEBUG_EDGE_TRANSLATING;
}

/*
 * Direct CAN smoke test, independent of the chassis/kinematics layers:
 * make motor 1 rotate exactly one revolution (3200 pulses @ 16 microstep).
 *
 * Path: Emm_V5_Pos_Control -> can_SendCmd -> FDCAN2. Nothing else.
 *
 * A status query runs first: if the driver does not answer, the test
 * refuses to move and returns 0. Returns 1 only when the bus round trip
 * is proven and the one-rev position command has been queued.
 */
uint8_t Motor1_RevTest(void)
{
    uint8_t status = 0U;

    if (Emm_V5_Read_Status(2U, &status, 60U) == 0U)
        return 0U;                      /* no CAN reply: bus still dead */

    Emm_V5_En_Control(2U, true, false);
    HAL_Delay(4U);

    Emm_V5_Pos_Control(2U, 1U, 30U, 10U,
                       1U * (uint32_t)CHASSIS_MOTOR_PULSES_PER_REV, false, false);

    return 1U;
}

/*
 * Direct CAN smoke test for the slide-rail motor (CAN address MOTOR_SLIDE_ID):
 * rotate two motor revolutions, 30 rpm, acc 10.
 *
 * NOTE: pulses per revolution assume the same 16-microstep setup as the
 * wheel motors (3200/rev). If the slide driver uses a different
 * subdivision, adjust SLIDE_TEST_PULSES_PER_REV.
 *
 * Same contract as Motor1_RevTest(): a status query on id 6 runs first;
 * no reply -> returns 0 and the motor stays still. Direction 0 is an
 * arbitrary reference; flip it if the carriage runs the wrong way.
 */
#define SLIDE_TEST_PULSES_PER_REV   3200U

uint8_t Slide_RevTest(void)
{
    uint8_t status = 0U;

    if (Emm_V5_Read_Status(5U, &status, 60U) == 0U)
        return 0U;                      /* no CAN reply: id6 silent */

    Emm_V5_En_Control(5U, true, false);
    HAL_Delay(5U);

    Emm_V5_Pos_Control(5U, 0U, 30U, 10U,
                       0.5f * SLIDE_TEST_PULSES_PER_REV, false, false);

    return 1U;
}

void VofaDebug_Init(void)
{
    MecanumConfig cfg;

    rx_i = 0U;
    cmd_ready = 0U;
    last_cmd = 0U;
    last_result = 0U;
    last_tx_ms = 0U;
    k230_telemetry_enable = 0U;
    formal_motion_active = 0U;
    debug_nav_initialized = 0U;
    debug_edge_active = 0U;
    debug_edge_phase = DEBUG_EDGE_IDLE;
    debug_settle_until_ms = 0U;
    debug_yaw_correct_start_ms = 0U;
    debug_yaw_start_sign = 0;
    debug_yaw_velocity_active = 0U;
    txp.p.tail = 0x7F800000UL; /* 00 00 80 7F on Cortex-M */

    /*
     * Watch 标定面板上电默认值。
     * 任意 *_req=0 都表示“不执行”，因此上电不会自行转动。
     * 默认数值选择当前低位标定的2号环常用起点，仅用于减少
     * Watch 手工输入；仍必须由操作者将对应 *_req 写 1 才会动作。
     */
    cal_dbg_takeover_req = 0U;
    cal_dbg_rough_start_req = 0U;
    cal_dbg_storage_start_req = 0U;
    cal_dbg_home_confirm_req = 0U;
    cal_dbg_ring = 2U;
    cal_dbg_select_ring_req = 0U;
    cal_dbg_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
    cal_dbg_apply_carousel_req = 0U;
    cal_dbg_slide_mm = ROUGH_VISION_LOW_SLIDE_POS2_MM;
    cal_dbg_apply_slide_req = 0U;
    cal_dbg_drop_mm = ROUGH_VISION_PRE_DROP_MM;
    cal_dbg_apply_drop_req = 0U;
    cal_dbg_observe_drop_mm = ROUGH_VISION_PRE_DROP_MM;
    cal_dbg_place_drop_mm = -MANIP_PICKUP_GROUND_MM;
    cal_dbg_apply_observe_drop_req = 0U;
    cal_dbg_apply_place_drop_req = 0U;
    cal_dbg_raise_only_req = 0U;
    cal_dbg_slide_retract_only_req = 0U;
    cal_dbg_current_drop_mm = 0.0f;
    cal_dbg_last_gimbal_delta_mm = 0.0f;
    cal_dbg_retract_req = 0U;
    cal_dbg_k230_start_req = 0U;
    cal_dbg_k230_stop_req = 0U;
    cal_dbg_storage_align_req = 0U;
    cal_dbg_chassis_align_req = 0U;
    cal_dbg_abort_req = 0U;
    cal_dbg_estop_req = 0U;
    cal_dbg_last_action = CAL_DBG_ACTION_NONE;
    cal_dbg_last_result = 0U;
    cal_dbg_execute_count = 0U;

    /*
     * Use the real calibrated chassis configuration.
     * This is the same motion/kinematics layer used by navigation.
     */
    chassis_config_init_mecanum(&cfg);
    chassis_init(&s_debug_chassis, &cfg);
    chassis_bind_imu_filter(&s_debug_chassis, AppUart_GetImu());
    s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;

    /*
     * 粗加工/暂存区标定共享这一个VOFA底盘对象，但各自复用正式EdgeMotion。
     * 这里只绑定对象，不会在上电时产生任何底盘或机械臂动作。
     */
    RoughCalibration_Init(&s_debug_chassis);
    StorageCalibration_Init(&s_debug_chassis);
}

void VofaDebug_FeedByte(uint8_t b)
{
    if (rx_i == 0U) {
        if (b == 0xAAU) { rx[0] = b; rx_i = 1U; }
        return;
    }

    if (rx_i == 1U) {
        if (b == 0x55U) { rx[1] = b; rx_i = 2U; }
        else {
            rx_i = (b == 0xAAU) ? 1U : 0U;
            if (rx_i == 1U) rx[0] = 0xAAU;
        }
        return;
    }

    rx[rx_i++] = b;
    if (rx_i >= RX_LEN) {
        if (!cmd_ready) {
            cmd_v = rx[2];
            p1_v = rx[3];
            p2_v = rx[4];
            p3_v = rx[5];
            p4_v = rx[6];
            cmd_ready = 1U;
        }
        rx_i = 0U;
    }
}

/*
 * 10.7.2 RobotApp + VOFA coexistence diagnostics.
 * Only commands that can change actuators require manual takeover.
 * 0x02 only changes the IMU software zero and is left available.
 */
static uint8_t vofa_cmd_needs_manual_override(uint8_t cmd)
{
    if (cmd == 0x01U) return 1U;
    if (cmd >= 0x10U && cmd <= 0x23U) return 1U;
    if (cmd >= 0x30U && cmd <= 0x60U) return 1U;
    if (cmd >= 0x80U && cmd <= 0x91U) return 1U;
    return 0U;
}

static void process_cmd(void)
{
    uint8_t c, p1, p2, p3, p4;
    uint32_t primask;
    uint16_t rpm, us, pulses;

    if (!cmd_ready) return;

    primask = __get_PRIMASK();
    __disable_irq();
    c = cmd_v; p1 = p1_v; p2 = p2_v; p3 = p3_v; p4 = p4_v;
    cmd_ready = 0U;
    if (!primask) __enable_irq();

    last_cmd = c;
    last_result = 1U;

    /*
     * 0x70 is the only control-owner switch in this diagnostic build.
     * RobotApp stops first and then remains suspended until MCU reset.
     */
    if (c == 0x70U)
    {
        RobotApp_EnterVofaOverride();
        last_result = 1U;
        return;
    }

    /*
     * 任一工位标定启动后，底盘/云台/滑台都归标定状态机独占。只放行：
     *   0x01       全局急停；
     *   0x20       夹爪开/中/关，用于完整模拟夹取和放置；
     *   0x72/0x73  K230观察命令（暂存流程到N4前均禁止，避免干扰自动对准）；
     *   0x74       切换遥测页；
     *   0x81~0x86  标定状态机自己的水平调参/回收命令；
     *   0x88       粗加工N2/暂存区N4的二号环底盘视觉位置锁定。
     *   0x8B       低位模式的电机5绝对下降量，用于验证真实落料点。
     *   0x8C~0x8F 读取Keil Watch中的十进制目标值并执行，不再需要
     *               手工把角度/毫米换算成VOFA十六进制参数。
     *   0x90/0x91 分别只抬丝杆、只收齿条，供分轴验证回程。
     * 这样可避免旧的直接电机命令和状态机同时向同一执行器下发动作。
     */
    if ((RoughCalibration_IsActive() || StorageCalibration_IsActive()) &&
        c != 0x01U &&
        c != 0x20U &&
        c != 0x74U &&
        !((c == 0x72U || c == 0x73U) &&
          (RoughCalibration_IsActive() ||
           StorageCalibration_AllowsManualVision())) &&
        c != 0x88U &&
        (c < 0x8BU || c > 0x91U) &&
        (c < 0x81U || c > 0x86U))
    {
        last_result = 17U;
        return;
    }

    /* Reject actuator commands until the user explicitly takes control. */
    if (vofa_cmd_needs_manual_override(c))
    {
        if (!RobotApp_IsVofaOverride())
        {
            last_result = 14U;
            return;
        }

        /* RobotApp is still flushing its four-wheel STOP sequence. */
        if (!RobotApp_IsVofaOverrideReady())
        {
            last_result = 15U;
            return;
        }
    }

    switch (c) {
    case 0x01U:
        RoughCalibration_Abort();
        StorageCalibration_Abort();
        chassis_motion_stop(&s_debug_chassis);
        formal_motion_active = 0U;
        debug_nav_abort();
        stop_all();
        break;

    case 0x02U:
        hwt101_set_zero(AppUart_GetImu());
        break;

    case 0x03U:
        /*
         * HWT101 零偏自校准。
         *   P1=1 开始：发送 FF AA 48 01 00，整车必须静止；到时自动发送
         *              FF AA 48 00 00 退出。
         *   P1=0 中止：立即发送退出指令。
         *   P2 = 校准时长（秒），0 表示默认 20 s，内部限制 5~120 s。
         *
         * 只允许在任务未启动/已结束，或 VOFA 已接管（RobotApp 暂停）
         * 时执行，避免边行驶边校准。
         */
        if (p1 == 0U)
        {
            AppUart_ImuZeroBiasCalAbort();
            break;
        }

        if (p1 != 1U)
        {
            last_result = 2U;
            break;
        }

        if (!RobotApp_IsVofaOverride() && !RobotApp_IsMissionIdle())
        {
            last_result = 2U;
            break;
        }

        if (!AppUart_ImuZeroBiasCalStart(HAL_GetTick(),
                                         (uint32_t)p2 * 1000U))
        {
            last_result = 2U;
        }
        break;

    case 0x10U:
        if (!valid_id(p1) || p2 > 1U) { last_result = 2U; break; }
        Emm_V5_En_Control(p1, p2 ? true : false, false);
        break;

    case 0x11U:
        if (!valid_id(p1) || p2 > 1U) { last_result = 2U; break; }
        rpm = (uint16_t)p3 * 10U;
        if (rpm > VOFA_DEBUG_MAX_RPM) rpm = VOFA_DEBUG_MAX_RPM;
        Emm_V5_Vel_Control(p1, p2, rpm, p4, false);
        break;

    case 0x12U:
        if (!valid_id(p1)) { last_result = 2U; break; }
        Emm_V5_Stop_Now(p1, false);
        break;

    case 0x13U:
        if ((p1 != MOTOR_GIMBAL_ID && p1 != MOTOR_SLIDE_ID) || p2 > 1U)
        {
            last_result = 2U;
            break;
        }

        pulses = (uint16_t)(((uint16_t)p3 << 8) | p4);
        if (pulses == 0U)
        {
            last_result = 2U;
            break;
        }

        Emm_V5_Pos_Control(p1, p2, MANIP_TEST_RPM, MANIP_TEST_ACC,
                           (uint32_t)pulses, false, false);
        break;

    case 0x20U:
        if (p1 == 0U) GripperServo_Open_Small();
        else if (p1 == 1U) GripperServo_Mid();
        else if (p1 == 2U) GripperServo_Close();
        /*
         * 正式夹取流程下降前使用大张开；原0/1/2命令编码
         * 保持不变，在末尾追加3，避免破坏已有VOFA按钮。
         */
        else if (p1 == 3U) GripperServo_Open_Large();
        else last_result = 2U;
        break;

    case 0x21U:
    if (p1 == 0U)
    {
        CarouselServo_ToPickup();
    }
    else if (p1 == 1U)
    {
        CarouselServo_ToPlace();
    }
    else if (p1 == 2U)
    {
        CarouselServo_ToMid();
    }
    else
    {
        last_result = 2U;
    }
    break;

    case 0x22U:
    us = (uint16_t)(((uint16_t)p2 << 8) | p3);

    if (p1 == 0U) {
        Pwm1Servo_SetUs(us);
    } else if (p1 == 1U) {
        CarouselServo_SetUs(us);
    } else if (p1 == 2U) {
        TraySelectorServo_SetUs(us);
    } else if (p1 == 3U) {
        GripperServo_SetUs(us);
    } else {
        last_result = 2U;
    }
    break;

		case 0x23U:
    if (p1 >= TRAY_SELECTOR_SLOT_COUNT)
    {
        last_result = 2U;
        break;
    }

    TraySelectorServo_GotoSlot(p1);
    break;
		
    case 0x30U: /* chassis enable/disable */
        if (p1 > 1U) { last_result = 2U; break; }
        chassis_enable(p1);
        break;

    case 0x31U: /* forward */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        /* ID1/2 forward=0, ID3/4 forward=1 */
        chassis_velocity_sync(0U, 0U, 1U, 1U, rpm, p2);
        break;

    case 0x32U: /* backward */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        chassis_velocity_sync(1U, 1U, 0U, 0U, rpm, p2);
        break;

    case 0x33U: /* strafe left (-X) */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        /* FL-, RL+, RR-, FR+ */
        chassis_velocity_sync(1U, 0U, 0U, 1U, rpm, p2);
        break;

    case 0x34U: /* strafe right (+X) */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        /* FL+, RL-, RR+, FR- */
        chassis_velocity_sync(0U, 1U, 1U, 0U, rpm, p2);
        break;

    case 0x35U: /* rotate counter-clockwise (+wz) */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        /* FL-, RL-, RR+, FR+ -> all four EMM direction bits are 1 */
        chassis_velocity_sync(1U, 1U, 1U, 1U, rpm, p2);
        break;

    case 0x36U: /* rotate clockwise (-wz) */
        rpm = (uint16_t)p1 * 10U;
        if (rpm == 0U) { last_result = 2U; break; }
        chassis_velocity_sync(0U, 0U, 0U, 0U, rpm, p2);
        break;

    case 0x37U: /* stop chassis */
        chassis_stop_sync();
        break;


    case 0x40U: /* formal forward by VOFA_FORMAL_DISTANCE_M */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_forward(&s_debug_chassis,
                               VOFA_FORMAL_DISTANCE_M,
                               VOFA_FORMAL_MOVE_SPEED_MPS,
                               HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x41U: /* formal backward by VOFA_FORMAL_DISTANCE_M */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_backward(&s_debug_chassis,
                                VOFA_FORMAL_DISTANCE_M,
                                VOFA_FORMAL_MOVE_SPEED_MPS,
                                HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x42U: /* formal strafe left by VOFA_FORMAL_DISTANCE_M */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_strafe_left(&s_debug_chassis,
                                   VOFA_FORMAL_DISTANCE_M,
                                   VOFA_FORMAL_MOVE_SPEED_MPS,
                                   HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x43U: /* formal strafe right by VOFA_FORMAL_DISTANCE_M */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_strafe_right(&s_debug_chassis,
                                    VOFA_FORMAL_DISTANCE_M,
                                    VOFA_FORMAL_MOVE_SPEED_MPS,
                                    HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x44U: /* formal rotate CCW 90 deg */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_rotate_ccw_deg(&s_debug_chassis,
                                      VOFA_FORMAL_ANGLE_DEG,
                                      VOFA_FORMAL_MOVE_SPEED_MPS,
                                      HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x45U: /* formal rotate CW 90 deg */
        if (chassis_motion_busy(&s_debug_chassis) || debug_nav_is_running()) {
            last_result = 2U;
            break;
        }
        s_debug_chassis.acceleration = VOFA_FORMAL_MOVE_ACC;
        chassis_motion_rotate_cw_deg(&s_debug_chassis,
                                     VOFA_FORMAL_ANGLE_DEG,
                                     VOFA_FORMAL_MOVE_SPEED_MPS,
                                     HAL_GetTick());
        formal_motion_active = 1U;
        break;

    case 0x46U: /* stop formal position motion */
        chassis_motion_stop(&s_debug_chassis);
        formal_motion_active = 0U;
        break;


    case 0x50U: /* navigation debug init/reset: P1=start node 1..9 */
        if (p1 < 1U || p1 > NODE_NUM ||
            chassis_motion_busy(&s_debug_chassis) ||
            formal_motion_active)
        {
            last_result = 2U;
            break;
        }

        if (!Navigation_Init(&s_debug_nav, p1, 0UL))
        {
            last_result = 7U;
            break;
        }

        debug_nav_initialized = 1U;
        debug_edge_active = 0U;
        break;

    case 0x51U: /* plan and physically drive to node P1 */
        if (!debug_nav_initialized ||
            p1 < 1U || p1 > NODE_NUM ||
            formal_motion_active ||
            chassis_motion_busy(&s_debug_chassis) ||
            debug_edge_active ||
            debug_nav_is_running())
        {
            last_result = 2U;
            break;
        }

        if (!Navigation_PlanTo(&s_debug_nav, p1))
        {
            last_result = 7U;
            break;
        }

        /* start == target: Dijkstra is already complete */
        if (Navigation_IsDone(&s_debug_nav))
            last_result = 6U;
        break;

    case 0x52U: /* abort navigation and stop chassis */
        if (chassis_motion_busy(&s_debug_chassis) || debug_edge_active)
            chassis_motion_stop(&s_debug_chassis);

        formal_motion_active = 0U;
        debug_nav_abort();
        break;

    case 0x53U: /* set static obstacle mask; little-endian P1..P4 */
        if (!debug_nav_initialized ||
            debug_nav_is_running() ||
            debug_edge_active ||
            chassis_motion_busy(&s_debug_chassis))
        {
            last_result = 2U;
            break;
        }
        {
            uint32_t mask =
                ((uint32_t)p1) |
                ((uint32_t)p2 << 8) |
                ((uint32_t)p3 << 16) |
                ((uint32_t)p4 << 24);

            if (!Navigation_SetObstacleMask(&s_debug_nav, mask))
                last_result = 7U;
        }
        break;

    case 0x60U: /* motor 1 one-revolution CAN smoke test (direct emm layer) */
        last_result = Motor1_RevTest() ? 1U : 13U;
        break;
		
		//k230测试
		case 0x71U: /* Start K230 color detection. */
    if ((p1 < 1U) || (p1 > 6U))
    {
        last_result = 2U;
        break;
    }

    if (K230_StartDetect(p1) != HAL_OK)
    {
        last_result = 16U;
    }
    break;

		case 0x72U: /* Start K230 placement alignment. */
				if ((p1 < 1U) || (p1 > 3U))
				{
						last_result = 2U;
						break;
				}
		
				if (K230_StartPlace(p1) != HAL_OK)
				{
						last_result = 16U;
				}
				break;
		
		case 0x73U: /* Stop current K230 task. */
				if (K230_Stop() != HAL_OK)
				{
						last_result = 16U;
				}
				break;

        case 0x74U: /* 0=常规，1=K230，2=粗加工标定，3=暂存区标定。 */
            if (p1 > 3U)
            {
                last_result = 2U;
                break;
            }
            k230_telemetry_enable = p1;
            break;

        case 0x80U: /* 旧粗加工高位识别：电机5保持最高点，自动到N2。 */
            if (formal_motion_active ||
                debug_nav_is_running() ||
                debug_edge_active ||
                chassis_motion_busy(&s_debug_chassis))
            {
                last_result = 2U;
                break;
            }

            if (!RoughCalibration_Start(p1, HAL_GetTick()))
                last_result = 18U;
            break;

        case 0x89U: /* 新粗加工低位识别：路线同0x80，0x83伸出后自动预下降。 */
            if (formal_motion_active ||
                debug_nav_is_running() ||
                debug_edge_active ||
                chassis_motion_busy(&s_debug_chassis))
            {
                last_result = 2U;
                break;
            }

            if (!RoughCalibration_StartLow(p1, HAL_GetTick()))
                last_result = 18U;
            break;

        case 0x81U: /* 人工已把滑台缩到机械原点；P1 必须为确认口令 0xA5。 */
            if (!RoughCalibration_IsActive() ||
                !RoughCalibration_ConfirmSlideHome(p1))
                last_result = 18U;
            break;

        case 0x82U: /* 选择当前标定区的1/2/3号圆环，在0 mm安全位转云台。 */
            if (StorageCalibration_IsActive())
            {
                if (!StorageCalibration_SelectRing(p1, HAL_GetTick()))
                    last_result = 18U;
                else
                    cal_dbg_load_ring_defaults(1U, p1);
            }
            else if (!RoughCalibration_IsActive() ||
                     !RoughCalibration_SelectRing(p1, HAL_GetTick()))
            {
                last_result = 18U;
            }
            else
            {
                cal_dbg_load_ring_defaults(0U, p1);
            }
            break;

        case 0x83U: /* 滑台绝对位置：P1:P2 为 0.1 mm，例 1250 => 125.0 mm。 */
            {
                uint16_t target_deci_mm =
                    (uint16_t)(((uint16_t)p1 << 8) | p2);

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveSlideDeciMm(target_deci_mm))
                        last_result = 18U;
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveSlideDeciMm(target_deci_mm))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x84U: /* 从当前可信位置退回滑台软件原点 0 mm。 */
            if (StorageCalibration_IsActive())
            {
                if (!StorageCalibration_RetractSlide())
                    last_result = 18U;
            }
            else if (!RoughCalibration_IsActive() ||
                     !RoughCalibration_RetractSlide())
            {
                last_result = 18U;
            }
            break;

        case 0x85U: /* 终止当前工位标定；下次必须重新确认机械原点。 */
            if (StorageCalibration_IsActive())
                StorageCalibration_Abort();
            else
                RoughCalibration_Abort();
            break;

        case 0x86U: /* 当前标定区云台绝对角：P1:P2为0.1 deg。 */
            {
                uint16_t target_deci_deg =
                    (uint16_t)(((uint16_t)p1 << 8) | p2);

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveCarouselDeciDeg(target_deci_deg,
                                                                 HAL_GetTick()))
                        last_result = 18U;
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveCarouselDeciDeg(target_deci_deg,
                                                               HAL_GetTick()))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x87U: /* 旧暂存区高位识别全流程；P1=A5确认齿条机械原点。 */
            if (formal_motion_active ||
                debug_nav_is_running() ||
                debug_edge_active ||
                RoughCalibration_IsActive() ||
                chassis_motion_busy(&s_debug_chassis))
            {
                last_result = 2U;
                break;
            }

            if (!StorageCalibration_StartFromBay2(p1, HAL_GetTick()))
                last_result = 18U;
            break;

        case 0x8AU: /* 新暂存区低位识别全流程；路径同0x87并复现正式预下降。 */
            if (formal_motion_active ||
                debug_nav_is_running() ||
                debug_edge_active ||
                RoughCalibration_IsActive() ||
                chassis_motion_busy(&s_debug_chassis))
            {
                last_result = 2U;
                break;
            }

            if (!StorageCalibration_StartFromBay2Low(p1, HAL_GetTick()))
                last_result = 18U;
            break;

        case 0x88U: /* 当前区二号环：保持观察位姿，K230实际平移底盘对准。 */
            if (StorageCalibration_IsActive())
            {
                if (!StorageCalibration_StartStorageVisionAlign(HAL_GetTick()))
                    last_result = 18U;
            }
            else if (RoughCalibration_IsActive())
            {
                if (!RoughCalibration_StartVisionAlign(HAL_GetTick()))
                    last_result = 18U;
            }
            else
            {
                last_result = 18U;
            }
            break;

        case 0x8BU: /* 低位落料点验证：P1:P2=从最高原点累计下降0.1 mm。 */
            {
                uint16_t target_deci_mm =
                    (uint16_t)(((uint16_t)p1 << 8) | p2);

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveGimbalDropDeciMm(
                            target_deci_mm))
                    {
                        last_result = 18U;
                    }
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveGimbalDropDeciMm(
                             target_deci_mm))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x8CU: /* P1=当前圆环；执行Watch中的cal_dbg_carousel_deg。 */
            {
                uint16_t target_deci_deg;

                if (!cal_dbg_selected_ring_matches(p1))
                {
                    last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                    break;
                }

                if (!cal_dbg_float_to_deci(cal_dbg_carousel_deg,
                                            &target_deci_deg))
                {
                    last_result = 2U;
                    break;
                }

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveCarouselDeciDeg(
                            target_deci_deg, HAL_GetTick()))
                    {
                        last_result = 18U;
                    }
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveCarouselDeciDeg(
                             target_deci_deg, HAL_GetTick()))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x8DU: /* P1=当前圆环；执行Watch中的cal_dbg_slide_mm。 */
            {
                uint16_t target_deci_mm;

                if (!cal_dbg_selected_ring_matches(p1))
                {
                    last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                    break;
                }

                if (!cal_dbg_float_to_deci(cal_dbg_slide_mm,
                                            &target_deci_mm))
                {
                    last_result = 2U;
                    break;
                }

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveSlideDeciMm(target_deci_mm))
                        last_result = 18U;
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveSlideDeciMm(target_deci_mm))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x8EU: /* P1=当前圆环；丝杆到Watch观察高度（相对最高原点）。 */
            {
                uint16_t target_deci_mm;

                if (!cal_dbg_selected_ring_matches(p1))
                {
                    last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                    break;
                }

                if (!cal_dbg_float_to_deci(cal_dbg_observe_drop_mm,
                                            &target_deci_mm))
                {
                    last_result = 2U;
                    break;
                }

                cal_dbg_current_drop_mm =
                    -ManipulatorAction_SlideJogGetVisionDropMm();
                cal_dbg_last_gimbal_delta_mm =
                    cal_dbg_observe_drop_mm - cal_dbg_current_drop_mm;

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveGimbalDropDeciMm(
                            target_deci_mm))
                    {
                        last_result = 18U;
                    }
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveGimbalDropDeciMm(
                             target_deci_mm))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x8FU: /* P1=当前圆环；丝杆到Watch放置绝对高度（两区均可直达185）。 */
            {
                uint16_t target_deci_mm;

                if (!cal_dbg_selected_ring_matches(p1))
                {
                    last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                    break;
                }

                if (!cal_dbg_float_to_deci(cal_dbg_place_drop_mm,
                                            &target_deci_mm))
                {
                    last_result = 2U;
                    break;
                }

                cal_dbg_current_drop_mm =
                    -ManipulatorAction_SlideJogGetVisionDropMm();
                cal_dbg_last_gimbal_delta_mm =
                    cal_dbg_place_drop_mm - cal_dbg_current_drop_mm;

                if (StorageCalibration_IsActive())
                {
                    if (!StorageCalibration_MoveGimbalDropDeciMm(
                            target_deci_mm))
                    {
                        last_result = 18U;
                    }
                }
                else if (!RoughCalibration_IsActive() ||
                         !RoughCalibration_MoveGimbalDropDeciMm(
                             target_deci_mm))
                {
                    last_result = 18U;
                }
            }
            break;

        case 0x90U: /* P1=当前圆环；只把丝杆抬回最高原点，齿条保持伸出。 */
            if (!cal_dbg_selected_ring_matches(p1))
            {
                last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                break;
            }

            cal_dbg_current_drop_mm =
                -ManipulatorAction_SlideJogGetVisionDropMm();
            cal_dbg_last_gimbal_delta_mm = -cal_dbg_current_drop_mm;

            if (StorageCalibration_IsActive())
            {
                if (!StorageCalibration_RaiseGimbalToTop())
                    last_result = 18U;
            }
            else if (!RoughCalibration_IsActive() ||
                     !RoughCalibration_RaiseGimbalToTop())
            {
                last_result = 18U;
            }
            break;

        case 0x91U: /* P1=当前圆环；只收齿条到0 mm，要求丝杆已经最高。 */
            if (!cal_dbg_selected_ring_matches(p1))
            {
                last_result = (p1 >= 1U && p1 <= 3U) ? 18U : 2U;
                break;
            }

            if (StorageCalibration_IsActive())
            {
                if (!StorageCalibration_RetractSlideOnly())
                    last_result = 18U;
            }
            else if (!RoughCalibration_IsActive() ||
                     !RoughCalibration_RetractSlideOnly())
            {
                last_result = 18U;
            }
            break;

    default:
        last_result = 3U;
        break;
    }
}

/*
 * 将 Watch 中输入的浮点物理量转为原 VOFA 协议使用的0.1单位整数。
 *
 * 例如：78.5 mm -> 785，205.0 deg -> 2050。
 * 利用“value >= 0”的比较同时拒绝 NaN；先做范围判断再强转，
 * 避免 Watch 误输负数或超大值时发生无符号整数回绕。
 */
static uint8_t cal_dbg_float_to_deci(float value, uint16_t *deci)
{
    if (deci == NULL || !(value >= 0.0f) || value > 6553.5f)
        return 0U;

    *deci = (uint16_t)(value * 10.0f + 0.5f);
    return 1U;
}

/*
 * 清除所有可能产生后续动作的 Watch 请求。
 * 当操作者按下“急停”或“退出标定”时，如果 Watch 里还有其他
 * *_req 刚好也被写成1，必须一并丢弃，不能在下一个主循环又启动。
 */
static void cal_dbg_clear_motion_requests(void)
{
    cal_dbg_rough_start_req = 0U;
    cal_dbg_storage_start_req = 0U;
    cal_dbg_home_confirm_req = 0U;
    cal_dbg_select_ring_req = 0U;
    cal_dbg_apply_carousel_req = 0U;
    cal_dbg_apply_slide_req = 0U;
    cal_dbg_apply_drop_req = 0U;
    cal_dbg_apply_observe_drop_req = 0U;
    cal_dbg_apply_place_drop_req = 0U;
    cal_dbg_raise_only_req = 0U;
    cal_dbg_slide_retract_only_req = 0U;
    cal_dbg_retract_req = 0U;
    cal_dbg_k230_start_req = 0U;
    cal_dbg_k230_stop_req = 0U;
    cal_dbg_storage_align_req = 0U;
    cal_dbg_chassis_align_req = 0U;
}

/*
 * 每次主循环最多消费一个 Watch 请求。
 *
 * 优先级从高到低为：急停 -> 退出 -> 接管 -> 其他标定动作。
 * 除了急停/退出会主动丢弃其他待执行请求外，普通请求只清零自己。
 * 最终仍把动作写入 cmd_v/p1_v...并调用 process_cmd()，这是保证
 * Watch 途径与 VOFA 途径安全行为完全一致的关键。
 */
static void cal_dbg_process_watch_request(void)
{
    uint8_t action = CAL_DBG_ACTION_NONE;
    uint8_t command = 0U;
    uint8_t p1 = 0U;
    uint8_t p2 = 0U;
    uint16_t deci = 0U;

    if (cal_dbg_estop_req != 0U)
    {
        cal_dbg_estop_req = 0U;
        cal_dbg_abort_req = 0U;
        cal_dbg_clear_motion_requests();
        action = CAL_DBG_ACTION_ESTOP;
        command = 0x01U;
    }
    else if (cal_dbg_abort_req != 0U)
    {
        cal_dbg_abort_req = 0U;
        cal_dbg_clear_motion_requests();
        action = CAL_DBG_ACTION_ABORT;
        command = 0x85U;
    }
    else if (cal_dbg_takeover_req != 0U)
    {
        cal_dbg_takeover_req = 0U;
        action = CAL_DBG_ACTION_TAKEOVER;
        command = 0x70U;
    }
    else if (cal_dbg_rough_start_req != 0U)
    {
        cal_dbg_rough_start_req = 0U;
        action = CAL_DBG_ACTION_ROUGH_START;
        command = 0x89U;
        /* 直接调参面板固定复现比赛路线：从启停区2到粗加工N2。 */
        p1 = ROUGH_CAL_START_BAY2;
    }
    else if (cal_dbg_storage_start_req != 0U)
    {
        cal_dbg_storage_start_req = 0U;
        action = CAL_DBG_ACTION_STORAGE_START;
        command = 0x8AU;
        /* 暂存区入口固定从启停区2出发，A5表示齿条已在机械原点。 */
        p1 = STORAGE_CAL_HOME_CONFIRM_KEY;
    }
    else if (cal_dbg_home_confirm_req != 0U)
    {
        cal_dbg_home_confirm_req = 0U;
        action = CAL_DBG_ACTION_HOME_CONFIRM;
        command = 0x81U;
        p1 = 0xA5U;
    }
    else if (cal_dbg_select_ring_req != 0U)
    {
        cal_dbg_select_ring_req = 0U;
        action = CAL_DBG_ACTION_SELECT_RING;
        command = 0x82U;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_apply_carousel_req != 0U)
    {
        cal_dbg_apply_carousel_req = 0U;
        action = CAL_DBG_ACTION_APPLY_CAROUSEL;
        if (!cal_dbg_float_to_deci(cal_dbg_carousel_deg, &deci))
        {
            cal_dbg_last_action = action;
            cal_dbg_last_result = 2U;
            cal_dbg_execute_count++;
            return;
        }
        command = 0x86U;
        p1 = (uint8_t)(deci >> 8);
        p2 = (uint8_t)deci;
    }
    else if (cal_dbg_apply_slide_req != 0U)
    {
        cal_dbg_apply_slide_req = 0U;
        action = CAL_DBG_ACTION_APPLY_SLIDE;
        if (!cal_dbg_float_to_deci(cal_dbg_slide_mm, &deci))
        {
            cal_dbg_last_action = action;
            cal_dbg_last_result = 2U;
            cal_dbg_execute_count++;
            return;
        }
        command = 0x83U;
        p1 = (uint8_t)(deci >> 8);
        p2 = (uint8_t)deci;
    }
    else if (cal_dbg_apply_drop_req != 0U)
    {
        cal_dbg_apply_drop_req = 0U;
        action = CAL_DBG_ACTION_APPLY_DROP;
        if (!cal_dbg_float_to_deci(cal_dbg_drop_mm, &deci))
        {
            cal_dbg_last_action = action;
            cal_dbg_last_result = 2U;
            cal_dbg_execute_count++;
            return;
        }
        command = 0x8BU;
        p1 = (uint8_t)(deci >> 8);
        p2 = (uint8_t)deci;
    }
    else if (cal_dbg_apply_observe_drop_req != 0U)
    {
        cal_dbg_apply_observe_drop_req = 0U;
        action = CAL_DBG_ACTION_APPLY_OBSERVE_DROP;
        command = 0x8EU;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_apply_place_drop_req != 0U)
    {
        cal_dbg_apply_place_drop_req = 0U;
        action = CAL_DBG_ACTION_APPLY_PLACE_DROP;
        command = 0x8FU;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_raise_only_req != 0U)
    {
        cal_dbg_raise_only_req = 0U;
        action = CAL_DBG_ACTION_RAISE_ONLY;
        command = 0x90U;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_slide_retract_only_req != 0U)
    {
        cal_dbg_slide_retract_only_req = 0U;
        action = CAL_DBG_ACTION_SLIDE_RETRACT_ONLY;
        command = 0x91U;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_retract_req != 0U)
    {
        cal_dbg_retract_req = 0U;
        action = CAL_DBG_ACTION_RETRACT;
        command = 0x84U;
    }
    else if (cal_dbg_k230_start_req != 0U)
    {
        cal_dbg_k230_start_req = 0U;
        action = CAL_DBG_ACTION_K230_START;
        command = 0x72U;
        p1 = cal_dbg_ring;
    }
    else if (cal_dbg_k230_stop_req != 0U)
    {
        cal_dbg_k230_stop_req = 0U;
        action = CAL_DBG_ACTION_K230_STOP;
        command = 0x73U;
    }
    else if (cal_dbg_storage_align_req != 0U)
    {
        cal_dbg_storage_align_req = 0U;
        /* 旧名与新通用名是同一个动作，同时置1时也只允许执行一次。 */
        cal_dbg_chassis_align_req = 0U;
        action = CAL_DBG_ACTION_STORAGE_ALIGN;
        command = 0x88U;
    }
    else if (cal_dbg_chassis_align_req != 0U)
    {
        cal_dbg_chassis_align_req = 0U;
        cal_dbg_storage_align_req = 0U;
        action = CAL_DBG_ACTION_CHASSIS_ALIGN;
        command = 0x88U;
    }

    if (action == CAL_DBG_ACTION_NONE)
        return;

    /*
     * process_cmd()在主循环中运行，此处与UART接收中断共享命令槽。
     * 写入的几个字节很少，但仍使用临界区保证不会与新到的VOFA帧
     * 交叉。如果此刻刚好已有一帧VOFA等待处理，Watch请求本周期
     * 记为忙（2），不会覆盖串口命令。
     */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (cmd_ready)
        {
            if (!primask) __enable_irq();
            cal_dbg_last_action = action;
            cal_dbg_last_result = 2U;
            cal_dbg_execute_count++;
            return;
        }

        cmd_v = command;
        p1_v = p1;
        p2_v = p2;
        p3_v = 0U;
        p4_v = 0U;
        cmd_ready = 1U;
        if (!primask) __enable_irq();
    }

    process_cmd();
    cal_dbg_last_action = action;
    cal_dbg_last_result = last_result;
    cal_dbg_execute_count++;
}

static void send_telemetry(uint32_t now_ms)
{
    Hwt101 *imu;

    if ((uint32_t)(now_ms - last_tx_ms) < VOFA_DEBUG_PERIOD_MS) return;
    if (huart5.gState != HAL_UART_STATE_READY) return;

    imu = AppUart_GetImu();

    if (k230_telemetry_enable == 1U)
    {
        txp.p.ch[0] = (float)g_k230_status.color_id;
        txp.p.ch[1] = (float)k230_direction_code(g_k230_status.direction);
        txp.p.ch[2] = (float)g_k230_status.offset_px;
        txp.p.ch[3] = K230_IsOnline(now_ms) ? 1.0f : 0.0f;
        txp.p.ch[4] = (float)g_k230_status.last_event;
        txp.p.ch[5] = (float)k230_rx_byte_count;
        txp.p.ch[6] = (float)k230_valid_line_count;
        txp.p.ch[7] = (float)k230_invalid_line_count;
    }
    else if (k230_telemetry_enable == 2U)
    {
        /*
         * 粗加工标定页：前 8 路集中显示“车到哪、朝向多少、滑台到哪”。
         * ch4=-1 表示滑台软件位置已失效，必须人工回机械原点后发 0x81。
         */
        txp.p.ch[0] = (float)RoughCalibration_GetState();
        txp.p.ch[1] = (float)RoughCalibration_GetError();
        txp.p.ch[2] = (float)RoughCalibration_GetCurrentNode();
        txp.p.ch[3] = rough_cal_world_yaw_deg;
        txp.p.ch[4] = ManipulatorAction_SlideJogPositionKnown() ?
                      ManipulatorAction_SlideJogGetPositionMm() : -1.0f;
        txp.p.ch[5] = RoughCalibration_GetSlideTargetMm();
        /* 任意角标定后操作者必须能直接读回目标角；当前圆环号仍可在
         * rough_cal_selected_ring / Keil Watch 中查看。 */
        txp.p.ch[6] = RoughCalibration_GetCarouselTargetDeg();
        /*
         * 标定齿条长度只关心 K230 的 w/s 径向轴：w 记正、s 记负。
         * 不再使用“最后一行 offset”，否则 K230 两轴报文的 a/d 行可能覆盖
         * w/s，操作者会误把云台切向误差当成齿条距离误差。
         */
        txp.p.ch[7] = (g_k230_status.axis_x_dir == 'w') ?
                      (float)g_k230_status.axis_x_px :
                      ((g_k230_status.axis_x_dir == 's') ?
                       -(float)g_k230_status.axis_x_px : 0.0f);
    }
    else if (k230_telemetry_enable == 3U)
    {
        /*
         * 暂存区标定页与粗加工页保持相同通道含义，方便直接复用VOFA曲线：
         * ch0状态，ch1错误，ch2节点，ch3世界航向，ch4实际齿条位置，
         * ch5目标齿条，ch6目标云台角，ch7 K230径向偏差(w正/s负)。
         */
        txp.p.ch[0] = (float)StorageCalibration_GetState();
        txp.p.ch[1] = (float)StorageCalibration_GetError();
        txp.p.ch[2] = (float)StorageCalibration_GetCurrentNode();
        txp.p.ch[3] = storage_cal_world_yaw_deg;
        txp.p.ch[4] = ManipulatorAction_SlideJogPositionKnown() ?
                      ManipulatorAction_SlideJogGetPositionMm() : -1.0f;
        txp.p.ch[5] = StorageCalibration_GetSlideTargetMm();
        txp.p.ch[6] = StorageCalibration_GetCarouselTargetDeg();
        txp.p.ch[7] = (g_k230_status.axis_x_dir == 'w') ?
                      (float)g_k230_status.axis_x_px :
                      ((g_k230_status.axis_x_dir == 's') ?
                       -(float)g_k230_status.axis_x_px : 0.0f);
    }
    else
    {
        txp.p.ch[0] = imu->roll_deg;
        txp.p.ch[1] = imu->pitch_deg;
        txp.p.ch[2] = hwt101_yaw_relative_deg(imu);
        txp.p.ch[3] = hwt101_is_fresh(imu, now_ms, 500U) ? 1.0f : 0.0f;
        txp.p.ch[4] = (float)can_error_step;
        txp.p.ch[5] = (float)can_error_count;
        txp.p.ch[6] = (float)can_busoff_cnt;
        txp.p.ch[7] = can_rx_flag ? 1.0f : 0.0f;
    }
    txp.p.ch[8] = (float)last_cmd;
    txp.p.ch[9] = (float)last_result;

    if (HAL_UART_Transmit_IT(&huart5, txp.raw, (uint16_t)sizeof(txp.raw)) == HAL_OK)
        last_tx_ms = now_ms;
}

void VofaDebug_Update(uint32_t now_ms)
{
    /*
     * Watch读回始终使用正的“从最高原点累计下降量”。底层为兼容电机方向
     * 保存的是负数，因此只在调试界面边界转换一次，避免操作者混淆符号。
     */
    cal_dbg_current_drop_mm =
        -ManipulatorAction_SlideJogGetVisionDropMm();

    process_cmd();

    /*
     * UART VOFA帧处理完后再查看 Keil Watch 按钮。
     * 这样旧VOFA工具和新Watch面板可以并存，但不会相互覆盖命令。
     */
    cal_dbg_process_watch_request();

    /* 标定状态机先生成本周期的新指令，下面统一调用一次chassis_update()。 */
    RoughCalibration_Update(now_ms);
    StorageCalibration_Update(now_ms);

    /* 动作状态机本周期可能刚提交新高度，再刷新一次供断点/Watch读取。 */
    cal_dbg_current_drop_mm =
        -ManipulatorAction_SlideJogGetVisionDropMm();

    /*
     * Required by the non-blocking chassis state machine.
     * Keep calling this every main-loop iteration.
     */
    chassis_update(&s_debug_chassis, now_ms);

    if (formal_motion_active) {
        if (chassis_motion_failed(&s_debug_chassis)) {
            last_result = 5U;
            formal_motion_active = 0U;
        }
        else if (chassis_motion_done(&s_debug_chassis)) {
            last_result = 4U;
            formal_motion_active = 0U;
        }
    }

    if (!RoughCalibration_IsActive() && !StorageCalibration_IsActive())
        debug_navigation_update(now_ms);

    send_telemetry(now_ms);
}
