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
     * Use the real calibrated chassis configuration.
     * This is the same motion/kinematics layer used by navigation.
     */
    chassis_config_init_mecanum(&cfg);
    chassis_init(&s_debug_chassis, &cfg);
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
    if (cmd >= 0x80U && cmd <= 0x88U) return 1U;
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
     *   0x72/0x73  K230观察命令（暂存流程到N4前均禁止，避免干扰自动对准）；
     *   0x74       切换遥测页；
     *   0x81~0x86  标定状态机自己的控制命令。
     *   0x88       仅暂存区N4手动调参态可用的二号环底盘位置锁定。
     * 这样可避免旧的直接电机命令和状态机同时向同一执行器下发动作。
     */
    if ((RoughCalibration_IsActive() || StorageCalibration_IsActive()) &&
        c != 0x01U &&
        c != 0x74U &&
        !((c == 0x72U || c == 0x73U) &&
          (RoughCalibration_IsActive() ||
           StorageCalibration_AllowsManualVision())) &&
        !(c == 0x88U && StorageCalibration_AllowsManualVision()) &&
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

        case 0x80U: /* 粗加工标定：P1=节点1..9，A1/A2=对应启停区，自动到 N2。 */
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
            }
            else if (!RoughCalibration_IsActive() ||
                     !RoughCalibration_SelectRing(p1, HAL_GetTick()))
            {
                last_result = 18U;
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

        case 0x87U: /* 暂存区标定全流程；P1=A5确认齿条已在机械原点。 */
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

        case 0x88U: /* N4二号环：保持当前机械位姿，K230平移底盘并复核180°。 */
            if (!StorageCalibration_IsActive() ||
                !StorageCalibration_StartStorageVisionAlign(HAL_GetTick()))
            {
                last_result = 18U;
            }
            break;

    default:
        last_result = 3U;
        break;
    }
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
    process_cmd();

    /* 标定状态机先生成本周期的新指令，下面统一调用一次chassis_update()。 */
    RoughCalibration_Update(now_ms);
    StorageCalibration_Update(now_ms);

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
