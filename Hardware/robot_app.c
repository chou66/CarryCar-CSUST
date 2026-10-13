#include "robot_app.h"
#include "robot_config.h"
#include "chassis_config.h"
#include "app_uart.h"
#include "hwt101_uart.h"
#include "hmi_display.h"
#include "hmi_protocol.h"
#include "k230.h"
#include "k230_align.h"
#include "qr_scanner.h"
#include "servo.h"
#include "manipulator.h"
#include "manipulator_action.h"
#include "material_load_task.h"
#include "zone_task.h"
#include "navigation.h"
#include "map_graph.h"
#include "chassis_motion.h"
#include <math.h>

static Chassis s_chassis;
static HMI_Navigation_t s_nav;
static EdgeMotion_t s_edge;

/*
 * Manual VOFA takeover:
 *   0 = RobotApp owns the chassis;
 *   1 = VOFA owns the chassis until MCU reset.
 */
volatile uint8_t robot_app_vofa_override = 0U;
volatile uint8_t robot_app_vofa_override_ready = 0U;

#if MISSION_ENABLE
/*
 * Formal full mission.
 *
 * HMI start bay -> N3/N9 road corner
 *   -> N6 QR
 *   -> N8 RAW-1
 *   -> N2 ROUGH-1
 *   -> N4 STORAGE-1
 *   -> N8 RAW-2
 *   -> N2 ROUGH-2
 *   -> N4 STORAGE-2
 *   -> HMI return road corner -> return bay
 *
 * The real task actions are not connected yet. At N6/N8/N2/N4 the robot:
 *   stop -> align to the node's fixed task yaw -> stay stopped 2 s -> continue.
 * The 2 s hold is an explicit placeholder for the future task state machines.
 */
typedef enum
{
    MISSION_WAIT_MAP = 0,
    MISSION_START_DELAY,
    MISSION_EXIT_PARK,
    MISSION_NAVIGATING,
    MISSION_ENTER_PARK,
    MISSION_ALIGNING,
    MISSION_ALIGN_SETTLE,
    MISSION_TASK_HOLD,
    MISSION_TASK_RUN,
    /* 追加在末尾，保持前面所有枚举值不变。 */
    MISSION_VISION_ALIGN,
    MISSION_DONE,
    MISSION_ERROR
} MissionState_t;

volatile uint8_t robot_mission_state = MISSION_WAIT_MAP;
volatile uint8_t robot_mission_stage = 0U;
volatile uint8_t robot_mission_target = 0U;
volatile uint8_t robot_mission_error = 0U;

/* World-yaw / task-yaw diagnostics for Keil Watch. */
volatile uint8_t robot_world_zero_ready = 0U;
volatile float robot_world_yaw_deg = 0.0f;
volatile float robot_mission_target_yaw_deg = 0.0f;
volatile uint8_t robot_mission_align_attempt = 0U;

static uint32_t s_mission_deadline_ms = 0U;

/* 区域机械动作进度：Keil Watch / VOFA 可观察，不占用 HMI 串口。 */
volatile uint8_t robot_mission_task_step = 0U;
volatile uint8_t robot_mission_task_steps = 0U;

/* 视觉对位进度：同上。 */
volatile uint8_t robot_mission_vision_iteration = 0U;

#define MISSION_STAGE_COUNT                 8U
#define MISSION_ERROR_CONFIG                1U
#define MISSION_ERROR_NO_PATH               2U
#define MISSION_ERROR_NAVIGATION            3U
#define MISSION_ERROR_EDGE_MOTION           4U
#define MISSION_ERROR_RETURN_NODE           5U
#define MISSION_ERROR_VOFA_OVERRIDE         6U
#define MISSION_ERROR_IMU                    7U
#define MISSION_ERROR_ALIGN                  8U
#define MISSION_ERROR_PARK                   9U
#define MISSION_ERROR_TASK                  10U
#define MISSION_ERROR_VISION                11U
#define MISSION_DEG_TO_RAD_F     0.01745329251994329577f

static uint8_t mission_node_valid(uint8_t node)
{
    return (node >= 1U && node <= NODE_NUM) ? 1U : 0U;
}

static float mission_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float mission_wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

static uint8_t mission_config_valid(void)
{
    return (mission_node_valid(MISSION_QR_NODE) &&
            mission_node_valid(MISSION_RAW_NODE) &&
            mission_node_valid(MISSION_ROUGH_NODE) &&
            mission_node_valid(MISSION_STORAGE_NODE)) ? 1U : 0U;
}

static uint8_t mission_target_for_stage(uint8_t stage)
{
    switch (stage)
    {
        case 0U: return MISSION_QR_NODE;
        case 1U: return MISSION_RAW_NODE;
        case 2U: return MISSION_ROUGH_NODE;
        case 3U: return MISSION_STORAGE_NODE;
        case 4U: return MISSION_RAW_NODE;
        case 5U: return MISSION_ROUGH_NODE;
        case 6U: return MISSION_STORAGE_NODE;
        case 7U: return HMI_Navigation_GetReturnNode(&s_nav);
        default: return 0U;
    }
}

static const char *mission_stage_text(uint8_t stage)
{
    switch (stage)
    {
        case 0U: return "QR";
        case 1U: return "RAW-1";
        case 2U: return "ROUGH-1";
        case 3U: return "STORAGE-1";
        case 4U: return "RAW-2";
        case 5U: return "ROUGH-2";
        case 6U: return "STORAGE-2";
        case 7U: return "RETURN";
        default: return "MISSION";
    }
}

static void mission_fail(uint8_t error_code)
{
    robot_mission_error = error_code;
    robot_mission_state = MISSION_ERROR;
    robot_mission_target = 0U;

    RobotApp_EStop();

    HMI_Race_SetStage("MISSION");
    HMI_Race_SetAction("STOP");
    HMI_Race_SetResult("ERROR");
}

static void mission_done(void)
{
    chassis_motion_stop(&s_chassis);
    robot_mission_state = MISSION_DONE;
    robot_mission_target = 0U;
    robot_mission_target_yaw_deg = 0.0f;

    HMI_Race_SetStage("RETURN");
    HMI_Race_SetAction("DONE");
    HMI_Race_SetResult("SUCCESS");
}

/* 把当前 stage 映射成“区域 + 批次”；stage 0(QR)/7(RETURN) 没有区域动作。 */
static uint8_t mission_zone_for_stage(uint8_t stage,
                                      ZoneId_t *zone,
                                      uint8_t *batch)
{
    switch (stage)
    {
        case 1U: *zone = ZONE_RAW;     *batch = 0U; return 1U;
        case 2U: *zone = ZONE_ROUGH;   *batch = 0U; return 1U;
        case 3U: *zone = ZONE_STORAGE; *batch = 0U; return 1U;
        case 4U: *zone = ZONE_RAW;     *batch = 1U; return 1U;
        case 5U: *zone = ZONE_ROUGH;   *batch = 1U; return 1U;
        case 6U: *zone = ZONE_STORAGE; *batch = 1U; return 1U;
        default: return 0U;
    }
}

/* Drive the short bay <-> road-corner leg outside the nine-node graph.
 * Convert world displacement to the body frame using the current IMU yaw,
 * with the same convention as EdgeMotion's road-edge translation. */
static uint8_t mission_start_park_move(uint8_t corner, uint8_t entering,
                                       uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    float park_x, park_y, dx, dy, yaw, c, sn;

    if (!MapGraph_GetParkingPose(corner, &park_x, &park_y))
    {
        mission_fail(MISSION_ERROR_PARK);
        return 0U;
    }
    if (!hwt101_is_fresh(imu, now_ms, 300U) ||
        !hwt101_world_zero_ready(imu))
        return 0U; /* Stay parked until a fresh orientation is available. */

    dx = g_nodes[corner].x - park_x;
    dy = g_nodes[corner].y - park_y;
    if (entering) { dx = -dx; dy = -dy; }
    yaw = hwt101_world_yaw_deg(imu) * MISSION_DEG_TO_RAD_F;
    c = cosf(yaw);
    sn = sinf(yaw);
    chassis_motion_translate(&s_chassis, dx * c - dy * sn,
                              dx * sn + dy * c,
                              ROUTE_EDGE_SPEED_MPS, now_ms);
    robot_mission_state = entering ? MISSION_ENTER_PARK : MISSION_EXIT_PARK;
    HMI_Race_SetAction(entering ? "PARK" : "EXIT PARK");
    return 1U;
}

static void mission_enter_task_hold(uint32_t now_ms)
{
    ZoneId_t zone;
    uint8_t batch;

    chassis_motion_stop(&s_chassis);

    if (!mission_zone_for_stage(robot_mission_stage, &zone, &batch))
    {
        /*
         * stage 0 (QR) 与 stage 7 (RETURN) 没有区域机械动作。
         * 本版不接扫码，stage 0 继续保留 2 秒占位。
         */
        s_mission_deadline_ms = now_ms + MISSION_TASK_HOLD_MS;
        robot_mission_state = MISSION_TASK_HOLD;

        HMI_Race_SetAction("TASK WAIT");
        HMI_Race_SetResult("PLACEHOLDER");
        return;
    }

    if (!ZoneTask_Start(zone, batch))
    {
        mission_fail(MISSION_ERROR_TASK);
        return;
    }

    robot_mission_task_step = 0U;
    robot_mission_task_steps = ZoneTask_GetStepCount();

    robot_mission_state = MISSION_TASK_RUN;

    HMI_Race_SetAction("TASK");
    HMI_Race_SetResult("RUNNING");
}

/*
 * 视觉对位入口：IMU 朝向已对好、机械动作即将开始之前调用。
 * 只有存在区域动作的 stage（1~6）才做视觉；stage 0(QR) 与 7(RETURN) 直接进任务。
 */
static void mission_begin_vision_align(uint32_t now_ms)
{
    ZoneId_t zone;
    uint8_t batch;

    if (!mission_zone_for_stage(robot_mission_stage, &zone, &batch))
    {
        mission_enter_task_hold(now_ms);
        return;
    }

#if K230_ALIGN_ENABLE
    /*
     * 原料区每一块料都要重新识别一次颜色：对位不在这里预做，而是由
     * ZoneTask 在每步夹取之前各自做一轮（见 zone_task.c 的 align_color），
     * 所以原料区直接进任务。
     */
    if (zone == ZONE_RAW)
    {
        mission_enter_task_hold(now_ms);
        return;
    }

    /* 粗加工区 / 暂存区：整区域动作之前对准一次放置位。 */
    if (!K230Align_Start(K230_ALIGN_MODE_PLACE, K230_ALIGN_PLACE_ID,
                         K230_ALIGN_CAROUSEL_POS_US, now_ms))
    {
        mission_fail(MISSION_ERROR_VISION);
        return;
    }

    robot_mission_vision_iteration = 0U;
    robot_mission_state = MISSION_VISION_ALIGN;

    HMI_Race_SetAction("VISION");
    HMI_Race_SetResult("RUNNING");
#else
    mission_enter_task_hold(now_ms);
#endif
}

static void mission_begin_alignment(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    float current_yaw;
    float error_world_deg;
    float chassis_turn_deg;

    if (!mission_node_valid(robot_mission_target))
    {
        mission_fail(MISSION_ERROR_CONFIG);
        return;
    }

    if (!g_nodes[robot_mission_target].task_yaw_valid)
    {
        mission_begin_vision_align(now_ms);
        return;
    }

    robot_mission_target_yaw_deg = g_nodes[robot_mission_target].task_yaw_deg;

    if (!hwt101_is_fresh(imu, now_ms, 300U) || !hwt101_world_zero_ready(imu))
        return;

    current_yaw = hwt101_world_yaw_deg(imu);
    robot_world_yaw_deg = current_yaw;
    error_world_deg = mission_wrap_deg(robot_mission_target_yaw_deg - current_yaw);

    if (mission_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG)
    {
        mission_begin_vision_align(now_ms);
        return;
    }

    if (robot_mission_align_attempt >= MISSION_ALIGN_MAX_ATTEMPTS)
    {
        mission_fail(MISSION_ERROR_ALIGN);
        return;
    }

    if (chassis_motion_busy(&s_chassis))
        return;

    /*
     * World yaw is clockwise-positive, while chassis_motion_rotate_deg() uses
     * chassis +theta = counter-clockwise. Therefore the commanded chassis turn
     * is the negative of the world-yaw error.
     */
    chassis_turn_deg = -error_world_deg;
    chassis_motion_rotate_deg(&s_chassis,
                              chassis_turn_deg,
                              MISSION_ALIGN_SPEED_MPS,
                              now_ms);
    robot_mission_align_attempt++;
    robot_mission_state = MISSION_ALIGNING;

    HMI_Race_SetAction("ALIGN");
    HMI_Race_SetResult("RUNNING");
}

static uint8_t mission_plan_current_stage(void)
{
    uint8_t target = mission_target_for_stage(robot_mission_stage);

    if (!mission_node_valid(target))
    {
        if (robot_mission_stage == 7U)
            mission_fail(MISSION_ERROR_RETURN_NODE);
        else
            mission_fail(MISSION_ERROR_CONFIG);
        return 0U;
    }

    robot_mission_target = target;
    robot_mission_align_attempt = 0U;
    robot_mission_target_yaw_deg = (robot_mission_stage != 7U &&
                                    g_nodes[target].task_yaw_valid) ?
                                   g_nodes[target].task_yaw_deg : robot_world_yaw_deg;

    HMI_Race_SetStage(mission_stage_text(robot_mission_stage));
    HMI_Race_SetAction("NAV");
    HMI_Race_SetResult("RUNNING");

    /* Every stage replans from the last confirmed node using the current HMI obstacle mask. */
    if (!RobotApp_GotoNode(target))
    {
        mission_fail(MISSION_ERROR_NO_PATH);
        return 0U;
    }

    robot_mission_state = MISSION_NAVIGATING;
    return 1U;
}

/* 一个 stage 的任务结束（区域动作完成，或占位计时到期）后推进到下一段。 */
static void mission_advance_stage(void)
{
    robot_mission_stage++;
    if (robot_mission_stage >= MISSION_STAGE_COUNT)
    {
        mission_done();
        return;
    }

    (void)mission_plan_current_stage();
}

static void mission_update(uint32_t now_ms)
{
    Navigation_t *nav = &s_nav.nav;
    Hwt101 *imu = AppUart_GetImu();
    float park_x, park_y;

    if (hwt101_world_zero_ready(imu))
    {
        robot_world_zero_ready = 1U;
        robot_world_yaw_deg = hwt101_world_yaw_deg(imu);
    }

    if (robot_mission_state == MISSION_ERROR ||
        robot_mission_state == MISSION_DONE)
        return;

    if (EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
    {
        mission_fail(MISSION_ERROR_EDGE_MOTION);
        return;
    }

    if (robot_mission_state == MISSION_WAIT_MAP)
    {
        if (!HMI_Navigation_IsInitialized(&s_nav))
            return;

        if (!mission_config_valid())
        {
            mission_fail(MISSION_ERROR_CONFIG);
            return;
        }

        if (!MapGraph_GetParkingPose(Navigation_GetCurrentNode(nav),
                                     &park_x, &park_y) ||
            !MapGraph_GetParkingPose(HMI_Navigation_GetReturnNode(&s_nav),
                                     &park_x, &park_y))
        {
            mission_fail(MISSION_ERROR_RETURN_NODE);
            return;
        }

        /*
         * Establish the world frame exactly once for the whole mission.
         * The operator places the robot approximately facing map +Y; whatever
         * fresh HWT101 yaw exists at this moment becomes world 0 deg.
         */
        if (!hwt101_is_fresh(imu, now_ms, 300U))
            return;

        hwt101_set_world_zero(imu);
        if (!hwt101_world_zero_ready(imu))
        {
            mission_fail(MISSION_ERROR_IMU);
            return;
        }

        robot_world_zero_ready = 1U;
        robot_world_yaw_deg = 0.0f;
        robot_mission_target_yaw_deg = 0.0f;
        robot_mission_stage = 0U;
        robot_mission_target = 0U;
        robot_mission_error = 0U;
        robot_mission_align_attempt = 0U;
        s_mission_deadline_ms = now_ms + MISSION_START_DELAY_MS;
        robot_mission_state = MISSION_START_DELAY;

        HMI_Race_SetTaskCode("--");
        HMI_Race_SetColor("--");
        HMI_Race_SetStage("MISSION");
        HMI_Race_SetAction("READY");
        HMI_Race_SetResult("WAIT");
        return;
    }

    if (robot_mission_state == MISSION_START_DELAY)
    {
        if ((int32_t)(now_ms - s_mission_deadline_ms) < 0)
            return;

        (void)mission_start_park_move(Navigation_GetCurrentNode(nav), 0U, now_ms);
        return;
    }

    if (robot_mission_state == MISSION_EXIT_PARK ||
        robot_mission_state == MISSION_ENTER_PARK)
    {
        if (chassis_motion_failed(&s_chassis))
        {
            mission_fail(MISSION_ERROR_PARK);
            return;
        }
        if (chassis_motion_done(&s_chassis))
        {
            if (robot_mission_state == MISSION_EXIT_PARK)
                (void)mission_plan_current_stage();
            else
                mission_done();
        }
        return;
    }

    if (robot_mission_state == MISSION_NAVIGATING)
    {
        if (Navigation_HasError(nav))
        {
            mission_fail(MISSION_ERROR_NAVIGATION);
            return;
        }

        if (Navigation_IsDone(nav) && !EdgeMotion_IsActive(&s_edge))
        {
            HMI_Race_SetAction("ARRIVED");
            HMI_Race_SetResult("OK");
            /* The return node is a road corner. Enter its bay and stop at
             * the current travel yaw; task-facing yaw applies only at work areas. */
            if (robot_mission_stage == 7U)
            {
                (void)mission_start_park_move(robot_mission_target, 1U, now_ms);
                return;
            }
            robot_mission_align_attempt = 0U;
            mission_begin_alignment(now_ms);
        }
        return;
    }

    if (robot_mission_state == MISSION_ALIGNING)
    {
        if (chassis_motion_failed(&s_chassis))
        {
            mission_fail(MISSION_ERROR_ALIGN);
            return;
        }

        if (chassis_motion_done(&s_chassis))
        {
            s_mission_deadline_ms = now_ms + MISSION_ALIGN_SETTLE_MS;
            robot_mission_state = MISSION_ALIGN_SETTLE;
        }
        return;
    }

    if (robot_mission_state == MISSION_ALIGN_SETTLE)
    {
        if ((int32_t)(now_ms - s_mission_deadline_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, 300U))
            return;

        /* Re-check the world yaw; if needed, mission_begin_alignment() performs a small retry. */
        mission_begin_alignment(now_ms);
        return;
    }

    if (robot_mission_state == MISSION_VISION_ALIGN)
    {
        if (K230Align_HasError())
        {
            mission_fail(MISSION_ERROR_VISION);
            return;
        }

        robot_mission_vision_iteration = K230Align_GetIteration();

        if (K230Align_IsDone())
        {
            HMI_Race_SetResult("OK");
            mission_enter_task_hold(now_ms);
        }
        return;
    }

    if (robot_mission_state == MISSION_TASK_HOLD)
    {
        if ((int32_t)(now_ms - s_mission_deadline_ms) < 0)
            return;

        mission_advance_stage();
        return;
    }

    if (robot_mission_state == MISSION_TASK_RUN)
    {
        if (ZoneTask_HasError())
        {
            mission_fail(MISSION_ERROR_TASK);
            return;
        }

        robot_mission_task_step = ZoneTask_GetStep();
        robot_mission_task_steps = ZoneTask_GetStepCount();

        if (ZoneTask_IsDone())
        {
            HMI_Race_SetResult("OK");
            mission_advance_stage();
        }
        return;
    }
}
#endif

void RobotApp_Init(void)
{
    MecanumConfig cfg;

    chassis_config_init_mecanum(&cfg);
    chassis_init(&s_chassis, &cfg);

    HMI_Protocol_Init();
    K230_Init();
    AppUart_Init();
    HMI_Display_Init();
    HMI_Navigation_Init(&s_nav);
    EdgeMotion_Init(&s_edge, &s_chassis, &s_nav);

    /* Start PWM at neutral only. No manipulator motion occurs at boot. */
    Servo_Init();
    ManipulatorAction_Init();
    MaterialLoadTask_Init();
    ZoneTask_Init();
    K230Align_Init(&s_chassis);

#if MISSION_ENABLE
    robot_mission_state = MISSION_WAIT_MAP;
    robot_mission_stage = 0U;
    robot_mission_target = 0U;
    robot_mission_error = 0U;
    robot_world_zero_ready = 0U;
    robot_world_yaw_deg = 0.0f;
    robot_mission_target_yaw_deg = 0.0f;
    robot_mission_align_attempt = 0U;
    robot_mission_vision_iteration = 0U;
    s_mission_deadline_ms = 0U;
    hwt101_clear_world_zero(AppUart_GetImu());
#endif

    robot_app_vofa_override = 0U;
    robot_app_vofa_override_ready = 0U;
}

void RobotApp_Update(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    uint32_t events;
    float display_yaw = 0.0f;
    float display_target_yaw = 0.0f;
    float display_speed_mm_s = 0.0f;

    K230_TaskUpdate(now_ms);
    QRScanner_Update(now_ms);

    /* Smooth-servo services must keep running even during VOFA takeover. */
    CarouselServo_Update(now_ms);
    GripperServo_Update(now_ms);

    events = HMI_Navigation_Process(&s_nav);

    if (hwt101_world_zero_ready(imu))
        display_yaw = hwt101_world_yaw_deg(imu);

    /* CMD=0x03 changes only the page. It never starts/stops Navigation. */
    if (HMI_Mode_IsUpdated())
    {
        if (HMI_Mode_Get() == HMI_MODE_DEBUG)
        {
            HMI_Page_Debug();
            HMI_Navigation_UpdateDebugDisplay(&s_nav);
        }
        else
        {
            HMI_Page_Race();
        }

        HMI_Mode_ClearUpdated();
    }

    if (events & (HMI_NAV_EVENT_MAP_INIT |
                  HMI_NAV_EVENT_MAP_UPDATED |
                  HMI_NAV_EVENT_REPLANNED))
    {
        HMI_Navigation_UpdateDebugDisplay(&s_nav);
    }

    if (robot_app_vofa_override)
    {
        if (chassis_busy(&s_chassis))
            chassis_update(&s_chassis, now_ms);
        else
            robot_app_vofa_override_ready = 1U;

        HMI_Display_Periodic(now_ms, display_yaw, 0.0f, 0.0f);
        return;
    }

    /* Advance the existing manipulator state machines only in automatic mode. */
    ManipulatorAction_Update(now_ms);
    MaterialLoadTask_Update(now_ms);
    ZoneTask_Update(now_ms);
    K230Align_Update(now_ms);

    if ((events & (HMI_NAV_EVENT_REPLANNED | HMI_NAV_EVENT_NO_PATH)) &&
        EdgeMotion_IsActive(&s_edge))
    {
        EdgeMotion_EStop(&s_edge);
    }

#if MISSION_ENABLE
    mission_update(now_ms);
#endif

    EdgeMotion_Update(&s_edge, now_ms);
    chassis_update(&s_chassis, now_ms);

#if MISSION_ENABLE
    display_yaw = robot_world_yaw_deg;
    display_target_yaw = EdgeMotion_IsActive(&s_edge) ?
                         s_edge.edge_heading_ref_deg :
                         robot_mission_target_yaw_deg;
    display_speed_mm_s = EdgeMotion_IsActive(&s_edge) ?
                         (ROUTE_EDGE_SPEED_MPS * 1000.0f) : 0.0f;
#endif

    HMI_Display_Periodic(now_ms,
                         display_yaw,
                         display_target_yaw,
                         display_speed_mm_s);
}

uint8_t RobotApp_GotoNode(uint8_t target_node)
{
    if (!HMI_Navigation_PlanTo(&s_nav, target_node))
        return 0U;
    HMI_Navigation_UpdateDebugDisplay(&s_nav);
    return 1U;
}

uint8_t RobotApp_GotoReturnNode(void)
{
    uint8_t n = HMI_Navigation_GetReturnNode(&s_nav);
    return n ? RobotApp_GotoNode(n) : 0U;
}

void RobotApp_EStop(void)
{
    EdgeMotion_EStop(&s_edge);
    MaterialLoadTask_Abort();
    ZoneTask_Abort();
    K230Align_Abort();
}

void RobotApp_EnterVofaOverride(void)
{
    if (robot_app_vofa_override)
        return;

    robot_app_vofa_override_ready = 0U;
    RobotApp_EStop();
    robot_app_vofa_override = 1U;

#if MISSION_ENABLE
    robot_mission_state = MISSION_ERROR;
    robot_mission_target = 0U;
    robot_mission_error = MISSION_ERROR_VOFA_OVERRIDE;
#endif

    HMI_Race_SetStage("VOFA DEBUG");
    HMI_Race_SetAction("MANUAL");
    HMI_Race_SetResult("OVERRIDE");
}

uint8_t RobotApp_IsVofaOverride(void)
{
    return robot_app_vofa_override ? 1U : 0U;
}

uint8_t RobotApp_IsVofaOverrideReady(void)
{
    return (robot_app_vofa_override && robot_app_vofa_override_ready) ? 1U : 0U;
}

Chassis *RobotApp_GetChassis(void) { return &s_chassis; }
HMI_Navigation_t *RobotApp_GetNavigation(void) { return &s_nav; }
