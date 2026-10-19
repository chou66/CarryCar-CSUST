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
#include "raw_material_align.h"
#include "raw_station_task.h"
#include "zone_task.h"
#include "navigation.h"
#include "map_graph.h"
#include "chassis_motion.h"
#include "heading_control.h"
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
 * At each work node the robot first aligns to the node's fixed task yaw, then
 * enters vision/manipulator work. N6 QR remains a 2 s placeholder until the
 * scanner/task-code link is connected; N8/N2/N4 use the real zone state machines.
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
    MISSION_VISION_PRESET,   /* 对位前：云台先转到识别位，滑台再前伸到识别位 */
    MISSION_VISION_RETRACT,  /* 对位后：滑台退回原点 */
    MISSION_DONE,
    MISSION_ERROR,
    /* 原料区内部流程由 raw_station_task 封装。 */
    MISSION_RAW_STATION
} MissionState_t;

volatile uint8_t robot_mission_state = MISSION_WAIT_MAP;
volatile uint8_t robot_mission_stage = 0U;
volatile uint8_t robot_mission_target = 0U;
volatile uint8_t robot_mission_error = 0U;

/* World-yaw / task-yaw diagnostics for Keil Watch. */
volatile uint8_t robot_world_zero_ready = 0U;
volatile float robot_world_yaw_deg = 0.0f;
volatile float robot_mission_target_yaw_deg = 0.0f;
/* 仅用于观察纠偏重新启动次数，不再作为失败门限。 */
volatile uint8_t robot_mission_align_attempt = 0U;
volatile float robot_mission_align_wz_radps = 0.0f;
volatile float robot_mission_yaw_error_deg = 0.0f;
volatile uint8_t robot_mission_align_stable_samples = 0U;

static uint32_t s_mission_deadline_ms = 0U;
static HeadingController s_mission_align_controller;
static uint32_t s_mission_align_next_update_ms = 0U;
static int8_t s_mission_align_start_sign = 0;
static uint8_t s_mission_align_velocity_active = 0U;
static uint8_t s_mission_align_saved_acceleration = 0U;
static uint8_t s_mission_align_acceleration_saved = 0U;
static uint32_t s_mission_align_last_sample_ms = 0U;
static uint8_t s_mission_align_last_sample_valid = 0U;

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
#define MISSION_ERROR_PARK                   9U
#define MISSION_ERROR_TASK                  10U
#define MISSION_ERROR_VISION                11U
#define MISSION_ERROR_RAW_STATION           12U
#define MISSION_DEG_TO_RAD_F     0.01745329251994329577f
#define MISSION_TWO_PI_F         6.28318530717958647692f

/* 原料区退回 N8 后，复用任务航向 PD；对准后应推进 stage 而非再次进任务。 */
static uint8_t s_mission_advance_after_alignment = 0U;

/*
 * 粗加工区/暂存区的“圆心 + 航向”联合锁定标志。
 *
 * K230 平移底盘时，麦轮受力不完全一致可能让车体偏转几度；而随后原地
 * 校正航向时，相机又会绕车体中心走一小段圆弧，使刚对准的圆心再次偏掉。
 * 因此正式任务不能只执行一次“航向 -> K230”，必须循环执行：
 *
 *   K230 对圆心 -> IMU 复核航向 -> 若 PD 真的转过车，再 K230 对圆心
 *
 * 直到最新一次 K230 完成后无需再转车，才能收回齿条并开始区域抓放。
 */
volatile uint8_t robot_mission_pose_lock_active = 0U;
volatile uint8_t robot_mission_pose_lock_yaw_changed = 0U;

static void mission_advance_stage(void);

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

static int8_t mission_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

/*
 * 把“轮速 RPM 上限”换算成 Chassis 速度模式的原地旋转角速度。
 * 这样任务航向与 EdgeMotion 共用同一条 chassis_set_velocity -> CAN 链路。
 */
static float mission_align_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
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
             (MISSION_TWO_PI_F * chassis->config.wheel_radius_m);
    if (factor <= 0.0f)
        return 0.0f;

    return (float)rpm / (k * factor);
}

static void mission_align_save_acceleration(void)
{
    if (s_mission_align_acceleration_saved)
        return;

    s_mission_align_saved_acceleration = s_chassis.acceleration;
    s_mission_align_acceleration_saved = 1U;
}

static void mission_align_restore_acceleration(void)
{
    if (!s_mission_align_acceleration_saved)
        return;

    s_chassis.acceleration = s_mission_align_saved_acceleration;
    s_mission_align_acceleration_saved = 0U;
}

static void mission_align_stop_velocity(void)
{
    if (s_mission_align_velocity_active)
        chassis_motion_stop(&s_chassis);

    s_mission_align_velocity_active = 0U;
    robot_mission_align_wz_radps = 0.0f;
}

/* STOP 真正发完后才开始稳定计时，随后重新读取 IMU 验角。 */
static void mission_enter_align_settle(void)
{
    mission_align_stop_velocity();
    s_mission_deadline_ms = 0U;
    s_mission_align_last_sample_ms = 0U;
    s_mission_align_last_sample_valid = 0U;
    robot_mission_align_stable_samples = 0U;
    robot_mission_state = MISSION_ALIGN_SETTLE;
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
    mission_align_stop_velocity();
    mission_align_restore_acceleration();
    chassis_motion_stop(&s_chassis);
    robot_mission_state = MISSION_DONE;
    robot_mission_target = 0U;
    robot_mission_target_yaw_deg = 0.0f;
    robot_mission_pose_lock_active = 0U;
    robot_mission_pose_lock_yaw_changed = 0U;

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
 * This is a short parking-pose adjustment, not a topology edge, so it keeps
 * the calibrated combined body-frame displacement. All 1..9 road edges use
 * EdgeMotion's turn-first, longitudinal-only (forward/reverse) policy. */
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

/* Chassis-only route test: keep the normal mission scheduler and navigation
 * path, but replace every work-area action with a non-blocking two-second hold.
 * This deliberately lives in RobotApp rather than main.c so it still covers
 * HMI obstacles, Dijkstra, EdgeMotion, parking exit, and parking entry. */
static void mission_enter_chassis_hold(uint32_t now_ms)
{
    chassis_motion_stop(&s_chassis);
    s_mission_deadline_ms = now_ms + MISSION_TASK_HOLD_MS;
    robot_mission_state = MISSION_TASK_HOLD;

    HMI_Race_SetAction("CHASSIS HOLD");
    HMI_Race_SetResult("SKIP TASK");
}

#if K230_ALIGN_ENABLE
/*
 * 启动粗加工/暂存区的圆环对位。
 * 第一轮及粗加工区继续使用旧place,2；只有第二轮暂存区改用stack,2,颜色，
 * 让K230先凭第一层物料确认2号位，再用2号外圆做精确圆心。航向复核后若
 * 需要重拍，也必须再次走本函数，不能退回没有身份闸门的普通place。
 */
static bool mission_start_ring_align(uint32_t now_ms)
{
    ZoneId_t zone;
    uint8_t batch;

    if (!mission_zone_for_stage(robot_mission_stage, &zone, &batch))
    {
        return false;
    }

    if ((zone == ZONE_STORAGE) && (batch == 1U))
    {
        return K230Align_StartStackPreset(
            K230_ALIGN_PLACE_ID,
            K230_STORAGE_STACK_ANCHOR_COLOR,
            now_ms);
    }

    return K230Align_StartPreset(K230_ALIGN_MODE_PLACE,
                                 K230_ALIGN_PLACE_ID,
                                 now_ms);
}
#endif

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
        /*
         * 原料区的底盘停靠、三轮视觉抓取和固定距离撤回全部由
         * RawStationTask 管理。RobotApp 不展开这些子状态，只保留一个
         * MISSION_RAW_STATION 等待点，避免全流程状态机与机械细节耦合。
         */
        if (!RawStationTask_Start(batch, now_ms))
        {
            mission_fail(MISSION_ERROR_RAW_STATION);
            return;
        }

        robot_mission_task_step = 0U;
        robot_mission_task_steps = 3U;
        robot_mission_state = MISSION_RAW_STATION;
        HMI_Race_SetAction("RAW TASK");
        HMI_Race_SetResult("RUNNING");
        return;
    }

    /*
     * 粗加工区 / 暂存区：整区域动作之前对准一次放置位。
     *
     * 粗加工区的基准来自实车标定：标定阶段信任“启停区2直达N2”的物理
     * 停靠位，固定底盘，只调齿条得到 ROUGH_SLIDE_POS2_MM，使K230中心正好
     * 对准2号圆环。正式比赛经过多轮行驶后再复现这组2号相机位姿，反过来
     * 根据K230偏差平移底盘；对准后1号/3号全部走固定机械位姿，不再视觉。
     * 暂存区使用独立的 STORAGE_SLIDE_POS2_MM；它会由专用VOFA流程在经过
     * 粗加工2号环校准底盘后标定，因而不会再借用原料区的旧通用距离。
     */
    if (!((zone == ZONE_ROUGH) ?
          ManipulatorAction_SlideJogToRoughAlignStart() :
          ManipulatorAction_SlideJogToStorageAlignStart()))
    {
        mission_fail(MISSION_ERROR_VISION);
        return;
    }

    robot_mission_state = MISSION_VISION_PRESET;

    HMI_Race_SetAction("VISION");
    HMI_Race_SetResult("PRESET");
#else
    mission_enter_task_hold(now_ms);
#endif
}

/*
 * 航向 PD 对准后的分流：
 *   1. 原料区撤离复核完成：直接进入下一 stage；
 *   2. 圆环联合锁定中：若刚才转过车则保持机械预置并再跑一次 K230，
 *      若完全没有转车则说明“最新圆心 + 当前航向”同时合格，收回齿条；
 *   3. 普通到站首次对航向完成：进入该区域的第一次视觉对位。
 */
static void mission_continue_after_alignment(uint32_t now_ms)
{
    if (s_mission_advance_after_alignment)
    {
        s_mission_advance_after_alignment = 0U;
        mission_advance_stage();
        return;
    }

    if (robot_mission_pose_lock_active)
    {
        if (robot_mission_pose_lock_yaw_changed)
        {
            /*
             * 航向 PD 已改变车体姿态，旧的圆心结果随相机绕车体中心转动而
             * 失效。保持云台205 deg和当前齿条预伸量，立即重新跑本区域视觉；
             * 第二轮暂存区仍是带颜色身份锚点的STACK-2，绝不降级为普通PLACE-2。
             * 不允许中间收齿条或开始机械抓放。
             */
            robot_mission_pose_lock_yaw_changed = 0U;
            if (!mission_start_ring_align(now_ms))
            {
                mission_fail(MISSION_ERROR_VISION);
                return;
            }

            robot_mission_vision_iteration = 0U;
            robot_mission_state = MISSION_VISION_ALIGN;
            HMI_Race_SetAction("POSE LOCK");
            HMI_Race_SetResult("VISION RECHECK");
            return;
        }

        /* 最新一轮K230完成后航向仍在容差内，联合锁定才算真正完成。 */
        robot_mission_pose_lock_active = 0U;
        if (!ManipulatorAction_SlideJogHomeStart())
        {
            mission_fail(MISSION_ERROR_VISION);
            return;
        }

        robot_mission_state = MISSION_VISION_RETRACT;
        HMI_Race_SetAction("POSE LOCK");
        HMI_Race_SetResult("RETRACT");
        return;
    }

    mission_begin_vision_align(now_ms);
}

static void mission_begin_alignment(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    float current_yaw;
    float error_world_deg;
    float world_gyro_dps;
    float wz_cmd;

#if MISSION_CHASSIS_ONLY
    /* Stages 0..6 are work-area stops. Stage 7 enters the return parking bay
     * directly from MISSION_NAVIGATING and never reaches this function. */
    if (robot_mission_stage < 7U)
    {
        mission_align_restore_acceleration();
        mission_enter_chassis_hold(now_ms);
        return;
    }
#endif

    if (!mission_node_valid(robot_mission_target))
    {
        mission_fail(MISSION_ERROR_CONFIG);
        return;
    }

    if (!g_nodes[robot_mission_target].task_yaw_valid)
    {
        mission_align_restore_acceleration();
        mission_continue_after_alignment(now_ms);
        return;
    }

    robot_mission_target_yaw_deg = g_nodes[robot_mission_target].task_yaw_deg;

    if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
        !hwt101_world_zero_ready(imu))
        return;

    current_yaw = hwt101_world_yaw_deg(imu);
    robot_world_yaw_deg = current_yaw;
    error_world_deg = mission_wrap_deg(robot_mission_target_yaw_deg - current_yaw);
    robot_mission_yaw_error_deg = error_world_deg;

    if (mission_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG)
    {
        /* 单个样本不再直接放行；停稳后连续读取3个合格的新IMU样本。 */
        mission_enter_align_settle();
        return;
    }

    if (chassis_motion_busy(&s_chassis))
        return;

    /*
     * 与行驶边末纠偏共用 heading_control：世界角顺时针为正，底盘 wz
     * 逆时针为正，符号转换和陀螺阻尼都由 heading_update_world_cw() 完成。
     */
    heading_set_target(&s_mission_align_controller,
                       robot_mission_target_yaw_deg);
    world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
    wz_cmd = heading_update_world_cw(
        &s_mission_align_controller,
        current_yaw,
        world_gyro_dps,
        (float)MISSION_ALIGN_REFRESH_MS / 1000.0f);

    mission_align_save_acceleration();
    s_chassis.acceleration = MISSION_ALIGN_ACC;
    chassis_set_velocity(&s_chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    /*
     * 只有真正向底盘下发过旋转速度，才判定刚才的K230圆心需要重做。
     * 若航向本来就在容差内，则不会无意义地重复视觉识别。
     */
    if (robot_mission_pose_lock_active)
        robot_mission_pose_lock_yaw_changed = 1U;

    s_mission_align_start_sign = mission_signf(error_world_deg);
    s_mission_align_next_update_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
    s_mission_align_velocity_active = 1U;
    robot_mission_align_wz_radps = wz_cmd;
    if (robot_mission_align_attempt < 255U)
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

    mission_align_stop_velocity();
    mission_align_restore_acceleration();
    robot_mission_target = target;
    robot_mission_align_attempt = 0U;
    s_mission_advance_after_alignment = 0U;
    robot_mission_pose_lock_active = 0U;
    robot_mission_pose_lock_yaw_changed = 0U;
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

    /*
     * 任务点带固定 task_yaw 时，到达后 mission 会立即做任务航向 PD。
     * 告知 EdgeMotion 不要先在边末单独纠到当前边航向，直接提交节点，
     * 由任务航向一次转到目标角度。返回节点(N3/N9)没有任务航向，保持原
     * 边末纠偏，确保入库平移方向准确；仅底盘测试(CHASSIS_ONLY)也不会
     * 执行任务航向，必须继续纠偏。
     */
    EdgeMotion_SetArrivalAlign(&s_edge,
        ((MISSION_CHASSIS_ONLY == 0U) &&
         (robot_mission_stage != 7U) &&
         (g_nodes[target].task_yaw_valid != 0U)) ? 1U : 0U);

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
        float current_yaw;
        float error_world_deg;
        int8_t error_sign;

        /* IMU 暂时断帧只停车等待；不再把任务锁死到 ERROR。 */
        if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
        {
            mission_enter_align_settle();
            return;
        }

        current_yaw = hwt101_world_yaw_deg(imu);
        robot_world_yaw_deg = current_yaw;
        error_world_deg = mission_wrap_deg(robot_mission_target_yaw_deg -
                                           current_yaw);
        robot_mission_yaw_error_deg = error_world_deg;
        error_sign = mission_signf(error_world_deg);

        /* 进入容差或越过目标角都立刻 STOP，防止持续高速造成超调。 */
        if (mission_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG ||
            (error_sign != 0 &&
             s_mission_align_start_sign != 0 &&
             error_sign != s_mission_align_start_sign))
        {
            mission_enter_align_settle();
            return;
        }

        if ((int32_t)(now_ms - s_mission_align_next_update_ms) >= 0)
        {
            float world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
            float wz_cmd = heading_update_world_cw(
                &s_mission_align_controller,
                current_yaw,
                world_gyro_dps,
                (float)MISSION_ALIGN_REFRESH_MS / 1000.0f);

            chassis_set_velocity(&s_chassis, 0.0f, 0.0f, wz_cmd, now_ms);
            robot_mission_align_wz_radps = wz_cmd;
            s_mission_align_next_update_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
        }
        return;
    }

    if (robot_mission_state == MISSION_ALIGN_SETTLE)
    {
        float current_yaw;
        float error_world_deg;

        if (chassis_motion_busy(&s_chassis))
            return;

        if (s_mission_deadline_ms == 0U)
        {
            s_mission_deadline_ms = now_ms + MISSION_ALIGN_SETTLE_MS;
            return;
        }

        if ((int32_t)(now_ms - s_mission_deadline_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        /* 同一个IMU帧不能被高速主循环重复累计成多个“稳定样本”。 */
        if (s_mission_align_last_sample_valid &&
            (imu->last_update_ms == s_mission_align_last_sample_ms))
            return;

        s_mission_align_last_sample_ms = imu->last_update_ms;
        s_mission_align_last_sample_valid = 1U;
        current_yaw = hwt101_world_yaw_deg(imu);
        robot_world_yaw_deg = current_yaw;
        error_world_deg = mission_wrap_deg(robot_mission_target_yaw_deg -
                                           current_yaw);
        robot_mission_yaw_error_deg = error_world_deg;

        if (mission_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG)
        {
            if (robot_mission_align_stable_samples < MISSION_ALIGN_STABLE_SAMPLES)
                robot_mission_align_stable_samples++;

            if (robot_mission_align_stable_samples >= MISSION_ALIGN_STABLE_SAMPLES)
            {
                mission_align_stop_velocity();
                mission_align_restore_acceleration();
                mission_continue_after_alignment(now_ms);
                return;
            }

            /* 与PD刷新周期同速确认，避免短时间内把抖动误判为稳定。 */
            s_mission_deadline_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
            return;
        }

        robot_mission_align_stable_samples = 0U;
        /* 未对准就再次启动 PD；没有次数限制，也没有角度超时失败。 */
        mission_begin_alignment(now_ms);
        return;
    }

    if (robot_mission_state == MISSION_RAW_STATION)
    {
        uint8_t zone_step_count;

        if (RawStationTask_HasError())
        {
            mission_fail(MISSION_ERROR_RAW_STATION);
            return;
        }

        robot_mission_task_step = ZoneTask_GetStep();
        zone_step_count = ZoneTask_GetStepCount();
        if (zone_step_count != 0U)
        {
            robot_mission_task_steps = zone_step_count;
        }
        /* 把子模块进度镜像到原有 mission 调试变量，显示屏/Watch 无需
         * 理解 RawMaterialAlign 的内部状态即可继续显示调整次数。 */
        robot_mission_vision_iteration =
            (RawMaterialAlign_GetIteration() > 255U) ?
            255U : (uint8_t)RawMaterialAlign_GetIteration();

        if (!RawStationTask_IsDone())
            return;

        /* 停靠往返可能带来少量航向漂移：回到 N8 后重新跑现有 PD，
         * 对准成功才允许规划下一条边。 */
        HMI_Race_SetResult("OK");
        s_mission_advance_after_alignment = 1U;
        robot_mission_align_attempt = 0U;
        mission_begin_alignment(now_ms);
        return;
    }

    /* 对位前：等云台转到位、滑台前伸到识别位，两件都就位后才发 place 命令。 */
    if (robot_mission_state == MISSION_VISION_PRESET)
    {
        if (ManipulatorAction_SlideJogHasError())
        {
            mission_fail(MISSION_ERROR_VISION);
            return;
        }

        if (!ManipulatorAction_SlideJogIsDone())
            return;

        if (!mission_start_ring_align(now_ms))
        {
            mission_fail(MISSION_ERROR_VISION);
            return;
        }

        robot_mission_vision_iteration = 0U;
        robot_mission_state = MISSION_VISION_ALIGN;

        HMI_Race_SetResult("RUNNING");
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
            /*
             * K230 只证明此刻圆心像素正确，并不能证明平移过程中航向没有
             * 漂移。先保留云台/齿条的识别位姿，进入现有 STOP->稳定->PD
             * 验角链；如果PD真的转过，mission_continue_after_alignment()
             * 会自动再跑K230。两项同时合格以后才允许收齿条。
             */
            robot_mission_pose_lock_active = 1U;
            robot_mission_pose_lock_yaw_changed = 0U;
            robot_mission_align_attempt = 0U;
            HMI_Race_SetAction("POSE LOCK");
            HMI_Race_SetResult("YAW CHECK");
            mission_enter_align_settle();
        }
        return;
    }

    /* 对位后：等滑台退回原点，然后再开始区域机械动作。 */
    if (robot_mission_state == MISSION_VISION_RETRACT)
    {
        if (ManipulatorAction_SlideJogHasError())
        {
            mission_fail(MISSION_ERROR_VISION);
            return;
        }

        if (ManipulatorAction_SlideJogIsDone())
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
    chassis_bind_imu_filter(&s_chassis, AppUart_GetImu());
    HMI_Display_Init();
    HMI_Navigation_Init(&s_nav);
    EdgeMotion_Init(&s_edge, &s_chassis, &s_nav);

    /* Start PWM at neutral only. No manipulator motion occurs at boot. */
    Servo_Init();
    ManipulatorAction_Init();
    MaterialLoadTask_Init();
    RawMaterialAlign_Init();
    ZoneTask_Init();
    RawStationTask_Init(&s_chassis);
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
    robot_mission_align_wz_radps = 0.0f;
    robot_mission_yaw_error_deg = 0.0f;
    robot_mission_align_stable_samples = 0U;
    robot_mission_vision_iteration = 0U;
    s_mission_deadline_ms = 0U;
    s_mission_align_next_update_ms = 0U;
    s_mission_align_start_sign = 0;
    s_mission_align_velocity_active = 0U;
    s_mission_align_saved_acceleration = 0U;
    s_mission_align_acceleration_saved = 0U;
    s_mission_align_last_sample_ms = 0U;
    s_mission_align_last_sample_valid = 0U;
    s_mission_advance_after_alignment = 0U;
    robot_mission_pose_lock_active = 0U;
    robot_mission_pose_lock_yaw_changed = 0U;
    heading_init(&s_mission_align_controller,
                 MISSION_ALIGN_KP,
                 MISSION_ALIGN_KD,
                 mission_align_wz_for_rpm(&s_chassis, MISSION_ALIGN_MAX_RPM),
                 MISSION_ALIGN_TOLERANCE_DEG);
    heading_set_min_output(
        &s_mission_align_controller,
        mission_align_wz_for_rpm(&s_chassis, MISSION_ALIGN_MIN_RPM));
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
    RawMaterialAlign_Update(now_ms);
    ZoneTask_Update(now_ms);
    RawStationTask_Update(now_ms);
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

    /*
     * A new target clears only the previous route's display diagnostics.
     * The world-zero yaw is preserved; every physical edge independently
     * derives its travel heading from the current from/to node coordinates.
     */
    EdgeMotion_BeginRoute(&s_edge);

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
#if MISSION_ENABLE
    mission_align_stop_velocity();
    mission_align_restore_acceleration();
    robot_mission_pose_lock_active = 0U;
    robot_mission_pose_lock_yaw_changed = 0U;
#endif
    MaterialLoadTask_Abort();
    RawStationTask_Abort();
    ZoneTask_Abort();
    RawMaterialAlign_Abort();
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

uint8_t RobotApp_IsMissionIdle(void)
{
    return (robot_mission_state == MISSION_WAIT_MAP ||
            robot_mission_state == MISSION_DONE ||
            robot_mission_state == MISSION_ERROR) ? 1U : 0U;
}

Chassis *RobotApp_GetChassis(void) { return &s_chassis; }
HMI_Navigation_t *RobotApp_GetNavigation(void) { return &s_nav; }
