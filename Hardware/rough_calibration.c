#include "rough_calibration.h"
#include "app_uart.h"
#include "edge_motion.h"
#include "heading_control.h"
#include "hmi_navigation.h"
#include "hwt101_uart.h"
#include "manipulator_action.h"
#include "map_graph.h"
#include "navigation.h"
#include "robot_config.h"
#include "servo.h"
#include "chassis_motion.h"
#include <math.h>

/* 世界角度转弧度只在“RPM 上限 -> wz 上限”换算中使用。 */
#define ROUGH_CAL_TWO_PI_F 6.28318530717958647692f
#define ROUGH_CAL_DEG_TO_RAD_F 0.01745329251994329577f

static Chassis *s_chassis;
static HMI_Navigation_t s_nav;
static EdgeMotion_t s_edge;
static HeadingController s_heading;

static uint32_t s_deadline_ms;
static uint32_t s_next_heading_update_ms;
static int8_t s_heading_start_sign;
static uint8_t s_heading_velocity_active;
static uint8_t s_saved_acceleration;
static uint8_t s_acceleration_saved;
static uint8_t s_start_corner;
/* 云台动作完成后回到哪个可操作状态：缩回时回 ARM_READY，伸出时回 SLIDE_READY。 */
static RoughCalibrationState_t s_arm_done_state;

volatile uint8_t rough_cal_state = (uint8_t)ROUGH_CAL_IDLE;
volatile uint8_t rough_cal_error = ROUGH_CAL_ERROR_NONE;
volatile uint8_t rough_cal_current_node = 0U;
volatile uint8_t rough_cal_selected_ring = 0U;
volatile uint8_t rough_cal_home_confirmed = 0U;
volatile float rough_cal_target_slide_mm = 0.0f;
volatile float rough_cal_target_carousel_deg = 0.0f;
volatile float rough_cal_world_yaw_deg = 0.0f;

static float rough_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float rough_wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

static int8_t rough_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

/* 与正式任务航向层相同：把轮速上限换算成 chassis_set_velocity 的 wz。 */
static float rough_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
{
    float k;
    float factor;

    if (!chassis ||
        chassis->config.wheel_radius_m <= 0.0f ||
        chassis->config.gear_ratio <= 0.0f)
    {
        return 0.0f;
    }

    k = chassis->config.half_length_m + chassis->config.half_width_m;
    if (k <= 0.0f) return 0.0f;

    factor = 60.0f * chassis->config.gear_ratio /
             (ROUGH_CAL_TWO_PI_F * chassis->config.wheel_radius_m);
    if (factor <= 0.0f) return 0.0f;

    return (float)rpm / (k * factor);
}

static void rough_set_state(RoughCalibrationState_t state)
{
    rough_cal_state = (uint8_t)state;
}

static void rough_save_acceleration(void)
{
    if (!s_chassis || s_acceleration_saved) return;
    s_saved_acceleration = s_chassis->acceleration;
    s_acceleration_saved = 1U;
}

static void rough_restore_acceleration(void)
{
    if (!s_chassis || !s_acceleration_saved) return;
    s_chassis->acceleration = s_saved_acceleration;
    s_acceleration_saved = 0U;
}

static void rough_stop_heading(void)
{
    if (s_chassis && s_heading_velocity_active)
        chassis_motion_stop(s_chassis);

    s_heading_velocity_active = 0U;
}

static void rough_fail(uint8_t error)
{
    rough_stop_heading();
    if (s_chassis)
        chassis_motion_stop(s_chassis);
    rough_restore_acceleration();

    /* 滑台运动中失败后真实位置不再可信，必须急停并要求人工重新回零。 */
    if (ManipulatorAction_SlideJogIsBusy())
    {
        ManipulatorAction_Abort();
        rough_cal_home_confirmed = 0U;
    }

    rough_cal_error = error;
    rough_set_state(ROUGH_CAL_ERROR);
}

/* STOP 发完并稳定后重新验角；不设尝试次数上限，未对准就继续 PD。 */
static void rough_enter_align_settle(void)
{
    rough_stop_heading();
    s_deadline_ms = 0U;
    rough_set_state(ROUGH_CAL_ALIGN_SETTLE);
}

static void rough_begin_alignment(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    float current_yaw;
    float error_world_deg;
    float world_gyro_dps;
    float wz_cmd;

    if (!s_chassis ||
        !hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
        !hwt101_world_zero_ready(imu))
    {
        return;
    }

    current_yaw = hwt101_world_yaw_deg(imu);
    rough_cal_world_yaw_deg = current_yaw;
    error_world_deg = rough_wrap_deg(
        g_nodes[MISSION_ROUGH_NODE].task_yaw_deg - current_yaw);

    if (rough_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG)
    {
        rough_stop_heading();
        rough_restore_acceleration();
        rough_set_state(ROUGH_CAL_WAIT_HOME_CONFIRM);
        return;
    }

    if (chassis_motion_busy(s_chassis)) return;

    heading_set_target(&s_heading,
                       g_nodes[MISSION_ROUGH_NODE].task_yaw_deg);
    world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
    wz_cmd = heading_update_world_cw(
        &s_heading,
        current_yaw,
        world_gyro_dps,
        (float)MISSION_ALIGN_REFRESH_MS / 1000.0f);

    rough_save_acceleration();
    s_chassis->acceleration = MISSION_ALIGN_ACC;
    chassis_set_velocity(s_chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    s_heading_start_sign = rough_signf(error_world_deg);
    s_next_heading_update_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
    s_heading_velocity_active = 1U;
    rough_set_state(ROUGH_CAL_ALIGNING);
}

static float rough_ring_angle(uint8_t ring)
{
    /* 粗加工区拥有独立角度，调它不会连带改变原料区和暂存区。 */
    if (ring == 1U) return ROUGH_CAROUSEL_POS1_DEG;
    if (ring == 2U) return ROUGH_CAROUSEL_POS2_DEG;
    if (ring == 3U) return ROUGH_CAROUSEL_POS3_DEG;
    return -1.0f;
}

/*
 * 启停区不属于 1~9 路网。A1/A2 先映射到相邻路口，退出蓝色区域以后才把
 * Navigation 的 current_node 认定为 N9/N3。
 */
static uint8_t rough_start_corner(uint8_t start_code)
{
    if (start_code == ROUGH_CAL_START_BAY1) return 9U;
    if (start_code == ROUGH_CAL_START_BAY2) return 3U;
    return start_code;
}

static uint8_t rough_plan_to_n2(void)
{
    if (!HMI_Navigation_PlanTo(&s_nav, MISSION_ROUGH_NODE))
    {
        rough_fail(ROUGH_CAL_ERROR_NAVIGATION);
        return 0U;
    }

    EdgeMotion_BeginRoute(&s_edge);
    /*
     * 到 N2 后标定状态机会立即按 task_yaw(+90) 做与正式任务相同的
     * 航向 PD，因此边末不必先单独纠到当前边航向。
     */
    EdgeMotion_SetArrivalAlign(&s_edge, 1U);
    HMI_Navigation_UpdateDebugDisplay(&s_nav);
    rough_set_state(ROUGH_CAL_NAVIGATING);
    return 1U;
}

void RoughCalibration_Init(Chassis *chassis)
{
    s_chassis = chassis;
    HMI_Navigation_Init(&s_nav);
    EdgeMotion_Init(&s_edge, chassis, &s_nav);

    heading_init(&s_heading,
                 MISSION_ALIGN_KP,
                 MISSION_ALIGN_KD,
                 rough_wz_for_rpm(chassis, MISSION_ALIGN_MAX_RPM),
                 MISSION_ALIGN_TOLERANCE_DEG);
    heading_set_min_output(
        &s_heading,
        rough_wz_for_rpm(chassis, MISSION_ALIGN_MIN_RPM));

    s_deadline_ms = 0U;
    s_next_heading_update_ms = 0U;
    s_heading_start_sign = 0;
    s_heading_velocity_active = 0U;
    s_saved_acceleration = 0U;
    s_acceleration_saved = 0U;
    s_start_corner = 0U;
    s_arm_done_state = ROUGH_CAL_ARM_READY;

    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_cal_current_node = 0U;
    rough_cal_selected_ring = 0U;
    rough_cal_home_confirmed = 0U;
    rough_cal_target_slide_mm = 0.0f;
    rough_cal_target_carousel_deg = 0.0f;
    rough_cal_world_yaw_deg = 0.0f;
    s_start_corner = 0U;
    rough_set_state(ROUGH_CAL_IDLE);
}

uint8_t RoughCalibration_Start(uint8_t start_node, uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    uint8_t start_from_bay;
    uint8_t corner;
    float park_x;
    float park_y;
    float world_dx;
    float world_dy;
    float yaw_rad;
    float c;
    float sn;

    start_from_bay = (start_node == ROUGH_CAL_START_BAY1 ||
                      start_node == ROUGH_CAL_START_BAY2) ? 1U : 0U;
    corner = rough_start_corner(start_node);

    if (!s_chassis || corner < 1U || corner > NODE_NUM)
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARAM;
        return 0U;
    }

    if (RoughCalibration_IsActive() || chassis_motion_busy(s_chassis))
    {
        rough_cal_error = ROUGH_CAL_ERROR_BUSY;
        return 0U;
    }

    /* 不在新鲜 IMU 数据上建世界零点；否则整段路线和 N2 航向都会一起偏。 */
    if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS))
    {
        rough_cal_error = ROUGH_CAL_ERROR_IMU;
        return 0U;
    }

    hwt101_set_world_zero(imu);
    if (!hwt101_world_zero_ready(imu))
    {
        rough_cal_error = ROUGH_CAL_ERROR_IMU;
        return 0U;
    }

    HMI_Navigation_Init(&s_nav);
    if (!Navigation_Init(&s_nav.nav, corner, 0UL))
    {
        rough_cal_error = ROUGH_CAL_ERROR_NAVIGATION;
        return 0U;
    }

    /* HMI_Navigation 只是 EdgeMotion 所需的桥接容器；标定路线不等待屏幕地图。 */
    s_nav.initialized = 1U;
    s_nav.map.start_node = corner;
    s_nav.map.obstacle_mask = 0UL;

    EdgeMotion_Init(&s_edge, s_chassis, &s_nav);

    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_cal_current_node = start_from_bay ? 0U : corner;
    rough_cal_selected_ring = 0U;
    rough_cal_home_confirmed = 0U;
    rough_cal_target_slide_mm = 0.0f;
    rough_cal_target_carousel_deg = 0.0f;
    rough_cal_world_yaw_deg = 0.0f;
    s_deadline_ms = 0U;
    s_next_heading_update_ms = 0U;
    s_heading_start_sign = 0;
    s_heading_velocity_active = 0U;
    s_acceleration_saved = 0U;
    s_start_corner = corner;

    if (!start_from_bay)
        return rough_plan_to_n2();

    /*
     * 完整复用正式 Mission 的启停区坐标定义和世界->车体变换：
     * 启停区2且车头向上时，实测位移是 dx=-150 mm、dy=+200 mm，也就是
     * 左前斜移到 N3。这一小段是蓝色区到路网入口的停靠位移，不提交拓扑边。
     */
    if (!MapGraph_GetParkingPose(corner, &park_x, &park_y))
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARK;
        return 0U;
    }

    world_dx = g_nodes[corner].x - park_x;
    world_dy = g_nodes[corner].y - park_y;
    yaw_rad = hwt101_world_yaw_deg(imu) * ROUGH_CAL_DEG_TO_RAD_F;
    c = cosf(yaw_rad);
    sn = sinf(yaw_rad);

    chassis_motion_translate(s_chassis,
                             world_dx * c - world_dy * sn,
                             world_dx * sn + world_dy * c,
                             ROUTE_EDGE_SPEED_MPS,
                             now_ms);
    rough_set_state(ROUGH_CAL_EXIT_PARK);
    return 1U;
}

uint8_t RoughCalibration_ConfirmSlideHome(uint8_t confirm_key)
{
    if (confirm_key != 0xA5U)
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARAM;
        return 0U;
    }

    if (RoughCalibration_GetState() != ROUGH_CAL_WAIT_HOME_CONFIRM &&
        RoughCalibration_GetState() != ROUGH_CAL_HOME_READY)
    {
        rough_cal_error = ROUGH_CAL_ERROR_BUSY;
        return 0U;
    }

    if (!ManipulatorAction_SlideJogRebaseHome())
    {
        rough_cal_error = ROUGH_CAL_ERROR_HOME;
        return 0U;
    }

    rough_cal_home_confirmed = 1U;
    rough_cal_target_slide_mm = 0.0f;
    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_set_state(ROUGH_CAL_HOME_READY);
    return 1U;
}

uint8_t RoughCalibration_SelectRing(uint8_t ring, uint32_t now_ms)
{
    float angle_deg = rough_ring_angle(ring);

    if (angle_deg < 0.0f)
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARAM;
        return 0U;
    }

    /* 云台只允许在滑台确知为 0 mm 时换位，防止伸出状态扫到场地结构。 */
    if (RoughCalibration_GetState() != ROUGH_CAL_HOME_READY ||
        !rough_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        rough_absf(ManipulatorAction_SlideJogGetPositionMm()) > 0.05f)
    {
        rough_cal_error = ROUGH_CAL_ERROR_HOME;
        return 0U;
    }

    CarouselServo_SetAngle(angle_deg);
    GripperServo_Open_Large();
    rough_cal_selected_ring = ring;
    rough_cal_target_carousel_deg = angle_deg;
    s_arm_done_state = ROUGH_CAL_ARM_READY;
    s_deadline_ms = now_ms + K230_ALIGN_ROTATE_TIMEOUT_MS;
    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_set_state(ROUGH_CAL_ARM_MOVING);
    return 1U;
}

uint8_t RoughCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                             uint32_t now_ms)
{
    float target_deg = (float)target_deci_deg * 0.1f;
    RoughCalibrationState_t state = RoughCalibration_GetState();

    if (target_deg < ROUGH_CAROUSEL_CAL_MIN_DEG ||
        target_deg > ROUGH_CAROUSEL_CAL_MAX_DEG)
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARAM;
        return 0U;
    }

    /*
     * 必须先用 0x82 选择正在标定的圆环：这样夹爪已经张开、操作者也明确
     * 知道自己在改哪一个固定工位。允许在滑台伸出后继续微调角度，但禁止
     * 滑台运动过程中转动，避免两个执行器同时扫过未知轨迹。
     */
    if ((state != ROUGH_CAL_ARM_READY && state != ROUGH_CAL_SLIDE_READY) ||
        rough_cal_selected_ring < 1U || rough_cal_selected_ring > 3U ||
        !rough_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        rough_cal_error = ROUGH_CAL_ERROR_BUSY;
        return 0U;
    }

    CarouselServo_SetAngle(target_deg);
    rough_cal_target_carousel_deg = target_deg;
    s_arm_done_state = (state == ROUGH_CAL_SLIDE_READY) ?
                       ROUGH_CAL_SLIDE_READY : ROUGH_CAL_ARM_READY;
    s_deadline_ms = now_ms + K230_ALIGN_ROTATE_TIMEOUT_MS;
    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_set_state(ROUGH_CAL_ARM_MOVING);
    return 1U;
}

uint8_t RoughCalibration_MoveSlideDeciMm(uint16_t target_deci_mm)
{
    float target_mm = (float)target_deci_mm * 0.1f;
    RoughCalibrationState_t state = RoughCalibration_GetState();

    if (target_mm < 0.0f || target_mm > MANIP_SLIDE_SOFT_MAX_MM)
    {
        rough_cal_error = ROUGH_CAL_ERROR_PARAM;
        return 0U;
    }

    if ((state != ROUGH_CAL_ARM_READY && state != ROUGH_CAL_SLIDE_READY) ||
        !rough_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        rough_cal_error = ROUGH_CAL_ERROR_BUSY;
        return 0U;
    }

    if (!ManipulatorAction_SlideJogToPositionStart(target_mm))
    {
        rough_cal_error = ROUGH_CAL_ERROR_MANIPULATOR;
        return 0U;
    }

    rough_cal_target_slide_mm = target_mm;
    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_set_state(ROUGH_CAL_SLIDE_MOVING);
    return 1U;
}

uint8_t RoughCalibration_RetractSlide(void)
{
    return RoughCalibration_MoveSlideDeciMm(0U);
}

void RoughCalibration_Abort(void)
{
    if (EdgeMotion_IsActive(&s_edge) ||
        EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
    {
        EdgeMotion_EStop(&s_edge);
    }

    rough_stop_heading();
    if (s_chassis)
        chassis_motion_stop(s_chassis);
    rough_restore_acceleration();

    /* 退出标定按急停语义处理：即使当前静止，也不再相信滑台软件坐标。 */
    ManipulatorAction_Abort();
    if (s_nav.initialized)
        Navigation_Abort(&s_nav.nav);

    rough_cal_error = ROUGH_CAL_ERROR_NONE;
    rough_cal_current_node = 0U;
    rough_cal_selected_ring = 0U;
    rough_cal_home_confirmed = 0U;
    rough_cal_target_slide_mm = 0.0f;
    rough_cal_target_carousel_deg = 0.0f;
    rough_cal_world_yaw_deg = 0.0f;
    rough_set_state(ROUGH_CAL_IDLE);
}

void RoughCalibration_Update(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    RoughCalibrationState_t state = RoughCalibration_GetState();

    if (state == ROUGH_CAL_IDLE || state == ROUGH_CAL_ERROR)
        return;

    if (hwt101_world_zero_ready(imu))
        rough_cal_world_yaw_deg = hwt101_world_yaw_deg(imu);

    if (state == ROUGH_CAL_EXIT_PARK)
    {
        if (chassis_motion_failed(s_chassis))
        {
            rough_fail(ROUGH_CAL_ERROR_PARK);
            return;
        }

        if (chassis_motion_done(s_chassis))
        {
            /* 只有真实完成蓝色区 -> 路口位移后，逻辑位置才正式变成 N3/N9。 */
            rough_cal_current_node = s_start_corner;
            (void)rough_plan_to_n2();
        }
        return;
    }

    if (state == ROUGH_CAL_NAVIGATING)
    {
        EdgeMotion_Update(&s_edge, now_ms);
        rough_cal_current_node = Navigation_GetCurrentNode(&s_nav.nav);

        if (EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
        {
            rough_fail(ROUGH_CAL_ERROR_EDGE_MOTION);
            return;
        }

        if (Navigation_HasError(&s_nav.nav))
        {
            rough_fail(ROUGH_CAL_ERROR_NAVIGATION);
            return;
        }

        if (Navigation_IsDone(&s_nav.nav) && !EdgeMotion_IsActive(&s_edge))
        {
            rough_cal_current_node = MISSION_ROUGH_NODE;
            /* 先进入稳定态，再用与正式任务完全相同的 PD 参数对准 N2 航向。 */
            rough_enter_align_settle();
        }
        return;
    }

    if (state == ROUGH_CAL_ALIGNING)
    {
        float current_yaw;
        float error_world_deg;
        int8_t error_sign;

        if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
        {
            /* IMU 断帧先停，再等待恢复；不允许盲转。 */
            rough_enter_align_settle();
            return;
        }

        current_yaw = hwt101_world_yaw_deg(imu);
        rough_cal_world_yaw_deg = current_yaw;
        error_world_deg = rough_wrap_deg(
            g_nodes[MISSION_ROUGH_NODE].task_yaw_deg - current_yaw);
        error_sign = rough_signf(error_world_deg);

        if (rough_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG ||
            (error_sign != 0 &&
             s_heading_start_sign != 0 &&
             error_sign != s_heading_start_sign))
        {
            rough_enter_align_settle();
            return;
        }

        if ((int32_t)(now_ms - s_next_heading_update_ms) >= 0)
        {
            float world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
            float wz_cmd = heading_update_world_cw(
                &s_heading,
                current_yaw,
                world_gyro_dps,
                (float)MISSION_ALIGN_REFRESH_MS / 1000.0f);

            chassis_set_velocity(s_chassis, 0.0f, 0.0f, wz_cmd, now_ms);
            s_next_heading_update_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
        }
        return;
    }

    if (state == ROUGH_CAL_ALIGN_SETTLE)
    {
        if (chassis_motion_busy(s_chassis)) return;

        if (s_deadline_ms == 0U)
        {
            s_deadline_ms = now_ms + MISSION_ALIGN_SETTLE_MS;
            return;
        }

        if ((int32_t)(now_ms - s_deadline_ms) < 0) return;
        if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
        {
            return;
        }

        rough_begin_alignment(now_ms);
        return;
    }

    if (state == ROUGH_CAL_ARM_MOVING)
    {
        if ((CarouselServo_IsDone() && GripperServo_IsDone()) ||
            (int32_t)(now_ms - s_deadline_ms) >= 0)
        {
            rough_set_state(s_arm_done_state);
        }
        return;
    }

    if (state == ROUGH_CAL_SLIDE_MOVING)
    {
        /* RobotApp 在 VOFA 接管时不再推进机械臂，所以由本标定层独占更新。 */
        ManipulatorAction_Update(now_ms);

        if (ManipulatorAction_SlideJogHasError())
        {
            rough_cal_home_confirmed = 0U;
            rough_fail(ROUGH_CAL_ERROR_MANIPULATOR);
            return;
        }

        if (ManipulatorAction_SlideJogIsDone())
        {
            if (rough_cal_target_slide_mm <= 0.05f)
                rough_set_state(ROUGH_CAL_HOME_READY);
            else
                rough_set_state(ROUGH_CAL_SLIDE_READY);
        }
    }
}

uint8_t RoughCalibration_IsActive(void)
{
    return RoughCalibration_GetState() != ROUGH_CAL_IDLE ? 1U : 0U;
}

RoughCalibrationState_t RoughCalibration_GetState(void)
{
    return (RoughCalibrationState_t)rough_cal_state;
}

uint8_t RoughCalibration_GetError(void)
{
    return rough_cal_error;
}

uint8_t RoughCalibration_GetCurrentNode(void)
{
    return rough_cal_current_node;
}

uint8_t RoughCalibration_GetSelectedRing(void)
{
    return rough_cal_selected_ring;
}

uint8_t RoughCalibration_IsHomeConfirmed(void)
{
    return rough_cal_home_confirmed;
}

float RoughCalibration_GetSlideTargetMm(void)
{
    return rough_cal_target_slide_mm;
}

float RoughCalibration_GetCarouselTargetDeg(void)
{
    return rough_cal_target_carousel_deg;
}
