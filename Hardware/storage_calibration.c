#include "storage_calibration.h"

#include "app_uart.h"
#include "chassis_motion.h"
#include "edge_motion.h"
#include "heading_control.h"
#include "hmi_navigation.h"
#include "hwt101_uart.h"
#include "k230_align.h"
#include "manipulator_action.h"
#include "map_graph.h"
#include "navigation.h"
#include "robot_config.h"
#include "servo.h"

#include <math.h>

/* 只用于角速度上限换算和启停区世界坐标到车体坐标的旋转。 */
#define STORAGE_CAL_TWO_PI_F       6.28318530717958647692f
#define STORAGE_CAL_DEG_TO_RAD_F   0.01745329251994329577f

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
static uint8_t s_align_node;
/*
 * N4二号环“底盘位置 + 航向”联合锁定标志。
 * K230只控制平移，航向PD只控制旋转；两者会相互轻微影响，因此按
 * K230 -> 验航向 ->（若转过）再K230的顺序循环，直到两项同时成立。
 */
static uint8_t s_storage_pose_lock_active;
static uint8_t s_storage_pose_lock_yaw_changed;
/* 云台动作结束后回到ARM_READY还是SLIDE_READY。 */
static StorageCalibrationState_t s_arm_done_state;

volatile uint8_t storage_cal_state = (uint8_t)STORAGE_CAL_IDLE;
volatile uint8_t storage_cal_error = STORAGE_CAL_ERROR_NONE;
volatile uint8_t storage_cal_current_node = 0U;
volatile uint8_t storage_cal_selected_ring = 0U;
volatile uint8_t storage_cal_home_confirmed = 0U;
volatile uint8_t storage_cal_low_vision = 0U;
volatile float storage_cal_target_slide_mm = 0.0f;
volatile float storage_cal_target_carousel_deg = 0.0f;
volatile float storage_cal_target_gimbal_drop_mm = 0.0f;
volatile float storage_cal_world_yaw_deg = 0.0f;

static float storage_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float storage_wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

static int8_t storage_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

/* 与正式任务航向层相同：把轮速RPM上限换算为底盘wz上限。 */
static float storage_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
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
             (STORAGE_CAL_TWO_PI_F * chassis->config.wheel_radius_m);
    if (factor <= 0.0f) return 0.0f;

    return (float)rpm / (k * factor);
}

static void storage_set_state(StorageCalibrationState_t state)
{
    storage_cal_state = (uint8_t)state;
}

static void storage_save_acceleration(void)
{
    if (!s_chassis || s_acceleration_saved) return;
    s_saved_acceleration = s_chassis->acceleration;
    s_acceleration_saved = 1U;
}

static void storage_restore_acceleration(void)
{
    if (!s_chassis || !s_acceleration_saved) return;
    s_chassis->acceleration = s_saved_acceleration;
    s_acceleration_saved = 0U;
}

static void storage_stop_heading(void)
{
    if (s_chassis && s_heading_velocity_active)
        chassis_motion_stop(s_chassis);

    s_heading_velocity_active = 0U;
}

static void storage_fail(uint8_t error)
{
    if (EdgeMotion_IsActive(&s_edge) ||
        EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
    {
        EdgeMotion_EStop(&s_edge);
    }

    storage_stop_heading();
    if (s_chassis)
        chassis_motion_stop(s_chassis);
    storage_restore_acceleration();

    if (K230Align_IsBusy())
        K230Align_Abort();

    /* 运动中失败后真实齿条位置未知，必须阻止继续累计相对位移。 */
    if (ManipulatorAction_SlideJogIsBusy())
    {
        ManipulatorAction_Abort();
        storage_cal_home_confirmed = 0U;
    }

    s_storage_pose_lock_active = 0U;
    s_storage_pose_lock_yaw_changed = 0U;
    storage_cal_error = error;
    storage_set_state(STORAGE_CAL_ERROR);
}

static float storage_ring_angle(uint8_t ring)
{
    if (ring == 1U) return STORAGE_CAROUSEL_POS1_DEG;
    if (ring == 2U) return STORAGE_CAROUSEL_POS2_DEG;
    if (ring == 3U) return STORAGE_CAROUSEL_POS3_DEG;
    return -1.0f;
}

/*
 * 规划下一段拓扑路线。N2到N4在当前图上会由Dijkstra选择N2->N5->N4，
 * EdgeMotion仍负责每条边“先转向、只前行、边末校正”，不在本层写横移捷径。
 */
static uint8_t storage_plan_to(uint8_t target_node,
                               StorageCalibrationState_t nav_state)
{
    if (!HMI_Navigation_PlanTo(&s_nav, target_node))
    {
        storage_fail(STORAGE_CAL_ERROR_NAVIGATION);
        return 0U;
    }

    EdgeMotion_BeginRoute(&s_edge);
    /*
     * 两段导航的终点 N2/N4 都带 task_yaw，到站后标定状态机会立即做任务
     * 航向 PD，所以边末不需要先单独纠到当前边航向。
     */
    EdgeMotion_SetArrivalAlign(&s_edge, 1U);
    HMI_Navigation_UpdateDebugDisplay(&s_nav);
    storage_set_state(nav_state);
    return 1U;
}

/* STOP真正发完并稳定后重新验角；未对准就继续PD，不限制尝试次数。 */
static void storage_enter_align_settle(uint8_t node)
{
    storage_stop_heading();
    s_align_node = node;
    s_deadline_ms = 0U;

    storage_set_state((node == MISSION_ROUGH_NODE) ?
                      STORAGE_CAL_ROUGH_ALIGN_SETTLE :
                      STORAGE_CAL_STORAGE_ALIGN_SETTLE);
}

/*
 * 航向对准成功后的分流：
 *   N2：只做正式任务的2号参考环视觉定位，不进入粗加工抓放程序；
 *   N4：车辆停住，直接开放暂存区参数标定。
 */
static void storage_alignment_done(uint32_t now_ms)
{
    storage_stop_heading();
    storage_restore_acceleration();

    if (s_align_node == MISSION_ROUGH_NODE)
    {
        /*
         * 途经N2时必须与本轮标定高度一致：旧入口复现高位相机姿态；低位
         * 入口复现正式比赛的“伸到2号长度后下降电机5”姿态。两套参数不能
         * 混在一次启停区2->N4路线中，否则N2底盘基准本身就不一致。
         */
        if (!(storage_cal_low_vision ?
              ManipulatorAction_SlideJogToRoughAlignStart() :
              ManipulatorAction_SlideJogToRoughAlignHighStart()))
        {
            storage_fail(STORAGE_CAL_ERROR_MANIPULATOR);
            return;
        }

        storage_cal_target_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
        storage_cal_target_slide_mm = storage_cal_low_vision ?
                                      ROUGH_VISION_LOW_SLIDE_POS2_MM :
                                      ROUGH_VISION_HIGH_SLIDE_POS2_MM;
        storage_set_state(STORAGE_CAL_ROUGH_PRESET);
        return;
    }

    storage_cal_current_node = MISSION_STORAGE_NODE;

    if (s_storage_pose_lock_active)
    {
        /*
         * 这是0x88后的航向复核，而不是首次到达N4。
         * 如果刚才PD确实转过车，摄像头绕车体中心发生了小位移，之前的
         * K230圆心结果已经不能直接沿用，必须在保持当前2号机械位姿的
         * 条件下再跑一次PLACE-2。若本轮根本没转，说明“刚完成的K230”
         * 和“当前航向合格”同时成立，可以把控制权交还给手工调参。
         */
        if (s_storage_pose_lock_yaw_changed)
        {
            s_storage_pose_lock_yaw_changed = 0U;

            if (!K230Align_StartPreset(K230_ALIGN_MODE_PLACE,
                                       K230_ALIGN_PLACE_ID,
                                       now_ms))
            {
                storage_fail(STORAGE_CAL_ERROR_K230);
                return;
            }

            storage_set_state(STORAGE_CAL_STORAGE_VISION_ALIGN);
            return;
        }

        s_storage_pose_lock_active = 0U;
        storage_set_state(STORAGE_CAL_SLIDE_READY);
        return;
    }

    /* 首次抵达N4：滑台仍在0 mm，先停住，等待操作者选择暂存位并调参。 */
    storage_cal_selected_ring = 0U;
    storage_cal_target_carousel_deg = 0.0f;
    storage_cal_target_slide_mm = 0.0f;
    storage_set_state(STORAGE_CAL_HOME_READY);
}

static void storage_begin_alignment(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    float current_yaw;
    float error_world_deg;
    float world_gyro_dps;
    float wz_cmd;

    if (!s_chassis || s_align_node < 1U || s_align_node > NODE_NUM ||
        !hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
        !hwt101_world_zero_ready(imu))
    {
        return;
    }

    current_yaw = hwt101_world_yaw_deg(imu);
    storage_cal_world_yaw_deg = current_yaw;
    error_world_deg = storage_wrap_deg(
        g_nodes[s_align_node].task_yaw_deg - current_yaw);

    if (storage_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG)
    {
        storage_alignment_done(now_ms);
        return;
    }

    if (chassis_motion_busy(s_chassis)) return;

    heading_set_target(&s_heading, g_nodes[s_align_node].task_yaw_deg);
    world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
    wz_cmd = heading_update_world_cw(
        &s_heading,
        current_yaw,
        world_gyro_dps,
        (float)MISSION_ALIGN_REFRESH_MS / 1000.0f);

    storage_save_acceleration();
    s_chassis->acceleration = MISSION_ALIGN_ACC;
    chassis_set_velocity(s_chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    /* 0x88联合锁定期间只要PD真的下发过旋转，随后就必须再做一次K230复查。 */
    if (s_storage_pose_lock_active && s_align_node == MISSION_STORAGE_NODE)
        s_storage_pose_lock_yaw_changed = 1U;

    s_heading_start_sign = storage_signf(error_world_deg);
    s_next_heading_update_ms = now_ms + MISSION_ALIGN_REFRESH_MS;
    s_heading_velocity_active = 1U;
    storage_set_state((s_align_node == MISSION_ROUGH_NODE) ?
                      STORAGE_CAL_ROUGH_ALIGNING :
                      STORAGE_CAL_STORAGE_ALIGNING);
}

void StorageCalibration_Init(Chassis *chassis)
{
    s_chassis = chassis;
    HMI_Navigation_Init(&s_nav);
    EdgeMotion_Init(&s_edge, chassis, &s_nav);

    heading_init(&s_heading,
                 MISSION_ALIGN_KP,
                 MISSION_ALIGN_KD,
                 storage_wz_for_rpm(chassis, MISSION_ALIGN_MAX_RPM),
                 MISSION_ALIGN_TOLERANCE_DEG);
    heading_set_min_output(
        &s_heading,
        storage_wz_for_rpm(chassis, MISSION_ALIGN_MIN_RPM));

    s_deadline_ms = 0U;
    s_next_heading_update_ms = 0U;
    s_heading_start_sign = 0;
    s_heading_velocity_active = 0U;
    s_saved_acceleration = 0U;
    s_acceleration_saved = 0U;
    s_align_node = 0U;
    s_storage_pose_lock_active = 0U;
    s_storage_pose_lock_yaw_changed = 0U;
    s_arm_done_state = STORAGE_CAL_ARM_READY;

    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_cal_current_node = 0U;
    storage_cal_selected_ring = 0U;
    storage_cal_home_confirmed = 0U;
    storage_cal_low_vision = 0U;
    storage_cal_target_slide_mm = 0.0f;
    storage_cal_target_carousel_deg = 0.0f;
    storage_cal_target_gimbal_drop_mm = 0.0f;
    storage_cal_world_yaw_deg = 0.0f;
    storage_set_state(STORAGE_CAL_IDLE);
}

static uint8_t storage_start_from_bay2_internal(uint8_t confirm_key,
                                                uint32_t now_ms,
                                                uint8_t low_vision)
{
    Hwt101 *imu = AppUart_GetImu();
    float park_x;
    float park_y;
    float world_dx;
    float world_dy;
    float yaw_rad;
    float c;
    float sn;

    if (confirm_key != STORAGE_CAL_HOME_CONFIRM_KEY)
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    if (!s_chassis || StorageCalibration_IsActive() ||
        chassis_motion_busy(s_chassis) || K230Align_IsBusy() ||
        ManipulatorAction_IsBusy() || ManipulatorAction_SlideJogIsBusy())
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    /* 世界0度必须建立在新鲜IMU数据和“车头朝地图+Y”的实车姿态上。 */
    if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS))
    {
        storage_cal_error = STORAGE_CAL_ERROR_IMU;
        return 0U;
    }

    HMI_Navigation_Init(&s_nav);
    if (!Navigation_Init(&s_nav.nav, 3U, 0UL))
    {
        storage_cal_error = STORAGE_CAL_ERROR_NAVIGATION;
        return 0U;
    }

    /*
     * 0x70接管会按急停语义作废滑台坐标。A5表示操作者已经确认滑台物理
     * 缩到底；这里只把它立为软件0 mm，不会驱动电机寻找原点。
     */
    /*
     * 若上一轮低位标定被急停，电机5可能仍停在下降位置。不能仅凭A5把滑台
     * 重新立零后继续，否则途经N2时会再次叠加预下降。应先人工恢复最高点
     * 或复位控制器，使视觉下降记录回到0，再重新启动标定。
     */
    if (storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) > 0.05f ||
        !ManipulatorAction_SlideJogRebaseHome())
    {
        storage_cal_error = STORAGE_CAL_ERROR_HOME;
        return 0U;
    }

    hwt101_set_world_zero(imu);
    if (!hwt101_world_zero_ready(imu))
    {
        storage_cal_error = STORAGE_CAL_ERROR_IMU;
        return 0U;
    }

    s_nav.initialized = 1U;
    s_nav.map.start_node = 3U;
    s_nav.map.obstacle_mask = 0UL;
    EdgeMotion_Init(&s_edge, s_chassis, &s_nav);

    /* RobotApp_Init曾把K230Align绑定到正式底盘；VOFA接管后改绑调试底盘。 */
    K230Align_Init(s_chassis);

    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_cal_current_node = 0U;
    storage_cal_selected_ring = 0U;
    storage_cal_home_confirmed = 1U;
    storage_cal_low_vision = low_vision ? 1U : 0U;
    storage_cal_target_slide_mm = 0.0f;
    storage_cal_target_carousel_deg = 0.0f;
    storage_cal_target_gimbal_drop_mm = 0.0f;
    storage_cal_world_yaw_deg = 0.0f;
    s_deadline_ms = 0U;
    s_next_heading_update_ms = 0U;
    s_heading_start_sign = 0;
    s_heading_velocity_active = 0U;
    s_acceleration_saved = 0U;
    s_align_node = 0U;
    s_storage_pose_lock_active = 0U;
    s_storage_pose_lock_yaw_changed = 0U;

    /*
     * 启停区2不属于路网节点。先按正式地图坐标从(2.25,0.15)退出到
     * N3(2.10,0.35)：车头向上时等价于车体左移150 mm、前进200 mm。
     */
    if (!MapGraph_GetParkingPose(3U, &park_x, &park_y))
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARK;
        return 0U;
    }

    world_dx = g_nodes[3U].x - park_x;
    world_dy = g_nodes[3U].y - park_y;
    yaw_rad = hwt101_world_yaw_deg(imu) * STORAGE_CAL_DEG_TO_RAD_F;
    c = cosf(yaw_rad);
    sn = sinf(yaw_rad);

    chassis_motion_translate(s_chassis,
                             world_dx * c - world_dy * sn,
                             world_dx * sn + world_dy * c,
                             ROUTE_EDGE_SPEED_MPS,
                             now_ms);
    storage_set_state(STORAGE_CAL_EXIT_PARK);
    return 1U;
}

uint8_t StorageCalibration_StartFromBay2(uint8_t confirm_key,
                                         uint32_t now_ms)
{
    /* 旧高位识别流程原样保留，便于与已经记录的旧标定值做对照。 */
    return storage_start_from_bay2_internal(confirm_key, now_ms, 0U);
}

uint8_t StorageCalibration_StartFromBay2Low(uint8_t confirm_key,
                                            uint32_t now_ms)
{
    /* 新低位识别流程：N2途中基准和N4手工标定均复现正式预下降高度。 */
    return storage_start_from_bay2_internal(confirm_key, now_ms, 1U);
}

uint8_t StorageCalibration_SelectRing(uint8_t ring, uint32_t now_ms)
{
    float angle_deg = storage_ring_angle(ring);

    if (angle_deg < 0.0f)
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    /* 换暂存位只允许在可信0 mm处进行，避免伸出的夹爪横扫场地。 */
    if (StorageCalibration_GetState() != STORAGE_CAL_HOME_READY ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) > 0.05f ||
        storage_absf(ManipulatorAction_SlideJogGetPositionMm()) > 0.05f)
    {
        storage_cal_error = STORAGE_CAL_ERROR_HOME;
        return 0U;
    }

    CarouselServo_SetAngle(angle_deg);
    GripperServo_Open_Large();
    storage_cal_selected_ring = ring;
    storage_cal_target_carousel_deg = angle_deg;
    s_arm_done_state = STORAGE_CAL_ARM_READY;
    s_deadline_ms = now_ms + K230_ALIGN_ROTATE_TIMEOUT_MS;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_set_state(STORAGE_CAL_ARM_MOVING);
    return 1U;
}

uint8_t StorageCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                               uint32_t now_ms)
{
    float target_deg = (float)target_deci_deg * 0.1f;
    StorageCalibrationState_t state = StorageCalibration_GetState();

    if (target_deg < STORAGE_CAROUSEL_CAL_MIN_DEG ||
        target_deg > STORAGE_CAROUSEL_CAL_MAX_DEG)
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    if ((state != STORAGE_CAL_ARM_READY && state != STORAGE_CAL_SLIDE_READY) ||
        storage_cal_selected_ring < 1U || storage_cal_selected_ring > 3U ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    CarouselServo_SetAngle(target_deg);
    storage_cal_target_carousel_deg = target_deg;
    s_arm_done_state = (state == STORAGE_CAL_SLIDE_READY) ?
                       STORAGE_CAL_SLIDE_READY : STORAGE_CAL_ARM_READY;
    s_deadline_ms = now_ms + K230_ALIGN_ROTATE_TIMEOUT_MS;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_set_state(STORAGE_CAL_ARM_MOVING);
    return 1U;
}

uint8_t StorageCalibration_MoveSlideDeciMm(uint16_t target_deci_mm)
{
    float target_mm = (float)target_deci_mm * 0.1f;
    float current_mm = ManipulatorAction_SlideJogGetPositionMm();
    StorageCalibrationState_t state = StorageCalibration_GetState();

    if (target_mm < 0.0f || target_mm > MANIP_SLIDE_SOFT_MAX_MM)
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    if ((state != STORAGE_CAL_ARM_READY && state != STORAGE_CAL_SLIDE_READY) ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    /*
     * 与粗加工区保持相同调参手感：丝杆下降到实际落料高度后仍允许小步
     * 调整齿条，以便直接观察夹爪中心。单条命令最多改变10 mm；回到0 mm
     * 属于明确的安全回收动作，不受本步长限制，并仍会先抬丝杆再收齿条。
     */
    if (storage_cal_low_vision && state == STORAGE_CAL_SLIDE_READY &&
        target_mm > 0.05f &&
        storage_absf(ManipulatorAction_SlideJogGetVisionDropMm() +
                      STORAGE_VISION_PRE_DROP_MM) > 0.1f &&
         storage_absf(target_mm - current_mm) >
             (STORAGE_LOW_CAL_MAX_SLIDE_STEP_MM + 0.05f))
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    /*
     * ==================== 高位/低位识别分流 ====================
     * 高位模式（旧0x87）只移动滑台；低位模式（新0x8A）第一次伸出后下降
     * 电机5，后续细调只动滑台。低位回零必须使用HomeStart，确保顺序是
     * “先抬回最高点，再收回齿条”，与正式任务完全相同。
     */
    if (storage_cal_low_vision && target_mm <= 0.05f &&
        storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) > 0.05f)
    {
        if (!ManipulatorAction_SlideJogHomeStart())
        {
            storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
            return 0U;
        }
        storage_cal_target_gimbal_drop_mm = 0.0f;
    }
    else if (storage_cal_low_vision && target_mm > 0.05f &&
             storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) <= 0.05f)
    {
        if (!ManipulatorAction_SlideJogToVisionPositionStart(
                target_mm, STORAGE_VISION_PRE_DROP_MM))
        {
            storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
            return 0U;
        }
        storage_cal_target_gimbal_drop_mm = STORAGE_VISION_PRE_DROP_MM;
    }
    else if (!ManipulatorAction_SlideJogToPositionStart(target_mm))
    {
        storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
        return 0U;
    }

    storage_cal_target_slide_mm = target_mm;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_set_state(STORAGE_CAL_SLIDE_MOVING);
    return 1U;
}

uint8_t StorageCalibration_MoveGimbalDropDeciMm(uint16_t target_deci_mm)
{
    float target_drop_mm = (float)target_deci_mm * 0.1f;
    float max_drop_mm = -MANIP_PICKUP_GROUND_MM;
    float current_drop_mm = -ManipulatorAction_SlideJogGetVisionDropMm();

    /*
     * 只接受0x8A低位流程。100 mm是当前正式取像高度，185 mm是第一层
     * 地面放置高度；125 mm（-185+60）自然落在区间内，可验证第二层叠放。
     * 上限硬锁在正式第一层高度，任何串口误码都不能让丝杆继续向地面下压。
     */
    if (!storage_cal_low_vision ||
        target_drop_mm < (STORAGE_VISION_PRE_DROP_MM - 0.05f) ||
        target_drop_mm > (max_drop_mm + 0.05f))
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    if (StorageCalibration_GetState() != STORAGE_CAL_SLIDE_READY ||
        storage_cal_current_node != MISSION_STORAGE_NODE ||
        storage_cal_selected_ring < 1U || storage_cal_selected_ring > 3U ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy() ||
        chassis_motion_busy(s_chassis) || K230Align_IsBusy() ||
        storage_cal_target_slide_mm <= 0.05f ||
        /* 1/2/3号均允许从最高点或观察位直达100~185 mm绝对高度。 */
        (current_drop_mm > 0.1f &&
         current_drop_mm < (STORAGE_VISION_PRE_DROP_MM - 0.1f)))
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    if (!ManipulatorAction_SlideJogToVisionDropStart(target_drop_mm))
    {
        storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
        return 0U;
    }

    storage_cal_target_gimbal_drop_mm = target_drop_mm;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    /* 复用已有非阻塞等待态；齿条目标保持不变，完成后仍回SLIDE_READY。 */
    storage_set_state(STORAGE_CAL_SLIDE_MOVING);
    return 1U;
}

uint8_t StorageCalibration_RaiseGimbalToTop(void)
{
    float current_drop_mm = -ManipulatorAction_SlideJogGetVisionDropMm();

    if (!storage_cal_low_vision ||
        StorageCalibration_GetState() != STORAGE_CAL_SLIDE_READY ||
        storage_cal_current_node != MISSION_STORAGE_NODE ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy() ||
        chassis_motion_busy(s_chassis) || K230Align_IsBusy() ||
        current_drop_mm <= 0.05f)
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    if (!ManipulatorAction_SlideJogRaiseGimbalOnlyStart())
    {
        storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
        return 0U;
    }

    storage_cal_target_gimbal_drop_mm = 0.0f;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    /* 齿条仍保持伸出，抬升完成后Update会重新回到SLIDE_READY。 */
    storage_set_state(STORAGE_CAL_SLIDE_MOVING);
    return 1U;
}

uint8_t StorageCalibration_RetractSlideOnly(void)
{
    if (StorageCalibration_GetState() != STORAGE_CAL_SLIDE_READY ||
        storage_cal_current_node != MISSION_STORAGE_NODE ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy() ||
        chassis_motion_busy(s_chassis) || K230Align_IsBusy() ||
        storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) > 0.05f)
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    if (!ManipulatorAction_SlideJogRetractSlideOnlyStart())
    {
        storage_cal_error = STORAGE_CAL_ERROR_MANIPULATOR;
        return 0U;
    }

    storage_cal_target_slide_mm = 0.0f;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_set_state(STORAGE_CAL_SLIDE_MOVING);
    return 1U;
}

uint8_t StorageCalibration_RetractSlide(void)
{
    return StorageCalibration_MoveSlideDeciMm(0U);
}

uint8_t StorageCalibration_StartStorageVisionAlign(uint32_t now_ms)
{
    StorageCalibrationState_t state = StorageCalibration_GetState();
    float actual_slide_mm = ManipulatorAction_SlideJogGetPositionMm();

    /*
     * 只接受“已经在N4、2号环、机械姿态完全静止”的请求。
     * 1/3号位的云台角不同，K230 w/a/s/d 到车体系的固定映射不再成立，
     * 因而绝不能借0x88移动底盘。
     */
    if (state != STORAGE_CAL_SLIDE_READY ||
        storage_cal_current_node != MISSION_STORAGE_NODE ||
        storage_cal_selected_ring != K230_ALIGN_PLACE_ID ||
        !storage_cal_home_confirmed ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy() ||
        chassis_motion_busy(s_chassis) || K230Align_IsBusy())
    {
        storage_cal_error = STORAGE_CAL_ERROR_BUSY;
        return 0U;
    }

    /*
     * 2号环必须保持273.33 deg识别角；齿条则使用操作者本轮已经调好的当前值，
     * 低位从95 mm相机基础值开始，后续候选长度无需改宏或重新烧录即可验证。
     */
    if (storage_absf(storage_cal_target_carousel_deg -
                     CAROUSEL_PICKUP_POS2_DEG) > 0.1f ||
        actual_slide_mm <= 0.05f ||
        storage_absf(actual_slide_mm - storage_cal_target_slide_mm) > 0.05f)
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    /*
     * 防止高/低标定入口和真实电机5高度串线：高位版必须仍在最高原点；
     * 低位版必须已经完成与正式流程相同的STORAGE预下降。只有高度匹配后
     * 才允许K230移动底盘，否则会把不同透视高度的像素中心混成一套参数。
     */
    if ((!storage_cal_low_vision &&
         storage_absf(ManipulatorAction_SlideJogGetVisionDropMm()) > 0.05f) ||
        (storage_cal_low_vision &&
         storage_absf(ManipulatorAction_SlideJogGetVisionDropMm() +
                      STORAGE_VISION_PRE_DROP_MM) > 0.1f))
    {
        storage_cal_error = STORAGE_CAL_ERROR_PARAM;
        return 0U;
    }

    if (!K230Align_StartPreset(K230_ALIGN_MODE_PLACE,
                               K230_ALIGN_PLACE_ID,
                               now_ms))
    {
        storage_cal_error = STORAGE_CAL_ERROR_K230;
        return 0U;
    }

    s_storage_pose_lock_active = 1U;
    s_storage_pose_lock_yaw_changed = 0U;
    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_set_state(STORAGE_CAL_STORAGE_VISION_ALIGN);
    return 1U;
}

void StorageCalibration_Abort(void)
{
    if (EdgeMotion_IsActive(&s_edge) ||
        EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
    {
        EdgeMotion_EStop(&s_edge);
    }

    storage_stop_heading();
    if (s_chassis)
        chassis_motion_stop(s_chassis);
    storage_restore_acceleration();
    K230Align_Abort();

    /* 急停后不猜齿条停在何处；下次必须物理回原点并重新用A5启动。 */
    ManipulatorAction_Abort();
    if (s_nav.initialized)
        Navigation_Abort(&s_nav.nav);

    storage_cal_error = STORAGE_CAL_ERROR_NONE;
    storage_cal_current_node = 0U;
    storage_cal_selected_ring = 0U;
    storage_cal_home_confirmed = 0U;
    storage_cal_low_vision = 0U;
    storage_cal_target_slide_mm = 0.0f;
    storage_cal_target_carousel_deg = 0.0f;
    storage_cal_target_gimbal_drop_mm = 0.0f;
    storage_cal_world_yaw_deg = 0.0f;
    s_align_node = 0U;
    s_storage_pose_lock_active = 0U;
    s_storage_pose_lock_yaw_changed = 0U;
    storage_set_state(STORAGE_CAL_IDLE);
}

void StorageCalibration_Update(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    StorageCalibrationState_t state = StorageCalibration_GetState();

    if (state == STORAGE_CAL_IDLE || state == STORAGE_CAL_ERROR)
        return;

    if (hwt101_world_zero_ready(imu))
        storage_cal_world_yaw_deg = hwt101_world_yaw_deg(imu);

    if (state == STORAGE_CAL_EXIT_PARK)
    {
        if (chassis_motion_failed(s_chassis))
        {
            storage_fail(STORAGE_CAL_ERROR_PARK);
            return;
        }

        if (chassis_motion_done(s_chassis))
        {
            storage_cal_current_node = 3U;
            (void)storage_plan_to(MISSION_ROUGH_NODE,
                                  STORAGE_CAL_NAV_TO_ROUGH);
        }
        return;
    }

    if (state == STORAGE_CAL_NAV_TO_ROUGH ||
        state == STORAGE_CAL_NAV_TO_STORAGE)
    {
        uint8_t target_node = (state == STORAGE_CAL_NAV_TO_ROUGH) ?
                              MISSION_ROUGH_NODE : MISSION_STORAGE_NODE;

        EdgeMotion_Update(&s_edge, now_ms);
        storage_cal_current_node = Navigation_GetCurrentNode(&s_nav.nav);

        if (EdgeMotion_GetState(&s_edge) == EDGE_MOTION_ERROR)
        {
            storage_fail(STORAGE_CAL_ERROR_EDGE_MOTION);
            return;
        }

        if (Navigation_HasError(&s_nav.nav))
        {
            storage_fail(STORAGE_CAL_ERROR_NAVIGATION);
            return;
        }

        if (Navigation_IsDone(&s_nav.nav) && !EdgeMotion_IsActive(&s_edge))
        {
            storage_cal_current_node = target_node;
            storage_enter_align_settle(target_node);
        }
        return;
    }

    if (state == STORAGE_CAL_ROUGH_ALIGNING ||
        state == STORAGE_CAL_STORAGE_ALIGNING)
    {
        float current_yaw;
        float error_world_deg;
        int8_t error_sign;

        if (!hwt101_is_fresh(imu, now_ms, MISSION_ALIGN_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
        {
            storage_enter_align_settle(s_align_node);
            return;
        }

        current_yaw = hwt101_world_yaw_deg(imu);
        storage_cal_world_yaw_deg = current_yaw;
        error_world_deg = storage_wrap_deg(
            g_nodes[s_align_node].task_yaw_deg - current_yaw);
        error_sign = storage_signf(error_world_deg);

        if (storage_absf(error_world_deg) <= MISSION_ALIGN_TOLERANCE_DEG ||
            (error_sign != 0 && s_heading_start_sign != 0 &&
             error_sign != s_heading_start_sign))
        {
            storage_enter_align_settle(s_align_node);
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

    if (state == STORAGE_CAL_ROUGH_ALIGN_SETTLE ||
        state == STORAGE_CAL_STORAGE_ALIGN_SETTLE)
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

        storage_begin_alignment(now_ms);
        return;
    }

    if (state == STORAGE_CAL_ROUGH_PRESET)
    {
        /* RobotApp被VOFA挂起后不再更新机械臂，标定层必须独占推进。 */
        ManipulatorAction_Update(now_ms);

        if (ManipulatorAction_SlideJogHasError())
        {
            storage_cal_home_confirmed = 0U;
            storage_fail(STORAGE_CAL_ERROR_MANIPULATOR);
            return;
        }

        if (ManipulatorAction_SlideJogIsDone())
        {
            if (!K230Align_StartPreset(K230_ALIGN_MODE_PLACE,
                                       K230_ALIGN_PLACE_ID,
                                       now_ms))
            {
                storage_fail(STORAGE_CAL_ERROR_K230);
                return;
            }

            storage_set_state(STORAGE_CAL_ROUGH_VISION_ALIGN);
        }
        return;
    }

    if (state == STORAGE_CAL_ROUGH_VISION_ALIGN)
    {
        K230Align_Update(now_ms);

        if (K230Align_HasError())
        {
            storage_fail(STORAGE_CAL_ERROR_K230);
            return;
        }

        if (K230Align_IsDone())
        {
            /* 只定位，不调用任何粗加工ZoneTask；先把相机齿条退回原点。 */
            if (!ManipulatorAction_SlideJogHomeStart())
            {
                storage_fail(STORAGE_CAL_ERROR_MANIPULATOR);
                return;
            }

            storage_cal_target_slide_mm = 0.0f;
            storage_set_state(STORAGE_CAL_ROUGH_RETRACT);
        }
        return;
    }

    if (state == STORAGE_CAL_STORAGE_VISION_ALIGN)
    {
        K230Align_Update(now_ms);

        if (K230Align_HasError())
        {
            storage_fail(STORAGE_CAL_ERROR_K230);
            return;
        }

        if (K230Align_IsDone())
        {
            /*
             * K230只保证圆心像素，不保证底盘仍是180 deg。先进入已有的
             * STOP->稳定->PD验角链；storage_alignment_done()会根据本轮
             * 是否真的转过，决定完成或再做一次K230。
             */
            s_storage_pose_lock_yaw_changed = 0U;
            storage_enter_align_settle(MISSION_STORAGE_NODE);
        }
        return;
    }

    if (state == STORAGE_CAL_ROUGH_RETRACT)
    {
        ManipulatorAction_Update(now_ms);

        if (ManipulatorAction_SlideJogHasError())
        {
            storage_cal_home_confirmed = 0U;
            storage_fail(STORAGE_CAL_ERROR_MANIPULATOR);
            return;
        }

        if (ManipulatorAction_SlideJogIsDone())
        {
            /* K230修正后的物理位置被认作准确N2，从这里继续正式拓扑路线。 */
            storage_cal_current_node = MISSION_ROUGH_NODE;
            (void)storage_plan_to(MISSION_STORAGE_NODE,
                                  STORAGE_CAL_NAV_TO_STORAGE);
        }
        return;
    }

    if (state == STORAGE_CAL_ARM_MOVING)
    {
        if ((CarouselServo_IsDone() && GripperServo_IsDone()) ||
            (int32_t)(now_ms - s_deadline_ms) >= 0)
        {
            storage_set_state(s_arm_done_state);
        }
        return;
    }

    if (state == STORAGE_CAL_SLIDE_MOVING)
    {
        ManipulatorAction_Update(now_ms);

        if (ManipulatorAction_SlideJogHasError())
        {
            storage_cal_home_confirmed = 0U;
            storage_fail(STORAGE_CAL_ERROR_MANIPULATOR);
            return;
        }

        if (ManipulatorAction_SlideJogIsDone())
        {
            if (storage_cal_target_slide_mm <= 0.05f)
                storage_set_state(STORAGE_CAL_HOME_READY);
            else
                storage_set_state(STORAGE_CAL_SLIDE_READY);
        }
    }
}

uint8_t StorageCalibration_IsActive(void)
{
    return StorageCalibration_GetState() != STORAGE_CAL_IDLE ? 1U : 0U;
}

uint8_t StorageCalibration_AllowsManualVision(void)
{
    StorageCalibrationState_t state = StorageCalibration_GetState();
    return (state == STORAGE_CAL_ARM_READY ||
            state == STORAGE_CAL_SLIDE_READY) ? 1U : 0U;
}

StorageCalibrationState_t StorageCalibration_GetState(void)
{
    return (StorageCalibrationState_t)storage_cal_state;
}

uint8_t StorageCalibration_GetError(void)
{
    return storage_cal_error;
}

uint8_t StorageCalibration_GetCurrentNode(void)
{
    return storage_cal_current_node;
}

uint8_t StorageCalibration_GetSelectedRing(void)
{
    return storage_cal_selected_ring;
}

uint8_t StorageCalibration_IsLowVision(void)
{
    return storage_cal_low_vision;
}

float StorageCalibration_GetSlideTargetMm(void)
{
    return storage_cal_target_slide_mm;
}

float StorageCalibration_GetCarouselTargetDeg(void)
{
    return storage_cal_target_carousel_deg;
}

float StorageCalibration_GetGimbalDropTargetMm(void)
{
    return storage_cal_target_gimbal_drop_mm;
}
