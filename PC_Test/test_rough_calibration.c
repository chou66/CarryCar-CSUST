#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_uart.h"
#include "edge_motion.h"
#include "hmi_navigation.h"
#include "k230_align.h"
#include "manipulator_action.h"
#include "map_graph.h"
#include "robot_config.h"
#include "rough_calibration.h"
#include "servo.h"

/*
 * 本测试只验证粗加工标定状态机的互锁与绝对位置语义。
 * 正式 EdgeMotion/HeadingController 已各有独立 PC 测试；这里用最小桩函数
 * 模拟“已经到 N2”，避免把同一底层算法重复测一遍。
 */

static Hwt101 s_imu;
static float s_world_yaw_deg;
static uint8_t s_edge_active;
static EdgeMotionState_t s_edge_state;
static uint8_t s_chassis_busy;
static uint8_t s_position_done;
static uint8_t s_position_failed;
static float s_translate_dx_m;
static float s_translate_dy_m;
static uint8_t s_slide_valid;
static uint8_t s_slide_busy;
static uint8_t s_slide_done;
static float s_slide_mm;
static float s_slide_target_mm;
static float s_vision_drop_mm;
static uint8_t s_low_vision_start_count;
static uint8_t s_low_vision_home_count;
static uint8_t s_vision_height_jog_count;
static uint8_t s_servo_done;
static float s_carousel_deg;
static uint8_t s_gripper_open_count;
static uint8_t s_planned_target;
static uint8_t s_k230_busy;
static uint8_t s_k230_done;
static uint8_t s_k230_error;
static Chassis *s_k230_chassis;
static int s_pass;
static int s_fail;

MapNode_t g_nodes[NODE_NUM + 1] =
{
    {0},
    {0.35f, 0.35f, MAP_TASK_NONE,    0U,   0.0f},
    {1.20f, 0.35f, MAP_TASK_ROUGH,   1U,  90.0f},
    {2.10f, 0.35f, MAP_TASK_NONE,    0U,   0.0f},
    {0.35f, 1.20f, MAP_TASK_STORAGE, 1U, 180.0f},
    {1.20f, 1.20f, MAP_TASK_NONE,    0U,   0.0f},
    {2.10f, 1.20f, MAP_TASK_QR,      1U,   0.0f},
    {0.35f, 2.10f, MAP_TASK_NONE,    0U,   0.0f},
    {1.20f, 2.10f, MAP_TASK_RAW,     1U, -90.0f},
    {2.10f, 2.10f, MAP_TASK_NONE,    0U,   0.0f}
};
uint16_t g_graph[NODE_NUM + 1][NODE_NUM + 1];

uint8_t MapGraph_GetParkingPose(uint8_t corner_node, float *x, float *y)
{
    if (!x || !y) return 0U;

    if (corner_node == 3U)
    {
        *x = 2.25f;
        *y = 0.15f;
        return 1U;
    }
    if (corner_node == 9U)
    {
        *x = 2.25f;
        *y = 2.25f;
        return 1U;
    }
    return 0U;
}

static int nearf_local(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

/* ---- IMU stubs ---- */
Hwt101 *AppUart_GetImu(void) { return &s_imu; }
bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms)
{
    return imu && imu->has_angle &&
           (uint32_t)(now_ms - imu->last_update_ms) <= timeout_ms;
}
void hwt101_set_world_zero(Hwt101 *imu)
{
    if (imu) imu->world_zero_ready = true;
}
bool hwt101_world_zero_ready(const Hwt101 *imu)
{
    return imu && imu->world_zero_ready;
}
float hwt101_world_yaw_deg(const Hwt101 *imu)
{
    (void)imu;
    return s_world_yaw_deg;
}

/* ---- HMI Navigation / Navigation stubs ---- */
void HMI_Navigation_Init(HMI_Navigation_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}
uint8_t HMI_Navigation_PlanTo(HMI_Navigation_t *ctx, uint8_t target_node)
{
    s_planned_target = target_node;
    ctx->nav.target_node = target_node;
    ctx->nav.current_node = target_node;
    ctx->nav.state = NAV_STATE_DONE;
    return 1U;
}
void HMI_Navigation_UpdateDebugDisplay(const HMI_Navigation_t *ctx)
{
    (void)ctx;
}
uint8_t Navigation_Init(Navigation_t *nav, uint8_t start_node,
                        uint32_t obstacle_mask)
{
    memset(nav, 0, sizeof(*nav));
    nav->current_node = start_node;
    nav->obstacle_mask = obstacle_mask;
    nav->state = NAV_STATE_IDLE;
    return 1U;
}
uint8_t Navigation_GetCurrentNode(const Navigation_t *nav)
{
    return nav ? nav->current_node : 0U;
}
uint8_t Navigation_HasError(const Navigation_t *nav)
{
    return !nav || nav->state == NAV_STATE_ERROR || nav->state == NAV_STATE_NO_PATH;
}
uint8_t Navigation_IsDone(const Navigation_t *nav)
{
    return nav && nav->state == NAV_STATE_DONE;
}
void Navigation_Abort(Navigation_t *nav)
{
    if (nav) nav->state = NAV_STATE_IDLE;
}

/* ---- EdgeMotion stubs ---- */
void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis,
                     HMI_Navigation_t *hmi_nav)
{
    memset(em, 0, sizeof(*em));
    em->chassis = chassis;
    em->hmi_nav = hmi_nav;
    em->state = EDGE_MOTION_IDLE;
    s_edge_state = EDGE_MOTION_IDLE;
    s_edge_active = 0U;
}
void EdgeMotion_BeginRoute(EdgeMotion_t *em) { (void)em; }
void EdgeMotion_SetArrivalAlign(EdgeMotion_t *em, uint8_t valid)
{ (void)em; (void)valid; }
void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms)
{ (void)em; (void)now_ms; }
void EdgeMotion_EStop(EdgeMotion_t *em)
{ if (em) em->state = EDGE_MOTION_IDLE; s_edge_active = 0U; }
EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em)
{ (void)em; return s_edge_state; }
uint8_t EdgeMotion_IsActive(const EdgeMotion_t *em)
{ (void)em; return s_edge_active; }

/* ---- Chassis stubs ---- */
bool chassis_motion_busy(const Chassis *chassis)
{ (void)chassis; return s_chassis_busy != 0U; }
void chassis_motion_stop(Chassis *chassis)
{ (void)chassis; s_chassis_busy = 0U; }
void chassis_motion_translate(Chassis *chassis,
                              float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms)
{
    (void)chassis; (void)speed_mps; (void)now_ms;
    s_translate_dx_m = dx_m;
    s_translate_dy_m = dy_m;
    s_position_done = 0U;
    s_position_failed = 0U;
    s_chassis_busy = 1U;
}
bool chassis_motion_done(const Chassis *chassis)
{ (void)chassis; return s_position_done != 0U; }
bool chassis_motion_failed(const Chassis *chassis)
{ (void)chassis; return s_position_failed != 0U; }
void chassis_set_velocity(Chassis *chassis, float vx_mps, float vy_mps,
                          float wz_radps, uint32_t now_ms)
{
    (void)chassis; (void)vx_mps; (void)vy_mps; (void)wz_radps; (void)now_ms;
    s_chassis_busy = 1U;
}

/* ---- Servo stubs ---- */
void CarouselServo_SetAngle(float angle_deg)
{ s_carousel_deg = angle_deg; s_servo_done = 0U; }
void GripperServo_Open_Large(void)
{ s_gripper_open_count++; s_servo_done = 0U; }
bool CarouselServo_IsDone(void) { return s_servo_done != 0U; }
bool GripperServo_IsDone(void) { return s_servo_done != 0U; }

/* ---- Manipulator absolute-position jog stubs ---- */
bool ManipulatorAction_SlideJogRebaseHome(void)
{
    if (s_slide_busy) return false;
    s_slide_valid = 1U;
    s_slide_mm = 0.0f;
    s_slide_target_mm = 0.0f;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogPositionKnown(void) { return s_slide_valid != 0U; }
bool ManipulatorAction_SlideJogIsBusy(void) { return s_slide_busy != 0U; }
bool ManipulatorAction_SlideJogHasError(void) { return false; }
bool ManipulatorAction_SlideJogIsDone(void) { return s_slide_done != 0U; }
float ManipulatorAction_SlideJogGetPositionMm(void) { return s_slide_mm; }
float ManipulatorAction_SlideJogGetVisionDropMm(void)
{ return s_vision_drop_mm; }
bool ManipulatorAction_SlideJogToPositionStart(float target_mm)
{
    if (!s_slide_valid || s_slide_busy) return false;
    s_slide_target_mm = target_mm;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogToVisionPositionStart(float target_mm,
                                                      float pre_drop_mm)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm != 0.0f ||
        s_vision_drop_mm != 0.0f || target_mm <= 0.0f || pre_drop_mm <= 0.0f)
    {
        return false;
    }
    s_slide_target_mm = target_mm;
    s_vision_drop_mm = -pre_drop_mm;
    s_low_vision_start_count++;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogToVisionDropStart(float target_drop_mm)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm <= 0.0f ||
        s_vision_drop_mm > 0.0f || target_drop_mm <= 0.0f)
    {
        return false;
    }
    s_vision_drop_mm = -target_drop_mm;
    s_vision_height_jog_count++;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogRaiseGimbalOnlyStart(void)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm <= 0.0f ||
        s_vision_drop_mm >= 0.0f)
    {
        return false;
    }
    s_vision_drop_mm = 0.0f;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogRetractSlideOnlyStart(void)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm <= 0.0f ||
        s_vision_drop_mm != 0.0f)
    {
        return false;
    }
    return ManipulatorAction_SlideJogToPositionStart(0.0f);
}
bool ManipulatorAction_SlideJogHomeStart(void)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm == 0.0f ||
        s_vision_drop_mm == 0.0f)
    {
        return false;
    }
    s_vision_drop_mm = 0.0f;
    s_slide_target_mm = 0.0f;
    s_low_vision_home_count++;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
void ManipulatorAction_Update(uint32_t now_ms)
{
    (void)now_ms;
    if (s_slide_busy)
    {
        s_slide_mm = s_slide_target_mm;
        s_slide_busy = 0U;
        s_slide_done = 1U;
    }
}
void ManipulatorAction_Abort(void)
{
    s_slide_busy = 0U;
    s_slide_done = 0U;
    s_slide_valid = 0U;
}
bool ManipulatorAction_IsBusy(void) { return false; }

/* ---- K230Align orchestration stubs ---- */
void K230Align_Init(Chassis *chassis)
{
    s_k230_chassis = chassis;
    s_k230_busy = 0U;
    s_k230_done = 0U;
    s_k230_error = 0U;
}
bool K230Align_StartPreset(K230AlignMode_t mode, uint8_t target_id,
                           uint32_t now_ms)
{
    (void)now_ms;
    if (!s_k230_chassis || mode != K230_ALIGN_MODE_PLACE || target_id != 2U ||
        s_k230_busy)
    {
        return false;
    }
    s_k230_busy = 1U;
    s_k230_done = 0U;
    return true;
}
void K230Align_Update(uint32_t now_ms) { (void)now_ms; }
void K230Align_Abort(void)
{
    s_k230_busy = 0U;
    s_k230_done = 0U;
}
bool K230Align_IsBusy(void) { return s_k230_busy != 0U; }
bool K230Align_IsDone(void) { return s_k230_done != 0U; }
bool K230Align_HasError(void) { return s_k230_error != 0U; }

int main(void)
{
    Chassis chassis;

    memset(&chassis, 0, sizeof(chassis));
    chassis.config.wheel_radius_m = 0.0385f;
    chassis.config.gear_ratio = 1.0f;
    chassis.config.half_length_m = 0.15f;
    chassis.config.half_width_m = 0.10f;

    memset(&s_imu, 0, sizeof(s_imu));
    s_imu.has_angle = true;
    s_imu.last_update_ms = 100U;
    s_world_yaw_deg = 0.0f;
    s_slide_valid = 0U; /* 模拟 VOFA 接管急停后软件位置已经作废。 */

    RoughCalibration_Init(&chassis);

    /*
     * 启停区2不是路网节点 N3。本专用起点先复用正式任务的出库位移：
     * 车头朝地图上方（世界 0 度）时，车体左移 150 mm、前进 200 mm，
     * 到达 N3 后才允许规划 N3 -> N2，避免把蓝色泊位误记成 N3。
     */
    check(RoughCalibration_Start(ROUGH_CAL_START_BAY2, 100U) &&
          RoughCalibration_GetState() == ROUGH_CAL_EXIT_PARK &&
          nearf_local(s_translate_dx_m, -0.15f, 0.001f) &&
          nearf_local(s_translate_dy_m, 0.20f, 0.001f),
          "Bay 2 start first executes the calibrated parking-exit displacement");

    s_chassis_busy = 0U;
    s_position_done = 1U;
    RoughCalibration_Update(101U);
    check(RoughCalibration_GetState() == ROUGH_CAL_NAVIGATING &&
          RoughCalibration_GetCurrentNode() == 3U &&
          s_planned_target == MISSION_ROUGH_NODE,
          "Only after reaching N3 does bay 2 start plan the route to N2");

    RoughCalibration_Abort();
    s_position_done = 0U;
    s_planned_target = 0U;
    s_imu.last_update_ms = 100U;
    s_world_yaw_deg = 90.0f;

    check(RoughCalibration_Start(2U, 100U) &&
          RoughCalibration_GetState() == ROUGH_CAL_NAVIGATING &&
          s_planned_target == MISSION_ROUGH_NODE,
          "Start always plans the production rough node N2");

    RoughCalibration_Update(100U);
    check(RoughCalibration_GetState() == ROUGH_CAL_ALIGN_SETTLE,
          "Arrival enters the same stop-and-settle phase used before yaw verification");

    RoughCalibration_Update(101U); /* 建立 200 ms 稳定计时。 */
    s_imu.last_update_ms = 302U;
    RoughCalibration_Update(302U);
    check(RoughCalibration_GetState() == ROUGH_CAL_WAIT_HOME_CONFIRM,
          "A verified +90 degree task yaw releases slide calibration");

    check(!RoughCalibration_ConfirmSlideHome(0x00U) &&
          RoughCalibration_GetError() == ROUGH_CAL_ERROR_PARAM,
          "Home rebase rejects an accidental command without the A5 key");

    check(RoughCalibration_ConfirmSlideHome(0xA5U) &&
          RoughCalibration_GetState() == ROUGH_CAL_HOME_READY &&
          RoughCalibration_IsHomeConfirmed(),
          "Explicit A5 confirmation establishes the physical home as software zero");

    check(RoughCalibration_SelectRing(1U, 400U) &&
          RoughCalibration_GetState() == ROUGH_CAL_ARM_MOVING &&
          nearf_local(s_carousel_deg, ROUGH_CAROUSEL_POS1_DEG, 0.01f) &&
          s_gripper_open_count == 1U,
          "Ring 1 selection rotates the arm only while slide is at zero");

    s_servo_done = 1U;
    RoughCalibration_Update(401U);
    check(RoughCalibration_GetState() == ROUGH_CAL_ARM_READY,
          "Slide commands remain locked until carousel and gripper are ready");

    check(RoughCalibration_MoveCarouselDeciDeg(2325U, 401U) &&
          RoughCalibration_GetState() == ROUGH_CAL_ARM_MOVING &&
          nearf_local(RoughCalibration_GetCarouselTargetDeg(), 232.5f, 0.01f),
          "2325 deci-degrees commands an absolute 232.5 degree carousel angle");
    s_servo_done = 1U;
    RoughCalibration_Update(402U);
    check(RoughCalibration_GetState() == ROUGH_CAL_ARM_READY,
          "Manual carousel adjustment returns to the arm-ready state");

    check(RoughCalibration_MoveSlideDeciMm(1250U) &&
          nearf_local(RoughCalibration_GetSlideTargetMm(), 125.0f, 0.01f),
          "1250 deci-mm is decoded as an absolute 125.0 mm target");
    RoughCalibration_Update(403U);
    check(RoughCalibration_GetState() == ROUGH_CAL_SLIDE_READY &&
          nearf_local(s_slide_mm, 125.0f, 0.01f),
          "Successful slide feedback commits the absolute position");

    check(RoughCalibration_MoveSlideDeciMm(1200U),
          "A second absolute target is accepted for fine adjustment");
    RoughCalibration_Update(404U);
    check(nearf_local(s_slide_mm, 120.0f, 0.01f),
          "Fine adjustment ends at 120.0 mm, not another relative 120 mm");

    check(!RoughCalibration_MoveSlideDeciMm(1601U) &&
          RoughCalibration_GetError() == ROUGH_CAL_ERROR_PARAM,
          "The shared 160.0 mm software limit rejects 160.1 mm");

    check(RoughCalibration_RetractSlide(),
          "Retract is issued as an absolute zero target from the known position");
    RoughCalibration_Update(405U);
    check(RoughCalibration_GetState() == ROUGH_CAL_HOME_READY &&
          nearf_local(s_slide_mm, 0.0f, 0.01f),
          "Returning to zero re-enables safe carousel selection");

    check(RoughCalibration_SelectRing(3U, 500U) &&
          nearf_local(s_carousel_deg, ROUGH_CAROUSEL_POS3_DEG, 0.01f),
          "Ring 3 has its own calibrated carousel pose");

    RoughCalibration_Abort();
    check(RoughCalibration_GetState() == ROUGH_CAL_IDLE && !s_slide_valid,
          "Abort exits calibration and invalidates the slide reference");

    /*
     * 新低位流程与旧高位流程共用路线和调参命令，但第一次伸出必须只下降
     * 一次，连续绝对位置细调不能叠加下降，回零必须走“先抬升、后收回”。
     */
    s_imu.last_update_ms = 600U;
    s_world_yaw_deg = 90.0f;
    check(RoughCalibration_StartLow(2U, 600U) &&
          RoughCalibration_IsLowVision(),
          "Low rough calibration records the formal low-camera mode");
    RoughCalibration_Update(600U);
    RoughCalibration_Update(601U);
    s_imu.last_update_ms = 802U;
    RoughCalibration_Update(802U);
    check(RoughCalibration_ConfirmSlideHome(0xA5U),
          "Low mode still requires explicit slide-home confirmation");
    check(RoughCalibration_SelectRing(2U, 803U),
          "Low mode selects the ring while slide and gimbal are high/home");
    s_servo_done = 1U;
    RoughCalibration_Update(804U);

    check(RoughCalibration_MoveSlideDeciMm(900U),
          "First low-mode extension starts slide plus gimbal pre-drop");
    RoughCalibration_Update(805U);
    check(s_low_vision_start_count == 1U &&
          nearf_local(s_vision_drop_mm, -ROUGH_VISION_PRE_DROP_MM, 0.01f),
          "Low rough calibration applies the configured drop exactly once");

    check(RoughCalibration_MoveSlideDeciMm(910U),
          "Low-mode fine adjustment is accepted at the lowered height");
    RoughCalibration_Update(806U);
    check(s_low_vision_start_count == 1U &&
          nearf_local(s_slide_mm, 91.0f, 0.01f),
          "Fine adjustment does not stack a second gimbal drop");

    check(RoughCalibration_MoveGimbalDropDeciMm(1850U),
          "Verified rough-area placement height accepts direct 100-to-185 mm descent");
    RoughCalibration_Update(807U);
    check(s_vision_height_jog_count == 1U &&
          nearf_local(s_vision_drop_mm, MANIP_PICKUP_GROUND_MM, 0.01f) &&
          RoughCalibration_GetState() == ROUGH_CAL_SLIDE_READY,
          "Direct placement probe reaches the formal absolute ground height");

    check(RoughCalibration_MoveSlideDeciMm(920U),
          "Placement-height slide tuning accepts a small 1 mm step");
    RoughCalibration_Update(808U);
    check(nearf_local(s_slide_mm, 92.0f, 0.01f),
          "Low slide tuning commits the requested absolute target");
    check(RoughCalibration_MoveSlideDeciMm(1000U),
          "Placement-height slide tuning accepts a step up to 10 mm");
    RoughCalibration_Update(809U);
    check(nearf_local(s_slide_mm, 100.0f, 0.01f),
          "Expanded low slide limit commits an 8 mm adjustment");
    check(!RoughCalibration_MoveSlideDeciMm(1110U) &&
          RoughCalibration_GetError() == ROUGH_CAL_ERROR_PARAM,
          "Placement-height slide tuning still rejects a step larger than 10 mm");
    check(RoughCalibration_MoveSlideDeciMm(910U),
          "Low slide can return with a 9 mm adjustment");
    RoughCalibration_Update(810U);

    check(RoughCalibration_MoveCarouselDeciDeg(2743U, 810U),
          "Placement-height carousel tuning accepts a small 1 degree step");
    s_servo_done = 1U;
    RoughCalibration_Update(811U);
    check(RoughCalibration_MoveCarouselDeciDeg(2933U, 812U),
          "Placement-height carousel has no single-step angle-delta limit");
    s_servo_done = 1U;
    RoughCalibration_Update(813U);
    check(RoughCalibration_MoveCarouselDeciDeg(2733U, 813U),
          "Low carousel can move directly back to the K230 ring-2 angle");
    s_servo_done = 1U;
    RoughCalibration_Update(814U);

    check(RoughCalibration_MoveGimbalDropDeciMm(1000U),
          "Placement probe can return to the formal low-camera height");
    RoughCalibration_Update(809U);
    check(s_vision_height_jog_count == 2U &&
          nearf_local(s_vision_drop_mm, -ROUGH_VISION_PRE_DROP_MM, 0.01f),
          "Returning to 100 mm restores the horizontal tuning height");

    check(!RoughCalibration_MoveGimbalDropDeciMm(1900U) &&
          RoughCalibration_GetError() == ROUGH_CAL_ERROR_PARAM,
          "Placement probe rejects descent below the formal ground height");

    check(RoughCalibration_StartVisionAlign(809U) && s_k230_busy,
          "Rough ring-2 observation pose can start real chassis alignment");
    s_k230_busy = 0U;
    s_k230_done = 1U;
    RoughCalibration_Update(809U);
    check(RoughCalibration_GetState() == ROUGH_CAL_SLIDE_READY,
          "Rough chassis alignment returns control without moving the arm");

    check(RoughCalibration_RaiseGimbalToTop(),
          "Split recovery first raises only motor 5 to the top origin");
    RoughCalibration_Update(810U);
    check(s_vision_drop_mm == 0.0f &&
          nearf_local(s_slide_mm, 91.0f, 0.01f) &&
          RoughCalibration_GetState() == ROUGH_CAL_SLIDE_READY,
          "Raise-only keeps the rack extended for inspection");

    check(RoughCalibration_MoveGimbalDropDeciMm(1000U),
          "Observation command can descend again from top while rack stays out");
    RoughCalibration_Update(811U);
    check(nearf_local(s_vision_drop_mm, -ROUGH_VISION_PRE_DROP_MM, 0.01f),
          "Re-entering observation uses the absolute 100 mm target");
    check(RoughCalibration_RaiseGimbalToTop(),
          "Gimbal can be raised to top again before rack retraction");
    RoughCalibration_Update(812U);

    check(RoughCalibration_RetractSlideOnly(),
          "Split recovery then retracts only the rack");
    RoughCalibration_Update(813U);
    check(nearf_local(s_slide_mm, 0.0f, 0.01f) &&
          RoughCalibration_GetState() == ROUGH_CAL_HOME_READY,
          "Split recovery restores slide home after gimbal is already high");

    RoughCalibration_Abort();

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
