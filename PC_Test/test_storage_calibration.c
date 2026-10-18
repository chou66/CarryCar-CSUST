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
#include "servo.h"
#include "storage_calibration.h"

/*
 * 本测试只验证暂存区标定的“总流程编排”和调参互锁。EdgeMotion、航向PD、
 * K230Align、滑台点动各有自己的测试，因此这里用最小桩函数让每个子动作
 * 可控地完成，重点检查：
 *   启停区2 -> N2 -> 粗加工视觉校准 -> N4 -> 暂存区手动调参。
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
static uint8_t s_servo_done;
static float s_carousel_deg;
static uint8_t s_gripper_open_count;
static uint8_t s_planned_target;
static uint8_t s_k230_busy;
static uint8_t s_k230_done;
static uint8_t s_k230_error;
static uint8_t s_k230_start_count;
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
    if (!x || !y || corner_node != 3U) return 0U;
    *x = 2.25f;
    *y = 0.15f;
    return 1U;
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
bool ManipulatorAction_SlideJogToPositionStart(float target_mm)
{
    if (!s_slide_valid || s_slide_busy) return false;
    s_slide_target_mm = target_mm;
    s_slide_busy = 1U;
    s_slide_done = 0U;
    return true;
}
bool ManipulatorAction_SlideJogToRoughAlignStart(void)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm != 0.0f) return false;
    return ManipulatorAction_SlideJogToPositionStart(ROUGH_SLIDE_POS2_MM);
}
bool ManipulatorAction_SlideJogHomeStart(void)
{
    if (!s_slide_valid || s_slide_busy || s_slide_mm == 0.0f) return false;
    return ManipulatorAction_SlideJogToPositionStart(0.0f);
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
    s_k230_start_count++;
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
    s_slide_valid = 0U; /* 模拟0x70急停接管后软件位置已作废。 */

    StorageCalibration_Init(&chassis);

    check(!StorageCalibration_StartFromBay2(0x00U, 100U) &&
          StorageCalibration_GetError() == STORAGE_CAL_ERROR_PARAM,
          "Start rejects a missing A5 physical-home confirmation key");

    check(StorageCalibration_StartFromBay2(0xA5U, 100U) &&
          StorageCalibration_GetState() == STORAGE_CAL_EXIT_PARK &&
          nearf_local(s_translate_dx_m, -0.15f, 0.001f) &&
          nearf_local(s_translate_dy_m, 0.20f, 0.001f) &&
          s_slide_valid,
          "A5 start rebuilds slide zero and exits bay 2 to N3");

    s_chassis_busy = 0U;
    s_position_done = 1U;
    StorageCalibration_Update(101U);
    check(StorageCalibration_GetState() == STORAGE_CAL_NAV_TO_ROUGH &&
          StorageCalibration_GetCurrentNode() == 3U &&
          s_planned_target == MISSION_ROUGH_NODE,
          "After the parking exit, the first topology target is rough node N2");

    StorageCalibration_Update(102U);
    check(StorageCalibration_GetState() == STORAGE_CAL_ROUGH_ALIGN_SETTLE,
          "Arrival at N2 enters the production task-yaw settle stage");

    StorageCalibration_Update(103U); /* 建立200 ms稳定计时。 */
    s_world_yaw_deg = 90.0f;
    s_imu.last_update_ms = 304U;
    StorageCalibration_Update(304U);
    check(StorageCalibration_GetState() == STORAGE_CAL_ROUGH_PRESET &&
          nearf_local(s_slide_target_mm, ROUGH_SLIDE_POS2_MM, 0.01f) &&
          nearf_local(StorageCalibration_GetCarouselTargetDeg(),
                      K230_ALIGN_CAROUSEL_ANGLE_DEG, 0.01f),
          "N2 +90 degrees starts the calibrated 205-degree/78-mm camera pose");

    StorageCalibration_Update(305U);
    check(StorageCalibration_GetState() == STORAGE_CAL_ROUGH_VISION_ALIGN &&
          s_k230_start_count == 1U && s_k230_chassis == &chassis,
          "Preset completion starts real K230 chassis alignment on debug chassis");

    s_k230_busy = 0U;
    s_k230_done = 1U;
    StorageCalibration_Update(306U);
    check(StorageCalibration_GetState() == STORAGE_CAL_ROUGH_RETRACT &&
          nearf_local(s_slide_target_mm, 0.0f, 0.01f),
          "K230 completion retracts the camera without running rough pick/place");

    StorageCalibration_Update(307U);
    check(StorageCalibration_GetState() == STORAGE_CAL_NAV_TO_STORAGE &&
          s_planned_target == MISSION_STORAGE_NODE &&
          StorageCalibration_GetCurrentNode() == MISSION_ROUGH_NODE,
          "Corrected N2 becomes the route origin for the trip to storage N4");

    StorageCalibration_Update(308U);
    check(StorageCalibration_GetState() == STORAGE_CAL_STORAGE_ALIGN_SETTLE,
          "Arrival at N4 enters the 180-degree task-yaw settle stage");

    StorageCalibration_Update(309U); /* 建立第二次200 ms稳定计时。 */
    s_world_yaw_deg = 180.0f;
    s_imu.last_update_ms = 510U;
    StorageCalibration_Update(510U);
    check(StorageCalibration_GetState() == STORAGE_CAL_HOME_READY &&
          StorageCalibration_GetCurrentNode() == MISSION_STORAGE_NODE &&
          nearf_local(s_slide_mm, 0.0f, 0.01f),
          "N4 at 180 degrees stops with slide home and opens tuning mode");

    check(StorageCalibration_SelectRing(2U, 600U) &&
          nearf_local(s_carousel_deg, STORAGE_CAROUSEL_POS2_DEG, 0.01f) &&
          s_gripper_open_count == 1U,
          "Ring 2 selection uses the storage-specific angle at safe slide home");

    s_servo_done = 1U;
    StorageCalibration_Update(601U);
    check(StorageCalibration_GetState() == STORAGE_CAL_ARM_READY,
          "Manual slide tuning unlocks after carousel and gripper are ready");

    /* 使用正式2号位的90 mm实测值覆盖“输入->保存->视觉锁定后保持”整条链路。 */
    check(StorageCalibration_MoveSlideDeciMm(900U),
          "Storage slide accepts the calibrated 90.0-mm ring-2 target");
    StorageCalibration_Update(602U);
    check(StorageCalibration_GetState() == STORAGE_CAL_SLIDE_READY &&
          nearf_local(s_slide_mm, STORAGE_SLIDE_POS2_MM, 0.01f) &&
          StorageCalibration_AllowsManualVision(),
          "At the requested length K230 observation is explicitly allowed");

    check(StorageCalibration_StartStorageVisionAlign(603U) &&
          StorageCalibration_GetState() == STORAGE_CAL_STORAGE_VISION_ALIGN &&
          s_k230_start_count == 2U,
          "N4 ring-2 lock starts K230 without changing the tuned slide pose");

    /* 第一轮：K230平移没有带来航向漂移，应在复核后直接回到SLIDE_READY。 */
    s_k230_busy = 0U;
    s_k230_done = 1U;
    StorageCalibration_Update(604U);
    check(StorageCalibration_GetState() == STORAGE_CAL_STORAGE_ALIGN_SETTLE,
          "K230 completion always enters a fresh 180-degree yaw verification");
    StorageCalibration_Update(605U);
    s_imu.last_update_ms = 806U;
    StorageCalibration_Update(806U);
    check(StorageCalibration_GetState() == STORAGE_CAL_SLIDE_READY &&
          nearf_local(s_slide_mm, STORAGE_SLIDE_POS2_MM, 0.01f),
          "Pose lock preserves the slide target when yaw is already correct");

    /*
     * 第二轮：模拟K230平移后航向漂到175 deg。PD真的转过以后必须自动再跑
     * 一次K230；第二次图像和航向都合格才允许结束联合锁定。
     */
    check(StorageCalibration_StartStorageVisionAlign(900U),
          "A second pose lock can run without returning to bay 2");
    s_k230_busy = 0U;
    s_k230_done = 1U;
    s_world_yaw_deg = 175.0f;
    s_imu.last_update_ms = 901U;
    StorageCalibration_Update(901U);
    StorageCalibration_Update(902U);
    s_imu.last_update_ms = 1103U;
    StorageCalibration_Update(1103U);
    check(StorageCalibration_GetState() == STORAGE_CAL_STORAGE_ALIGNING,
          "A yaw error after K230 starts the existing production PD controller");

    s_world_yaw_deg = 180.0f;
    s_imu.last_update_ms = 1104U;
    StorageCalibration_Update(1104U);
    StorageCalibration_Update(1105U);
    s_imu.last_update_ms = 1306U;
    StorageCalibration_Update(1306U);
    check(StorageCalibration_GetState() == STORAGE_CAL_STORAGE_VISION_ALIGN &&
          s_k230_start_count == 4U,
          "Any real yaw correction automatically triggers another K230 verification");

    s_k230_busy = 0U;
    s_k230_done = 1U;
    StorageCalibration_Update(1307U);
    StorageCalibration_Update(1308U);
    s_imu.last_update_ms = 1509U;
    StorageCalibration_Update(1509U);
    check(StorageCalibration_GetState() == STORAGE_CAL_SLIDE_READY,
          "The combined lock ends only when image and yaw are both valid");

    check(StorageCalibration_MoveCarouselDeciDeg(1780U, 1510U) &&
          nearf_local(StorageCalibration_GetCarouselTargetDeg(), 178.0f, 0.01f),
          "Carousel angle can be fine-tuned while the slide is extended");
    s_servo_done = 1U;
    StorageCalibration_Update(1511U);

    check(!StorageCalibration_MoveSlideDeciMm(1601U) &&
          StorageCalibration_GetError() == STORAGE_CAL_ERROR_PARAM,
          "The shared 160.0-mm software limit rejects 160.1 mm");

    check(StorageCalibration_RetractSlide(),
          "Retract is an absolute return to software zero");
    StorageCalibration_Update(1512U);
    check(StorageCalibration_GetState() == STORAGE_CAL_HOME_READY &&
          nearf_local(s_slide_mm, 0.0f, 0.01f),
          "Returning to zero safely re-enables selection of another ring");

    StorageCalibration_Abort();
    check(StorageCalibration_GetState() == STORAGE_CAL_IDLE && !s_slide_valid,
          "Abort exits storage calibration and invalidates the slide reference");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
