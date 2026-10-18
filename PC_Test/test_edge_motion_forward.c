#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_uart.h"
#include "chassis_motion.h"
#include "edge_motion.h"
#include "hwt101_uart.h"
#include "navigation.h"

static Hwt101 s_imu;
static uint8_t s_position_done;
static uint8_t s_position_failed;
static uint8_t s_motion_busy;
static unsigned s_rotate_count;
static unsigned s_translate_count;
static unsigned s_velocity_count;
static float s_last_rotate_deg;
static float s_last_dx_m;
static float s_last_dy_m;
static float s_last_wz_radps;
static int s_pass;
static int s_fail;

static int nearf(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static void reset_stubs(void)
{
    memset(&s_imu, 0, sizeof(s_imu));
    s_imu.has_angle = true;
    s_imu.world_zero_ready = true;
    s_position_done = 0U;
    s_position_failed = 0U;
    s_motion_busy = 0U;
    s_rotate_count = 0U;
    s_translate_count = 0U;
    s_velocity_count = 0U;
    s_last_rotate_deg = 0.0f;
    s_last_dx_m = 0.0f;
    s_last_dy_m = 0.0f;
    s_last_wz_radps = 0.0f;
}

static void set_yaw(float yaw_deg, uint32_t now_ms)
{
    s_imu.yaw_deg = yaw_deg;
    s_imu.last_update_ms = now_ms;
}

/* ---- AppUart / HWT101 stubs ---- */
Hwt101 *AppUart_GetImu(void) { return &s_imu; }
float hwt101_world_yaw_deg(const Hwt101 *imu) { return imu->yaw_deg; }
bool hwt101_world_zero_ready(const Hwt101 *imu)
{ return imu && imu->world_zero_ready; }
bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms)
{
    return imu && imu->has_angle &&
           (uint32_t)(now_ms - imu->last_update_ms) <= timeout_ms;
}

/* ---- HMI-Navigation bridge stubs ---- */
uint8_t HMI_Navigation_IsInitialized(const HMI_Navigation_t *ctx)
{ return ctx ? ctx->initialized : 0U; }
uint8_t HMI_Navigation_GetCurrentEdge(const HMI_Navigation_t *ctx,
                                      uint8_t *from_node, uint8_t *to_node)
{ return ctx && Navigation_GetCurrentEdge(&ctx->nav, from_node, to_node); }
uint8_t HMI_Navigation_EdgeReached(HMI_Navigation_t *ctx)
{ return ctx && Navigation_EdgeReached(&ctx->nav); }
void HMI_Navigation_UpdateDebugDisplay(const HMI_Navigation_t *ctx)
{ (void)ctx; }

/* ---- Chassis motion stubs ---- */
void chassis_motion_rotate_deg(Chassis *chassis, float angle_deg,
                               float speed_mps, uint32_t now_ms)
{
    (void)chassis; (void)speed_mps; (void)now_ms;
    s_rotate_count++;
    s_last_rotate_deg = angle_deg;
    s_position_done = 0U;
    s_motion_busy = 1U;
}

void chassis_motion_translate(Chassis *chassis, float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms)
{
    (void)chassis; (void)speed_mps; (void)now_ms;
    s_translate_count++;
    s_last_dx_m = dx_m;
    s_last_dy_m = dy_m;
    s_position_done = 0U;
    s_motion_busy = 1U;
}

bool chassis_motion_done(const Chassis *chassis)
{ (void)chassis; return s_position_done != 0U; }
bool chassis_motion_failed(const Chassis *chassis)
{ (void)chassis; return s_position_failed != 0U; }
bool chassis_motion_busy(const Chassis *chassis)
{ (void)chassis; return s_motion_busy != 0U; }
void chassis_motion_stop(Chassis *chassis)
{ (void)chassis; s_motion_busy = 0U; }
void chassis_set_velocity(Chassis *chassis, float vx_mps, float vy_mps,
                          float wz_radps, uint32_t now_ms)
{
    (void)chassis; (void)vx_mps; (void)vy_mps; (void)now_ms;
    s_velocity_count++;
    s_last_wz_radps = wz_radps;
    s_motion_busy = 1U;
}

static void finish_position(void)
{
    s_position_done = 1U;
    s_motion_busy = 0U;
}

static void test_turn_then_forward(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 5U, 0U);
    Navigation_PlanTo(&hmi.nav, 6U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    set_yaw(0.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TURNING_TO_EDGE &&
          s_rotate_count == 1U && s_translate_count == 0U &&
          nearf(s_last_rotate_deg, -90.0f, 0.01f),
          "N5->N6 turns clockwise before moving");

    finish_position();
    EdgeMotion_Update(&edge, 1U);
    check(edge.state == EDGE_MOTION_SETTLE_AFTER_TURN,
          "Turn completion enters settle state");

    set_yaw(90.0f, 250U);
    EdgeMotion_Update(&edge, 250U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          s_translate_count == 1U &&
          nearf(s_last_dx_m, 0.0f, 0.0001f) &&
          nearf(s_last_dy_m, 0.90f, 0.01f),
          "After turn, topology edge is pure forward motion");

    finish_position();
    EdgeMotion_Update(&edge, 251U);
    set_yaw(90.0f, 450U);
    EdgeMotion_Update(&edge, 450U);
    check(Navigation_IsDone(&hmi.nav) &&
          Navigation_GetCurrentNode(&hmi.nav) == 6U &&
          edge.state == EDGE_MOTION_IDLE,
          "Node advances only after forward motion and yaw check");
}

static void test_already_aligned_goes_forward(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 5U, 0U);
    Navigation_PlanTo(&hmi.nav, 8U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    set_yaw(0.4f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          s_rotate_count == 0U && s_translate_count == 1U &&
          nearf(s_last_dx_m, 0.0f, 0.0001f) &&
          nearf(s_last_dy_m, 0.90f, 0.01f),
          "Aligned edge skips turn but still commands pure forward");
}

static void test_node_behind_uses_reverse_without_full_turn(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 5U, 0U);
    Navigation_PlanTo(&hmi.nav, 8U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    /* N8在N5正北；车头朝南(180°)时，直接后退比原地转180°再前进更合理。 */
    set_yaw(180.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.drive_backward == 1U &&
          s_rotate_count == 0U && s_translate_count == 1U &&
          nearf(s_last_dx_m, 0.0f, 0.0001f) &&
          nearf(s_last_dy_m, -0.90f, 0.01f),
          "Node directly behind uses pure reverse without a 180-degree turn");
}

static void test_large_error_is_corrected_and_rechecked(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    chassis.config.wheel_radius_m = 0.0385f;
    chassis.config.half_length_m = 0.10f;
    chassis.config.half_width_m = 0.10f;
    chassis.config.gear_ratio = 1.0f;
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 5U, 0U);
    Navigation_PlanTo(&hmi.nav, 8U); /* target edge yaw = 0 deg */
    EdgeMotion_Init(&edge, &chassis, &hmi);

    set_yaw(0.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    finish_position();
    EdgeMotion_Update(&edge, 1U);

    /* The old implementation stopped with EDGE_MOTION_ERROR above 8 deg. */
    set_yaw(30.0f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_YAW_CORRECTING &&
          s_velocity_count == 1U && s_last_wz_radps > 0.0f,
          "30-degree edge-end error starts fast CCW correction instead of failing");

    /* Enter deadband, stop, then simulate a 1-degree rebound during settle. */
    set_yaw(-0.2f, 240U);
    EdgeMotion_Update(&edge, 240U);
    EdgeMotion_Update(&edge, 241U);
    set_yaw(1.0f, 400U);
    EdgeMotion_Update(&edge, 400U);
    check(edge.state == EDGE_MOTION_YAW_CORRECTING &&
          !Navigation_IsDone(&hmi.nav) && s_velocity_count >= 2U,
          "Post-stop rebound is rechecked and corrected before node commit");

    set_yaw(0.2f, 440U);
    EdgeMotion_Update(&edge, 440U);
    EdgeMotion_Update(&edge, 441U);
    set_yaw(0.2f, 600U);
    EdgeMotion_Update(&edge, 600U);
    check(edge.state == EDGE_MOTION_IDLE && Navigation_IsDone(&hmi.nav),
          "Node commits only after settled yaw is inside deadband");
}

/*
 * 中间节点后紧接着要转 90 度时，边末不再先纠到当前边航向：
 * 直接提交节点，下一次 Update 由 edge_turn 一次转到下一条边的目标角。
 *
 * 路径 1->2->5->6：第一段 1->2 朝东(90 度)，第二段 2->5 朝北(0 度)。
 */
static void test_following_turn_skips_redundant_edge_correction(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 1U, 0U);
    Navigation_PlanTo(&hmi.nav, 6U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    set_yaw(90.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    finish_position();
    EdgeMotion_Update(&edge, 1U);

    /* 0.6 度超过边末死区，但下一条边需要转 90 度。 */
    set_yaw(90.6f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_GetCurrentNode(&hmi.nav) == 2U &&
          !Navigation_IsDone(&hmi.nav) &&
          s_velocity_count == 0U,
          "Intermediate node commits without a redundant edge-end correction");

    EdgeMotion_Update(&edge, 201U);
    check(edge.state == EDGE_MOTION_TURNING_TO_EDGE &&
          s_rotate_count == 1U &&
          nearf(s_last_rotate_deg, 90.6f, 0.05f),
          "Next edge turns directly from the drifted angle to its target heading");
}

/*
 * 下一条边是直行通过时不能跳纠偏，否则 0.5~0.8 度的小误差会沿路径累积。
 * 路径 1->2->3 两段都朝东。
 */
static void test_straight_through_node_still_corrects(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    chassis.config.wheel_radius_m = 0.0385f;
    chassis.config.half_length_m = 0.10f;
    chassis.config.half_width_m = 0.10f;
    chassis.config.gear_ratio = 1.0f;
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 1U, 0U);
    Navigation_PlanTo(&hmi.nav, 3U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    set_yaw(90.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    finish_position();
    EdgeMotion_Update(&edge, 1U);

    set_yaw(90.6f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_YAW_CORRECTING &&
          Navigation_GetCurrentNode(&hmi.nav) == 1U &&
          s_velocity_count == 1U,
          "Straight-through node keeps the edge-end correction");
}

/*
 * 最终任务点由 mission 的 task_yaw PD 接管；声明后边末直接提交，
 * 即使当前边方向与任务航向相差很大，也交给任务 PD 一次转到位。
 */
static void test_final_task_node_defers_to_task_alignment(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 5U, 0U);
    Navigation_PlanTo(&hmi.nav, 6U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    EdgeMotion_BeginRoute(&edge);
    check(edge.arrival_align_valid == 0U,
          "BeginRoute clears a stale arrival-align declaration");
    EdgeMotion_SetArrivalAlign(&edge, 1U);

    set_yaw(90.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    finish_position();
    EdgeMotion_Update(&edge, 1U);

    set_yaw(90.6f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_GetCurrentNode(&hmi.nav) == 6U &&
          Navigation_IsDone(&hmi.nav) &&
          s_velocity_count == 0U,
          "Final task node defers heading work to task-yaw alignment");
}

int main(void)
{
    test_turn_then_forward();
    test_already_aligned_goes_forward();
    test_node_behind_uses_reverse_without_full_turn();
    test_large_error_is_corrected_and_rechecked();
    test_following_turn_skips_redundant_edge_correction();
    test_straight_through_node_still_corrects();
    test_final_task_node_defers_to_task_alignment();
    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
