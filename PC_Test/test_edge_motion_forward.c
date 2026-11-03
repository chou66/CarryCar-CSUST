#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_uart.h"
#include "chassis_motion.h"
#include "edge_motion.h"
#include "hwt101_uart.h"
#include "navigation.h"
#include "robot_config.h"

static Hwt101 s_imu;
static uint8_t s_position_done;
static uint8_t s_position_failed;
static uint8_t s_motion_busy;
static unsigned s_rotate_count;
static unsigned s_translate_count;
static unsigned s_velocity_count;
static unsigned s_arc_count;
static float s_last_rotate_deg;
static float s_last_dx_m;
static float s_last_dy_m;
static float s_last_world_dx_m;
static float s_last_world_dy_m;
static float s_last_target_yaw_deg;
static float s_last_arc_start_yaw_deg;
static float s_last_arc_turn_deg;
static float s_last_arc_radius_m;
static uint8_t s_last_arc_reverse;
static uint8_t s_last_ramp_in;
static uint8_t s_last_ramp_out;
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
    s_last_world_dx_m = 0.0f;
    s_last_world_dy_m = 0.0f;
    s_last_target_yaw_deg = 0.0f;
    s_arc_count = 0U;
    s_last_arc_start_yaw_deg = 0.0f;
    s_last_arc_turn_deg = 0.0f;
    s_last_arc_radius_m = 0.0f;
    s_last_arc_reverse = 0U;
    s_last_ramp_in = 1U;
    s_last_ramp_out = 1U;
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

void chassis_motion_drift_translate(Chassis *chassis,
                                    float world_dx_m, float world_dy_m,
                                    float distance_m,
                                    float target_world_yaw_deg,
                                    float speed_mps, uint32_t now_ms)
{
    chassis_motion_drift_segment(chassis, world_dx_m, world_dy_m, distance_m,
                                 target_world_yaw_deg, speed_mps, 1U, 1U,
                                 now_ms);
}

void chassis_motion_drift_segment(Chassis *chassis,
                                  float world_dx_m, float world_dy_m,
                                  float distance_m,
                                  float target_world_yaw_deg,
                                  float speed_mps,
                                  uint8_t ramp_in, uint8_t ramp_out,
                                  uint32_t now_ms)
{
    (void)chassis; (void)speed_mps; (void)now_ms;
    /* 边是纯车身Y运动，桩按距离记录 dy，dx 恒为 0；世界向量单独记录。 */
    s_translate_count++;
    s_last_dx_m = 0.0f;
    s_last_dy_m = distance_m;
    s_last_world_dx_m = world_dx_m;
    s_last_world_dy_m = world_dy_m;
    s_last_target_yaw_deg = target_world_yaw_deg;
    s_last_ramp_in = ramp_in;
    s_last_ramp_out = ramp_out;
    s_position_done = 0U;
    s_motion_busy = 1U;
}

void chassis_motion_arc_world_ex(Chassis *chassis,
                                 float start_tangent_world_yaw_deg,
                                 float turn_deg,
                                 float radius_m,
                                 float speed_mps,
                                 uint8_t reverse,
                                 uint32_t now_ms)
{
    (void)chassis; (void)speed_mps; (void)now_ms;
    s_arc_count++;
    s_last_arc_start_yaw_deg = start_tangent_world_yaw_deg;
    s_last_arc_turn_deg = turn_deg;
    s_last_arc_radius_m = radius_m;
    s_last_arc_reverse = reverse;
    s_position_done = 0U;
    s_motion_busy = 1U;
}

void chassis_motion_arc_world(Chassis *chassis,
                              float start_world_yaw_deg,
                              float turn_deg,
                              float radius_m,
                              float speed_mps,
                              uint32_t now_ms)
{
    chassis_motion_arc_world_ex(chassis, start_world_yaw_deg, turn_deg,
                                radius_m, speed_mps, 0U, now_ms);
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

    /*
     * 车头朝北(0°)，N5->N6 朝东(90°)：起点航向不对、且终点就是目标节点，
     * 不能做圆滑拐角，仍按原来的“原地转到边方向再走”。
     */
    set_yaw(0.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TURNING_TO_EDGE &&
          s_rotate_count == 1U && s_translate_count == 0U &&
          nearf(s_last_rotate_deg, -90.0f, 0.01f),
          "N5->N6 turns in place when the target node itself needs the turn");

    finish_position();
    EdgeMotion_Update(&edge, 1U);
    check(edge.state == EDGE_MOTION_SETTLE_AFTER_TURN,
          "Turn completion enters settle state");

    set_yaw(90.0f, 250U);
    EdgeMotion_Update(&edge, 250U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          s_translate_count == 1U &&
          nearf(s_last_dx_m, 0.0f, 0.0001f) &&
          nearf(s_last_dy_m, 0.90f, 0.01f) &&
          s_last_ramp_in == 1U && s_last_ramp_out == 1U,
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
 * 圆滑拐角：1->6 的路线是 1->4->5->6，第一段 1->4 朝北(0°)，第二段
 * 4->5->6 朝东(90°)。
 *
 * 领 1->4 时预读到 N4 要转 90°，于是直线末段不再减速，按半径
 * ROUTE_CORNER_ARC_RADIUS_M 走一段圆弧转过拐角，弧后保持满速直接接上
 * 4->6 的直线——N4 不停车、不做原地转。
 */
static void test_corner_arc_blends_intermediate_node(void)
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

    set_yaw(0.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          s_rotate_count == 0U && s_translate_count == 1U &&
          s_last_ramp_in == 1U && s_last_ramp_out == 0U &&
          nearf(s_last_world_dy_m, 0.85f, 0.01f) &&
          nearf(s_last_dy_m,
                0.85f - ROUTE_CORNER_ARC_RADIUS_M, 0.01f),
          "Straight keeps full speed into the corner arc");

    finish_position();
    EdgeMotion_Update(&edge, 1U);
    check(edge.state == EDGE_MOTION_CORNER_ARC &&
          s_arc_count == 1U && s_translate_count == 1U &&
          nearf(s_last_arc_start_yaw_deg, 0.0f, 0.01f) &&
          nearf(s_last_arc_turn_deg, 90.0f, 0.01f) &&
          nearf(s_last_arc_radius_m, ROUTE_CORNER_ARC_RADIUS_M, 0.0001f),
          "Corner is passed as a 90-degree arc of the configured radius");

    finish_position();
    EdgeMotion_Update(&edge, 2U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          s_rotate_count == 0U && s_translate_count == 2U &&
          s_last_ramp_in == 0U && s_last_ramp_out == 1U &&
          nearf(s_last_world_dx_m, 1.75f, 0.01f) &&
          nearf(s_last_world_dy_m, 0.0f, 0.01f) &&
          nearf(s_last_target_yaw_deg, 90.0f, 0.01f) &&
          nearf(s_last_dy_m,
                1.75f - ROUTE_CORNER_ARC_RADIUS_M, 0.01f),
          "Arc exits straight into the next edge without stopping");

    finish_position();
    EdgeMotion_Update(&edge, 3U);
    set_yaw(90.0f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_IsDone(&hmi.nav) &&
          Navigation_GetCurrentNode(&hmi.nav) == 6U &&
          s_velocity_count == 0U,
          "Corner run commits every crossed node after the final straight");
}

/*
 * 同一条车道上的连续边（1->2->3 都朝东）在领取时合并成一次直线平移：
 * 中途的 N2 不停车、不做边末航向校准，只在最终目标 N3 停车提交。
 */
static void test_straight_run_is_merged_into_one_translate(void)
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
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.run_steps == 2U &&
          edge.to_node == 3U &&
          s_translate_count == 1U &&
          nearf(s_last_dy_m, 1.75f, 0.01f),
          "Collinear edges merge into one 1.75 m straight translate");

    finish_position();
    EdgeMotion_Update(&edge, 1U);

    set_yaw(90.2f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_IsDone(&hmi.nav) &&
          Navigation_GetCurrentNode(&hmi.nav) == 3U &&
          s_velocity_count == 0U,
          "Merged run advances every crossed node and stops only at the target");
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

/*
 * 倒车过弯：N2->N4 的路线是 2->1->4。车在 N2 完成粗加工后车头朝东(90°)，
 * 第一段 2->1 朝西，edge_turn 选择倒车（不转 180°）。倒车直线末段同样
 * 保持满速，用倒车圆弧转过 N1，再继续倒车到 N4——N1 不停车。
 */
static void test_corner_arc_blends_backward_run(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 2U, 0U);
    Navigation_PlanTo(&hmi.nav, 4U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    /* N2 车头朝东(90°)，2->1 朝西 => 倒车，且预读 N1 的 90° 弯。 */
    set_yaw(90.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.drive_backward == 1U && edge.corner_valid == 1U &&
          edge.corner_reverse == 1U &&
          s_rotate_count == 0U && s_translate_count == 1U &&
          s_last_ramp_in == 1U && s_last_ramp_out == 0U &&
          nearf(s_last_world_dx_m, -0.85f, 0.01f) &&
          nearf(s_last_world_dy_m, 0.0f, 0.01f) &&
          nearf(s_last_target_yaw_deg, 90.0f, 0.01f) &&
          nearf(s_last_dy_m, -(0.85f - ROUTE_CORNER_ARC_RADIUS_M), 0.01f),
          "Backward straight keeps full speed into the corner arc");

    finish_position();
    EdgeMotion_Update(&edge, 1U);
    check(edge.state == EDGE_MOTION_CORNER_ARC &&
          s_arc_count == 1U && s_translate_count == 1U &&
          s_last_arc_reverse == 1U &&
          nearf(s_last_arc_start_yaw_deg, -90.0f, 0.01f) &&
          nearf(s_last_arc_turn_deg, 90.0f, 0.01f),
          "Backward corner is passed as a reverse arc");

    finish_position();
    EdgeMotion_Update(&edge, 2U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.to_node == 4U && edge.drive_backward == 1U &&
          s_rotate_count == 0U && s_translate_count == 2U &&
          s_last_ramp_in == 0U && s_last_ramp_out == 1U &&
          nearf(s_last_world_dx_m, 0.0f, 0.01f) &&
          nearf(s_last_world_dy_m, 0.85f, 0.01f) &&
          nearf(s_last_target_yaw_deg, 180.0f, 0.01f) &&
          nearf(s_last_dy_m, -(0.85f - ROUTE_CORNER_ARC_RADIUS_M), 0.01f),
          "Reverse arc exits into a backward straight without stopping");

    finish_position();
    EdgeMotion_Update(&edge, 3U);
    set_yaw(180.0f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_IsDone(&hmi.nav) &&
          Navigation_GetCurrentNode(&hmi.nav) == 4U &&
          s_velocity_count == 0U,
          "Backward corner run commits the crossed node without a pivot");
}

/*
 * 暂存区 N4 -> 原料区 N8 走偏好的 4->7->8（与 4->5->8 等长）。
 * N4 完成暂存任务后车头朝下(180°)，4->7 朝北、7->8 朝东，两段都倒车；
 * N7 用倒车圆弧 90° 通过，全程不停车、不原地转。
 */
static void test_storage_to_raw_route_uses_n7_arc(void)
{
    HMI_Navigation_t hmi;
    Chassis chassis;
    EdgeMotion_t edge;

    memset(&hmi, 0, sizeof(hmi));
    memset(&chassis, 0, sizeof(chassis));
    reset_stubs();
    hmi.initialized = 1U;
    Navigation_Init(&hmi.nav, 4U, 0U);
    Navigation_PlanTo(&hmi.nav, 8U);
    EdgeMotion_Init(&edge, &chassis, &hmi);

    /* N4 车头朝下(180°)，4->7 朝北 => 倒车，并预读 N7 的 90° 弯。 */
    set_yaw(180.0f, 0U);
    EdgeMotion_Update(&edge, 0U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.to_node == 7U && edge.drive_backward == 1U &&
          edge.corner_valid == 1U && edge.corner_reverse == 1U &&
          s_rotate_count == 0U && s_translate_count == 1U &&
          s_last_ramp_in == 1U && s_last_ramp_out == 0U &&
          nearf(s_last_world_dx_m, 0.0f, 0.01f) &&
          nearf(s_last_world_dy_m, 0.90f, 0.01f) &&
          nearf(s_last_target_yaw_deg, 180.0f, 0.01f) &&
          nearf(s_last_dy_m, -(0.90f - ROUTE_CORNER_ARC_RADIUS_M), 0.01f),
          "Storage->raw route starts with a backward 4->7 straight");

    finish_position();
    EdgeMotion_Update(&edge, 1U);
    check(edge.state == EDGE_MOTION_CORNER_ARC &&
          s_arc_count == 1U && s_translate_count == 1U &&
          s_last_arc_reverse == 1U &&
          nearf(s_last_arc_start_yaw_deg, 0.0f, 0.01f) &&
          nearf(s_last_arc_turn_deg, 90.0f, 0.01f),
          "N7 is passed as a 90-degree reverse arc");

    finish_position();
    EdgeMotion_Update(&edge, 2U);
    check(edge.state == EDGE_MOTION_TRANSLATING &&
          edge.to_node == 8U && edge.drive_backward == 1U &&
          s_rotate_count == 0U && s_translate_count == 2U &&
          s_last_ramp_in == 0U && s_last_ramp_out == 1U &&
          nearf(s_last_world_dx_m, 0.85f, 0.01f) &&
          nearf(s_last_world_dy_m, 0.0f, 0.01f) &&
          /* 7->8 朝东，倒车时车头朝西：偏航角按 [-180,180] 归一为 -90。 */
          nearf(s_last_target_yaw_deg, -90.0f, 0.01f) &&
          nearf(s_last_dy_m, -(0.85f - ROUTE_CORNER_ARC_RADIUS_M), 0.01f),
          "Arc exits straight into 7->8 backward without stopping");

    finish_position();
    EdgeMotion_Update(&edge, 3U);
    set_yaw(270.0f, 200U);
    EdgeMotion_Update(&edge, 200U);
    check(edge.state == EDGE_MOTION_IDLE &&
          Navigation_IsDone(&hmi.nav) &&
          Navigation_GetCurrentNode(&hmi.nav) == 8U &&
          s_velocity_count == 0U,
          "4->7->8 arrives at N8 aligned with -90 without a pivot");
}

int main(void)
{
    test_turn_then_forward();
    test_already_aligned_goes_forward();
    test_node_behind_uses_reverse_without_full_turn();
    test_large_error_is_corrected_and_rechecked();
    test_corner_arc_blends_intermediate_node();
    test_corner_arc_blends_backward_run();
    test_storage_to_raw_route_uses_n7_arc();
    test_straight_run_is_merged_into_one_translate();
    test_final_task_node_defers_to_task_alignment();
    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
