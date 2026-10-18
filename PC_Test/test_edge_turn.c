#include <math.h>
#include <stdio.h>
#include "edge_turn.h"
#include "map_graph.h"

static int s_pass;
static int s_fail;

static int nearf(float a, float b, float tolerance)
{
    float d = a - b;
    if (d < 0.0f) d = -d;
    return d <= tolerance;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

int main(void)
{
    EdgeTurnPlan_t plan;
    EdgeTurnOptions_t options;

    MapGraph_Build(0U);
    options.allow_reverse = 1U;
    options.reverse_min_turn_saving_deg = 5.0f;
    options.arrival_yaw_valid = 0U;
    options.arrival_world_yaw_deg = 0.0f;

    check(EdgeTurn_Plan(5U, 8U, 0.4f, 0.8f, &options, &plan) &&
          nearf(plan.target_world_yaw_deg, 0.0f, 0.01f) &&
          nearf(plan.distance_m, 0.90f, 0.01f) &&
          !plan.drive_backward && nearf(plan.body_dy_m, 0.90f, 0.01f) &&
          !plan.turn_required,
          "N5->N8 already faces forward");

    check(EdgeTurn_Plan(5U, 6U, 0.0f, 0.8f, &options, &plan) &&
          nearf(plan.target_world_yaw_deg, 90.0f, 0.01f) &&
          nearf(plan.world_yaw_error_deg, 90.0f, 0.01f) &&
          nearf(plan.chassis_turn_deg, -90.0f, 0.01f) &&
          plan.turn_required,
          "N5->N6 requests a clockwise quarter-turn");

    check(EdgeTurn_Plan(5U, 4U, 0.0f, 0.8f, &options, &plan) &&
          nearf(plan.target_world_yaw_deg, -90.0f, 0.01f) &&
          nearf(plan.chassis_turn_deg, 90.0f, 0.01f) &&
          plan.turn_required,
          "N5->N4 requests a counter-clockwise quarter-turn");

    check(EdgeTurn_Plan(5U, 2U, -179.4f, 0.8f, &options, &plan) &&
          nearf(plan.target_world_yaw_deg, 180.0f, 0.01f) &&
          nearf(plan.world_yaw_error_deg, -0.6f, 0.02f) &&
          !plan.turn_required,
          "Heading wrap avoids an unnecessary full turn");

    check(EdgeTurn_Plan(5U, 8U, 179.6f, 0.8f, &options, &plan) &&
          plan.drive_backward &&
          nearf(plan.target_world_yaw_deg, 180.0f, 0.01f) &&
          nearf(plan.body_dy_m, -0.90f, 0.01f) &&
          !plan.turn_required,
          "Node behind the car is reached by reversing without a 180-degree turn");

    /* 到站后还要转到0°作业时，不能只贪图当前少转而选180°后退。 */
    options.arrival_yaw_valid = 1U;
    options.arrival_world_yaw_deg = 0.0f;
    check(EdgeTurn_Plan(5U, 8U, 100.0f, 0.8f, &options, &plan) &&
          !plan.drive_backward &&
          nearf(plan.target_world_yaw_deg, 0.0f, 0.01f),
          "Final task yaw is included in forward/reverse cost");

    /* 当前转角同为90°时，选择能直接满足任务航向的后退姿态。 */
    options.arrival_world_yaw_deg = 180.0f;
    check(EdgeTurn_Plan(1U, 4U, 90.0f, 0.8f, &options, &plan) &&
          plan.drive_backward &&
          nearf(plan.target_world_yaw_deg, 180.0f, 0.01f) &&
          nearf(plan.body_dy_m, -0.85f, 0.01f),
          "Equal immediate turns use final task yaw as the tie breaker");

    options.arrival_yaw_valid = 0U;
    check(!EdgeTurn_Plan(1U, 5U, 0.0f, 0.8f, &options, &plan),
          "Reject non-adjacent nodes");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
