#include "edge_turn.h"
#include "map_graph.h"
#include <math.h>

#define EDGE_TURN_RAD_TO_DEG_F 57.2957795130823208768f
#define EDGE_TURN_MIN_DISTANCE_M 0.001f

static float edge_turn_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float EdgeTurn_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

uint8_t EdgeTurn_Plan(uint8_t from_node,
                      uint8_t to_node,
                      float current_world_yaw_deg,
                      float tolerance_deg,
                      const EdgeTurnOptions_t *options,
                      EdgeTurnPlan_t *plan)
{
    float forward_yaw_deg;
    float backward_yaw_deg;
    float forward_error_deg;
    float backward_error_deg;
    float forward_cost_deg;
    float backward_cost_deg;
    float reverse_saving_deg;
    uint8_t choose_backward = 0U;

    if (!plan || tolerance_deg < 0.0f ||
        from_node < 1U || from_node > NODE_NUM ||
        to_node < 1U || to_node > NODE_NUM ||
        !MapGraph_IsConnected(from_node, to_node))
        return 0U;

    plan->world_dx_m = g_nodes[to_node].x - g_nodes[from_node].x;
    plan->world_dy_m = g_nodes[to_node].y - g_nodes[from_node].y;
    plan->distance_m = sqrtf(plan->world_dx_m * plan->world_dx_m +
                             plan->world_dy_m * plan->world_dy_m);

    if (plan->distance_m < EDGE_TURN_MIN_DISTANCE_M)
        return 0U;

    /* atan2(dx, dy)直接得到“0=+Y、顺时针为正”的边几何方向。 */
    forward_yaw_deg =
        atan2f(plan->world_dx_m, plan->world_dy_m) *
        EDGE_TURN_RAD_TO_DEG_F;

    /* 后退时车头应朝向边方向的反方向，车身-Y才会指向to_node。 */
    backward_yaw_deg = EdgeTurn_WrapDeg(forward_yaw_deg + 180.0f);
    forward_error_deg = EdgeTurn_WrapDeg(forward_yaw_deg -
                                         current_world_yaw_deg);
    backward_error_deg = EdgeTurn_WrapDeg(backward_yaw_deg -
                                          current_world_yaw_deg);
    forward_cost_deg = edge_turn_absf(forward_error_deg);
    backward_cost_deg = edge_turn_absf(backward_error_deg);

    /*
     * 若to_node就是本次导航最终任务点，把到站后的任务航向转角也算入总代价。
     * 任务航向无效时只比较当前这一转，适用于普通中间节点和最终返回节点。
     */
    if (options && options->arrival_yaw_valid)
    {
        forward_cost_deg += edge_turn_absf(
            EdgeTurn_WrapDeg(options->arrival_world_yaw_deg - forward_yaw_deg));
        backward_cost_deg += edge_turn_absf(
            EdgeTurn_WrapDeg(options->arrival_world_yaw_deg - backward_yaw_deg));
    }

    reverse_saving_deg = options ? options->reverse_min_turn_saving_deg : 0.0f;
    if (reverse_saving_deg < 0.0f)
        reverse_saving_deg = 0.0f;

    /*
     * 必须“明确少转”才后退；代价相同或只差很小角度时固定选前进。
     * 因此90°附近不会因为IMU抖动而随机切换前进/后退。
     */
    if (options && options->allow_reverse &&
        backward_cost_deg + reverse_saving_deg < forward_cost_deg)
        choose_backward = 1U;

    plan->edge_world_yaw_deg = forward_yaw_deg;
    plan->drive_backward = choose_backward;
    plan->target_world_yaw_deg = choose_backward ? backward_yaw_deg : forward_yaw_deg;
    plan->world_yaw_error_deg = choose_backward ? backward_error_deg : forward_error_deg;
    plan->body_dy_m = choose_backward ? -plan->distance_m : plan->distance_m;
    plan->chassis_turn_deg = -plan->world_yaw_error_deg;
    plan->turn_required =
        (plan->world_yaw_error_deg > tolerance_deg ||
         plan->world_yaw_error_deg < -tolerance_deg) ? 1U : 0U;

    return 1U;
}
