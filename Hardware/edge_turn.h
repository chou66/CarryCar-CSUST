#ifndef __EDGE_TURN_H
#define __EDGE_TURN_H

#include <stdint.h>

/*
 * 拓扑边转向决策层。
 *
 * 本层只做计算，不直接操作底盘：
 *   1. from_node -> to_node 在世界坐标中的几何方向是多少；
 *   2. 选择车头朝向该方向前进，还是车尾朝向该方向后退；
 *   3. 当前世界航向到所选车身航向的最短转角是多少；
 *   4. 是否已经在允许误差内、可以直接纵向行驶。
 *
 * 世界航向：0=地图向上(+Y)，+90=向右(+X)，顺时针为正。
 * 底盘转角：正=逆时针，负=顺时针。因此 chassis_turn_deg 与
 * world_yaw_error_deg 符号相反。
 */
typedef struct
{
    float world_dx_m;
    float world_dy_m;
    float distance_m;

    /* 当前边的几何方向，以及最终选中的车身世界航向。后退时两者相差180°。 */
    float edge_world_yaw_deg;
    float target_world_yaw_deg;
    float world_yaw_error_deg;
    float chassis_turn_deg;

    /* 正数=车身+Y前进，负数=车身-Y后退；绝对值始终等于distance_m。 */
    float body_dy_m;
    uint8_t drive_backward;
    uint8_t turn_required;
} EdgeTurnPlan_t;

/*
 * 双向纵向行驶的选择参数。
 *
 * arrival_yaw_valid 仅在当前边终点就是本次导航最终目标时置1：
 * 这时前进/后退的代价不仅包含“现在要转多少”，还包含到站后必做的任务
 * 航向转角。这样不会为了当前少转几度，却在工作节点额外转180°。
 */
typedef struct
{
    uint8_t allow_reverse;
    float reverse_min_turn_saving_deg;
    uint8_t arrival_yaw_valid;
    float arrival_world_yaw_deg;
} EdgeTurnOptions_t;

/* 把角度归一化到 [-180, 180]。 */
float EdgeTurn_WrapDeg(float angle_deg);

/*
 * 根据当前拓扑边和实时 IMU 世界航向生成转向计划。
 * 返回 1 表示边合法且计划有效；返回 0 表示参数/节点/连通性非法。
 */
uint8_t EdgeTurn_Plan(uint8_t from_node,
                      uint8_t to_node,
                      float current_world_yaw_deg,
                      float tolerance_deg,
                      const EdgeTurnOptions_t *options,
                      EdgeTurnPlan_t *plan);

#endif
