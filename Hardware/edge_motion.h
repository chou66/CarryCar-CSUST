#ifndef __EDGE_MOTION_H
#define __EDGE_MOTION_H

#include "chassis.h"
#include "heading_control.h"
#include "hmi_navigation.h"
#include <stdint.h>

/*
 * 正式导航运动层（Navigation -> Chassis 的桥接层）
 *
 * 10.4 原版已经有 EdgeMotion 框架，但只负责：
 *   取当前边 -> chassis_motion_translate() -> 到位 -> EdgeReached()
 *
 * 每条边采用“纵向优先”策略：
 *   读取 from/to -> 同时评估前进/后退转角 -> 必要时原地转向 -> 只沿车身Y轴平移
 *   -> 停稳 -> 检查 yaw -> 必要时自适应闭环纠偏 -> EdgeReached()
 *
 * 领取边时会再向前预读一个拐角（ROUTE_CORNER_ARC_ENABLE）：若本段直线
 * 终点处要转 90° 的弯，则直线末段保持满速，以 ROUTE_CORNER_ARC_RADIUS_M
 * 走一段圆弧转过拐角，再接着走下一条边——不在拐角节点停车原地转，
 * 效果是“走一段直线，走一段弧线转过90°，再走一段直线”。
 *
 * 倒车边（例如 N2 车头朝东、先倒车去 N1 再去 N4）也走同一套圆弧：
 * 车头锁在切线+180°，直线和圆弧都按倒车执行，拐角同样不停车。
 *
 * N2（粗加工）-> N4（暂存）这一段单独使用 ROUTE_SEG_N2_N4_* 参数
 * （转弯容差、倒车阈值、转向速度、圆弧半径/最大转角/前后余量），
 * 与其它区段的全局 ROUTE_* 参数互不影响，见 edge_motion.c 的
 * edge_turn_tuning()。
 *
 * 领取边时会向前预读路径，把“共线同向”的连续拓扑边合并成一次直线平移
 * （见 ROUTE_MERGE_STRAIGHT_EDGES）。因此中途那些没有任务的节点既不停车，
 * 也不做边末航向校准；只有需要拐弯的节点和本次导航的目标节点才停车。
 *
 * 普通节点不需要配置航向角。edge_turn 层直接从相邻节点坐标计算本边航向；
 * EdgeMotion 状态机只负责异步动作和复查。路径拐弯时先选“少转向”的车身
 * 朝向，再前进或后退，因此不再用麦克纳姆大距离横移，也不会为了始终前进
 * 而无意义地转180°。
 *
 * 注意：
 * 1. 不修改 CAN / EMM / Chassis 底层；
 * 2. 不删除 VOFA 原调试状态机，保留它作为独立回归基线；
 * 3. 只有真实物理边完成并完成航向确认后，才允许通知 Navigation_EdgeReached()。
 */
typedef enum
{
    EDGE_MOTION_IDLE = 0,
    EDGE_MOTION_TRANSLATING,
    EDGE_MOTION_SETTLE_AFTER_TRANSLATE,
    EDGE_MOTION_YAW_CORRECTING,
    EDGE_MOTION_SETTLE_AFTER_YAW,
    EDGE_MOTION_PAUSE_ABORT,
    EDGE_MOTION_ERROR,

    /* 追加状态，保留旧状态的数值，方便既有 Keil Watch/调试脚本。 */
    EDGE_MOTION_TURNING_TO_EDGE,
    EDGE_MOTION_SETTLE_AFTER_TURN,

    /* 圆滑拐角：直线末段保持满速 -> 圆弧转过拐角 -> 下一条边的直线。 */
    EDGE_MOTION_CORNER_ARC
} EdgeMotionState_t;

/* 兼容 10.4 robot_app.c / 旧调试代码中对 MOVING 的判断。 */
#define EDGE_MOTION_MOVING EDGE_MOTION_TRANSLATING

typedef struct
{
    Chassis *chassis;
    HMI_Navigation_t *hmi_nav;
    EdgeMotionState_t state;

    /* 当前正在执行的拓扑边。只有 commit 后 Navigation 才推进 current_node。 */
    uint8_t from_node;
    uint8_t to_node;

    /*
     * 本次纵向平移实际跨越了几条拓扑边。
     *
     * 路径上共线同向的连续边（例如 1->2->3 同一条车道）会合并成一次直线
     * 平移，中间节点不停车、不做边末航向校准；from_node/to_node 记录的是
     * 合并后的起终点，run_steps 是这次移动覆盖的拓扑边条数。
     * commit 时按 run_steps 连续推进 Navigation 的节点下标。
     */
    uint8_t run_steps;

    /*
     * 世界坐标 -> 车身坐标转换诊断值，Keil Watch 可直接观察。
     * 世界：+X右、+Y上；车身：+X右、+Y前。
     */
    float world_dx_m;
    float world_dy_m;
    float body_dx_m;
    float body_dy_m;

    /* 当前边由节点坐标计算出的世界航向；Keil Watch/HMI 可直接观察。 */
    float edge_heading_ref_deg;

    /* 转向层诊断值：世界角误差与实际下发到底盘的位置转角符号相反。 */
    float turn_world_error_deg;
    float turn_chassis_command_deg;
    uint8_t drive_backward; /* 1=当前边使用车身-Y后退；0=车身+Y前进。 */
    uint8_t turn_attempt;

    /*
     * 圆滑拐角预读结果：本段直线结束后不再停车，而是走圆弧转过拐角，
     * 再接着走下一条边。corner_valid=0 时保持原来的“到节点停车”。
     */
    uint8_t corner_valid;
    uint8_t corner_next_to_node;   /* 圆弧后直线的终点（已合并共线边） */
    uint8_t corner_next_steps;     /* 圆弧后直线覆盖的拓扑边条数 */
    /*
     * 1=倒车过弯：本段直线是倒车边，车头朝向 = 行进切线 + 180°，圆弧
     * 也按倒车走，弧后直线继续倒车。用于 N2->N1->N4 这类“倒着进弯”的
     * 路径，仍然不在拐角节点停车。
     */
    uint8_t corner_reverse;
    float corner_turn_deg;         /* 圆弧转角，世界系顺时针为正（切线转角） */
    float corner_next_world_dx_m;  /* 圆弧后直线的世界向量 */
    float corner_next_world_dy_m;
    float corner_next_length_m;    /* 圆弧后直线长度（未扣半径） */
    float corner_next_yaw_deg;     /* 圆弧后直线的目标航向 */
    /*
     * 本段拐角实际采用的圆弧半径（m），corner_valid=1 时有效。
     * 普通区段等于 ROUTE_CORNER_ARC_RADIUS_M；N2->N4 区段取
     * ROUTE_SEG_N2_N4_CORNER_ARC_RADIUS_M。Keil Watch 可直接观察。
     */
    float corner_arc_radius_m;
    /* 刚走完的圆弧半径（m），供下一条直线的入口修剪使用；0=不是从圆弧出来。 */
    float arc_entry_radius_m;

    /*
     * 调用方声明：本次导航到达最终节点后，会直接做任务航向 PD。
     *
     * 置 1 时，边末若已经确定后面还有一次更大的转向（任务航向对准或下一
     * 段拓扑边转向），就不再先把车纠到当前边航向，而是直接提交节点，让
     * 那一次转向直接转到目标角度。默认 0：边末始终按原逻辑纠偏，独立导航
     * 测试/未声明任务航向的调用方行为不变。
     */
    uint8_t arrival_align_valid;

    /* 边末 HWT101 航向修正状态。 */
    uint32_t settle_until_ms;
    uint32_t yaw_correct_start_ms;
    uint32_t yaw_correct_deadline_ms;
    uint32_t next_yaw_refresh_ms;
    int8_t yaw_start_sign;
    uint8_t yaw_velocity_active;
    float yaw_command_wz_radps;
    HeadingController yaw_controller;

    /* EdgeMotion 临时使用导航标定加速度，边结束后恢复调用者原值。 */
    uint8_t saved_acceleration;
    uint8_t acceleration_saved;
} EdgeMotion_t;

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav);

/*
 * @brief 通知运动层“一次新的完整路径规划已经开始”。
 *
 * 清除上一条路线的显示/诊断值；实际边航向始终在领取每一条边时由
 * edge_turn 根据 from/to 坐标重新计算。
 */
void EdgeMotion_BeginRoute(EdgeMotion_t *em);

/*
 * @brief 声明本次导航到达最终节点后，上层会立即按节点的 task_yaw 做任务
 *        航向对准。
 *
 * 必须在 EdgeMotion_BeginRoute()/RobotApp_GotoNode() 之后调用；BeginRoute
 * 会把它清 0，避免上一条路线的声明泄漏到 VOFA/标定等其它导航。
 */
void EdgeMotion_SetArrivalAlign(EdgeMotion_t *em, uint8_t valid);

void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms);
void EdgeMotion_EStop(EdgeMotion_t *em);
EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em);

/*
 * @brief 当前是否已经接管一条真实物理边。
 *
 * 用于 RobotApp 在地图重规划/无路时判断是否必须停车。
 * IDLE / ERROR / PAUSE_ABORT 返回 0，其余边执行状态返回 1。
 */
uint8_t EdgeMotion_IsActive(const EdgeMotion_t *em);

#endif
