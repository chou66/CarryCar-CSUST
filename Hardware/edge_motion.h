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
    EDGE_MOTION_SETTLE_AFTER_TURN
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
 * 清除上一条路线的显示/诊断值。实际边航向始终在领取每一条边时由
 * edge_turn 根据 from/to 坐标重新计算。
 */
void EdgeMotion_BeginRoute(EdgeMotion_t *em);

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
