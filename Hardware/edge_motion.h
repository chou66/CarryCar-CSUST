#ifndef __EDGE_MOTION_H
#define __EDGE_MOTION_H

#include "chassis.h"
#include "hmi_navigation.h"
#include <stdint.h>

/*
 * 正式导航运动层（Navigation -> Chassis 的桥接层）
 *
 * 10.4 原版已经有 EdgeMotion 框架，但只负责：
 *   取当前边 -> chassis_motion_translate() -> 到位 -> EdgeReached()
 *
 * 本版把 VOFA 导航实车调试中已经验证过的 HWT101 边末航向修正迁入这里：
 *   平移 -> 停稳 -> 检查 yaw -> 必要时低速纠偏 -> 再停稳 -> EdgeReached()
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
    EDGE_MOTION_ERROR
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
    float edge_heading_ref_deg;

    /* 边末 HWT101 航向修正状态。 */
    uint32_t settle_until_ms;
    uint32_t yaw_correct_start_ms;
    uint32_t next_yaw_refresh_ms;
    int8_t yaw_start_sign;
    uint8_t yaw_velocity_active;

    /* EdgeMotion 临时使用导航标定加速度，边结束后恢复调用者原值。 */
    uint8_t saved_acceleration;
    uint8_t acceleration_saved;
} EdgeMotion_t;

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav);
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
