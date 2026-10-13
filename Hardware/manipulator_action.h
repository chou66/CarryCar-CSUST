#ifndef __MANIPULATOR_ACTION_H
#define __MANIPULATOR_ACTION_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file manipulator_action.h
 * @brief 机械臂组合动作状态机接口。
 *
 * 本模块使用两个参数确定一次“取料并放入物料托盘”的动作：
 *
 * 1. ManipulatorPickupSource_t：选择电机5下降高度。
 *    - MANIP_PICKUP_SOURCE_GROUND：
 *      电机5下降 MANIP_PICKUP_GROUND_MM，从地面取料。
 *
 *    - MANIP_PICKUP_SOURCE_TABLE：
 *      电机5下降 MANIP_PICKUP_TABLE_MM，从原料台取料。
 *
 * 2. ManipulatorPickupPosition_t：选择夹爪旋转舵机的取料方向。
 *    - MANIP_PICKUP_POSITION_1：夹取位置1。
 *    - MANIP_PICKUP_POSITION_2：夹取位置2。
 *    - MANIP_PICKUP_POSITION_3：夹取位置3。
 *
 * 推荐调用方法：
 *
 * 从地面位置1取料：
 * @code
 * ManipulatorAction_StartGroundToTrayAt(
 *     MANIP_PICKUP_POSITION_1);
 * @endcode
 *
 * 从地面位置2取料：
 * @code
 * ManipulatorAction_StartGroundToTrayAt(
 *     MANIP_PICKUP_POSITION_2);
 * @endcode
 *
 * 从原料台位置3取料：
 * @code
 * ManipulatorAction_StartTableToTrayAt(
 *     MANIP_PICKUP_POSITION_3);
 * @endcode
 *
 * 也可以同时明确指定取料高度和方向：
 * @code
 * ManipulatorAction_StartToTrayAt(
 *     MANIP_PICKUP_SOURCE_GROUND,
 *     MANIP_PICKUP_POSITION_1);
 * @endcode
 *
 * 旧接口继续保留，并默认使用夹取位置2：
 * @code
 * ManipulatorAction_StartGroundToTray();
 * ManipulatorAction_StartTableToTray();
 * @endcode
 *
 * 注意：
 * - Start函数只调用一次，用于启动动作。
 * - 主循环必须持续调用ManipulatorAction_Update()。
 * - 平滑舵机还需要持续调用CarouselServo_Update()和
 *   GripperServo_Update()。
 * - 电机5、6使用相对位移控制，启动前必须位于规定机械原点。
 */
 
typedef enum
	{/*电机5下降高度*/
    MANIP_PICKUP_SOURCE_GROUND = 0,
    MANIP_PICKUP_SOURCE_TABLE
} ManipulatorPickupSource_t;


typedef enum
{/*地面三个夹取方向*/
    MANIP_PICKUP_POSITION_1 = 1,
    MANIP_PICKUP_POSITION_2,
    MANIP_PICKUP_POSITION_3
} ManipulatorPickupPosition_t;

/*
 * 机械臂“外部目标位姿”：一次动作在外部端（取料点或放置点）的参数。
 * 上层按区域填表，引擎按此参数执行，不再依赖硬编码距离。
 *
 *   carousel_us     —— 夹爪旋转舵机目标脉宽（外部取料/放置方向），
 *                      直接来自 robot_config.h 的 CAROUSEL_*_US 标定宏
 *   slide_extend_mm —— 电机6 前伸距离，正数=前伸
 *   gimbal_drop_mm  —— 电机5 下降深度，负数=下降
 *   layer           —— 叠放层号：0=第一层（不叠放），1=叠到第一层上；
 *                      实际下降深度 = gimbal_drop_mm + layer * MANIP_LAYER_STEP_MM
 *
 * 回程位移由引擎内部按 -有效值现算，调用方不需要提供。
 */
typedef struct
{
    uint16_t carousel_us;
    float    slide_extend_mm;
    float    gimbal_drop_mm;
    uint8_t  layer;
} ManipPose_t;

/* 当前支持的叠放层数上限（含第一层）：layer 取值 0..MANIP_MAX_LAYER。 */
#define MANIP_MAX_LAYER   1U

typedef enum
{
    MANIP_ACTION_IDLE = 0,

    /* 夹爪旋转到地面夹取方向，并张开夹爪。 */
    MANIP_GROUND_TO_TRAY_PREPARE,
    MANIP_GROUND_TO_TRAY_WAIT_PREPARE,

    /* 电机5下降，电机6前伸。 */
    MANIP_GROUND_TO_TRAY_MOVE_TO_GROUND,
    MANIP_GROUND_TO_TRAY_WAIT_GROUND,

    /* 夹紧物料。 */
    MANIP_GROUND_TO_TRAY_CLOSE,
    MANIP_GROUND_TO_TRAY_WAIT_CLOSE,

		/* 电机5先上升回最高原点。 */
		MANIP_GROUND_TO_TRAY_RETURN_GIMBAL,
		MANIP_GROUND_TO_TRAY_WAIT_RETURN_GIMBAL,
		
		/* 电机5到位后，电机6再后缩回原点。 */
		MANIP_GROUND_TO_TRAY_RETRACT_SLIDE,
		MANIP_GROUND_TO_TRAY_WAIT_RETRACT_SLIDE,

    /* 夹爪旋转到料盘放置方向。 */
    MANIP_GROUND_TO_TRAY_ROTATE_TO_PLACE,
    MANIP_GROUND_TO_TRAY_WAIT_ROTATE,

    /* 电机5下降到料盘高度。 */
    MANIP_GROUND_TO_TRAY_MOVE_TO_TRAY,
    MANIP_GROUND_TO_TRAY_WAIT_TRAY,

    /* 张开夹爪，释放物料。 */
    MANIP_GROUND_TO_TRAY_OPEN,
    MANIP_GROUND_TO_TRAY_WAIT_OPEN,

    /* 电机5上升回原点。 */
    MANIP_GROUND_TO_TRAY_RETURN_HOME,
    MANIP_GROUND_TO_TRAY_WAIT_HOME,

    /* 夹爪旋转到物料托盘方向。 */
    MANIP_TRAY_TO_GROUND_ROTATE_TO_TRAY,
    MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_TRAY,

    /* 电机5下降到物料托盘夹取高度。 */
    MANIP_TRAY_TO_GROUND_MOVE_TO_TRAY,
    MANIP_TRAY_TO_GROUND_WAIT_TRAY,

    /* 从物料托盘夹紧物料。 */
    MANIP_TRAY_TO_GROUND_CLOSE,
    MANIP_TRAY_TO_GROUND_WAIT_CLOSE,

    /* 电机5上升回最高原点。 */
    MANIP_TRAY_TO_GROUND_RETURN_HOME,
    MANIP_TRAY_TO_GROUND_WAIT_HOME,

		/* 夹爪单独旋转到地面夹取方向。 */
		MANIP_TRAY_TO_GROUND_ROTATE_TO_GROUND,
		MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_GROUND,
		
		/* 舵机到位后，电机5下降、电机6前伸。 */
		MANIP_TRAY_TO_GROUND_MOVE_TO_GROUND,
		MANIP_TRAY_TO_GROUND_WAIT_GROUND,

    /* 张开夹爪，将物料释放到地面目标位置。 */
    MANIP_TRAY_TO_GROUND_OPEN,
    MANIP_TRAY_TO_GROUND_WAIT_OPEN,

		/* 电机5先上升回最高原点。 */
		MANIP_TRAY_TO_GROUND_FINAL_RETURN_GIMBAL,
		MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETURN_GIMBAL,
		
		/* 电机5到位后，电机6再后缩回原点。 */
		MANIP_TRAY_TO_GROUND_FINAL_RETRACT_SLIDE,
		MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETRACT_SLIDE,

    MANIP_ACTION_DONE,
    MANIP_ACTION_ERROR
} ManipulatorActionState_t;

/*
 * 夹取失败现场记录，全局可见，Keil Watch 直接观察。
 *   manip_debug_fail_state  = 失败时刻的 action_state；
 *   manip_debug_fail_reason = MANIP_FAIL_xxx；
 *   manip_debug_fail_motor  = 未到位/故障电机的位掩码（位定义同 MOTOR_GIMBAL_MASK）。
 * Start 成功、Init 或复位时清零。
 */
#define MANIP_FAIL_NONE       0U  /* 尚未失败 */
#define MANIP_FAIL_SEND       1U  /* 位置指令 CAN 发送失败 */
#define MANIP_FAIL_TIMEOUT    2U  /* 电机到位等待超时 */
#define MANIP_FAIL_FAULT      3U  /* 电机上报堵转/故障位 */
#define MANIP_FAIL_SERVO      4U  /* 舵机/夹爪阶段等待超时 */
#define MANIP_FAIL_ABORT      5U  /* 被 Abort 打断 */
#define MANIP_FAIL_STATE      6U  /* 状态机进入未知状态 */

extern volatile uint8_t manip_debug_fail_state;
extern volatile uint8_t manip_debug_fail_reason;
extern volatile uint8_t manip_debug_fail_motor;

/* 失败电机的最后 0x3A 状态字节：bit0=使能，bit2=堵转检测，bit3=堵转保护锁存。 */
extern volatile uint8_t manip_debug_fail_status;

/*
 * 状态镜像（非 static 全局，Keil Watch 直接输名字即可，无需限定名）：
 *   robot_manip_action_state = ManipulatorActionState_t 当前值
 *   robot_manip_active_flow  = MANIP_FLOW_xxx（1=装盘 TO_TRAY，2=卸盘 FROM_TRAY）
 * 由 manipulator_action.c 的 sync_action_state() 在状态/流程变化时刷新。
 */
extern volatile uint8_t robot_manip_action_state;
extern volatile uint8_t robot_manip_active_flow;

/*
 * 最近一次到位确认的证据来源快照（每段开始时清零）：
 *   bit4 = 电机5 经 0x02 标志边沿确认（段内先见过未到位，后置位）
 *   bit5 = 电机6 经 0x02 标志边沿确认
 */
extern volatile uint8_t manip_debug_confirm_src;

void ManipulatorAction_Init(void);
bool ManipulatorAction_StartToTray(ManipulatorPickupSource_t source);

/* 地面/原料台取料，然后放入物料托盘。 */
bool ManipulatorAction_StartTableToTray(void);
bool ManipulatorAction_StartGroundToTray(void);

/* 从物料托盘取料，然后放到地面目标位置。 */
bool ManipulatorAction_StartTrayToGround(void);

/*从当前物料托盘位置取料，然后放到指定的地面位置*/
bool ManipulatorAction_StartTrayToGroundAt( ManipulatorPickupPosition_t position);

/*根据来源和夹取方向启动装盘流程*/
bool ManipulatorAction_StartToTrayAt(
    ManipulatorPickupSource_t source,
    ManipulatorPickupPosition_t position);

/*从地面指定方向夹取，然后放到物料托盘。*/
bool ManipulatorAction_StartGroundToTrayAt(
    ManipulatorPickupPosition_t position);

/*从原料台指定方向夹取，然后放到物料托盘*/
bool ManipulatorAction_StartTableToTrayAt(
    ManipulatorPickupPosition_t position);

/*
 * 区域参数化入口：外部端位姿由调用方（zone_task 步骤表）给出。
 *   StartToTrayAtPose        —— 从外部（原料区/粗加工区工位）取料，放进料盘
 *   StartTrayToGroundAtPose  —— 从料盘取料，放到外部（粗加工区/暂存区）工位
 * 返回值 false 表示参数非法或当前动作未结束；此时不改变任何状态。
 * 旧接口全部保留，内部按原标定值组装 pose 后转发到这两个函数。
 */
bool ManipulatorAction_StartToTrayAtPose(const ManipPose_t *pickup_pose);
bool ManipulatorAction_StartTrayToGroundAtPose(const ManipPose_t *place_pose);

void ManipulatorAction_Update(uint32_t now_ms);
void ManipulatorAction_Abort(void);

bool ManipulatorAction_IsBusy(void);
bool ManipulatorAction_IsDone(void);
bool ManipulatorAction_HasError(void);

ManipulatorActionState_t ManipulatorAction_GetState(void);

#endif
