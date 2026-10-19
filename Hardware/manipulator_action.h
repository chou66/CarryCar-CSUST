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
 *   carousel_deg    —— 夹爪旋转舵机目标角度（外部取料/放置方向），
 *                      直接来自 robot_config.h 的 CAROUSEL_*_DEG 标定宏
 *   slide_extend_mm —— 电机6 前伸距离，正数=前伸
 *   gimbal_drop_mm  —— 电机5 下降深度，负数=下降
 *   layer           —— 叠放层号：0=第一层（不叠放），1=叠到第一层上；
 *                      实际下降深度 = gimbal_drop_mm + layer * MANIP_LAYER_STEP_MM
 *
 * 回程位移由引擎内部按 -有效值现算，调用方不需要提供。
 */
typedef struct
{
    float    carousel_deg;
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
    MANIP_ACTION_ERROR,

    /*
     * 追加在末尾，保持前面所有枚举值不变。
     *
     * 视觉已经完成、滑台停在相机验证过的伸出量、云台已在对准角度且夹爪
     * 保持张开时，不再退回原点重新伸出，直接下降电机5抓取。
     */
    MANIP_GROUND_TO_TRAY_DIRECT_DESCEND,
    MANIP_GROUND_TO_TRAY_WAIT_DIRECT_DESCEND
} ManipulatorActionState_t;

/*
 * 视觉预置/点动子状态机：视觉识别前后各一段机械预置动作。
 *
 *   MANIP_JOG_IDLE   未在动作
 *   MANIP_JOG_ROTATE 先转云台：夹爪转到识别姿态并张开，等舵机到位
 *   MANIP_JOG_MOVE   滑台指令已发出（等下一个 Update 切进 WAIT）
 *   MANIP_JOG_WAIT   等滑台到位
 *   MANIP_JOG_LOWER  滑台已到识别位，等电机5下降到预取像高度
 *   MANIP_JOG_RAISE  回原点前，等电机5先抬回最高原点
 *   MANIP_JOG_DONE   到位
 *   MANIP_JOG_ERROR  发送失败 / 超时 / 堵转保护
 *
 * 只有「伸到识别位」那一段会先走 ROTATE（云台先转、滑台后伸）；
 * 「退回原点」不动云台，直接从 MOVE 开始。
 * 暂存区带 STORAGE_VISION_PRE_DROP_MM 时，WAIT 到位后再走 LOWER；
 * 带预下降的回原点则先走 RAISE 抬回最高点，再回缩滑台。
 * 原料区不做预下降，仍然直接 DONE。
 */
typedef enum
{
    MANIP_JOG_IDLE = 0,
    MANIP_JOG_MOVE,
    MANIP_JOG_WAIT,
    MANIP_JOG_DONE,
    MANIP_JOG_ERROR,
    /* 追加在末尾，保持前面所有枚举值不变。 */
    MANIP_JOG_ROTATE,
    MANIP_JOG_LOWER,
    MANIP_JOG_RAISE
} ManipulatorSlideJogState_t;

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

/*
 * 原料区专用：从“视觉已经对准的当前位置”直接取料。
 *
 * 与 StartToTrayAtPose 的区别：
 *   - 不重新发送云台角度、不重新张开夹爪（视觉预置已经完成）；
 *   - 不发送滑台伸出指令（滑台已经停在相机验证过的位置）；
 *   - 直接从当前姿态下降电机5 -> 闭合夹爪，然后沿用 TO_TRAY 后半段：
 *     抬升 -> 滑台退回原点 -> 转料盘 -> 下降放料 -> 回原点。
 *
 * 调用前提：
 *   - 主动作空闲，滑台点动不在忙；
 *   - 滑台位置基准可信，且当前位置与 pickup_pose->slide_extend_mm 一致
 *     （容差 0.5 mm）；
 *   - 电机5位于最高原点（原料区不做视觉预下降），夹爪已张开。
 *
 * 返回值 false 表示前提不满足或参数非法；此时不改变任何动作状态。
 * 回程滑台位移按“实测当前位置”计算，确保夹取后能准确退回 0 mm。
 */
bool ManipulatorAction_StartToTrayFromAlignedPose(const ManipPose_t *pickup_pose);

bool ManipulatorAction_StartTrayToGroundAtPose(const ManipPose_t *place_pose);

void ManipulatorAction_Update(uint32_t now_ms);
void ManipulatorAction_Abort(void);

/*
 * 视觉预置、位置点动与回退：给视觉识别腾出取像位姿与取像距离。
 *
 * 相机装在夹爪上，识别统一在取料位2(POS2)姿态下做，所以识别之前要先把
 * 两件事按顺序做完（顺序不能换）：
 *   1) 云台先转——夹爪转到识别姿态 POS2 并张开。夹爪在缩回状态下回转，
 *      比在伸出 95mm 后再转更不容易和外围干涉。
 *   2) 滑台后伸——通用识别伸到 POS2 的伸出量；原料区停靠后使用
 *      RAW_VISION_SLIDE_START_MM（已扣除底盘靠近量），
 *      让取像距离与识别姿态自洽。
 *   3) 暂存区在滑台到位后，再下降电机5 STORAGE_VISION_PRE_DROP_MM，
 *      让相机靠近圆环后再发 place。原料区/粗加工区不预下降，保持在
 *      最高原点识别。
 * 三件都就位后才由上层发 detect/place（见 K230Align_StartPreset）。
 * 识别结束后：
 *   - 原料区由 RawMaterialAlign 动态调整云台/滑台，对准完成后调用
 *     StartToTrayFromAlignedPose() 从最高原点直接下降抓取，不再退回原点；
 *   - 粗加工区/暂存区仍由调用方退回原点：暂存区若带预下降，先 RAISE
 *     抬回最高点，再回缩滑台，然后从最高原点执行固定位姿动作。
 * 动作引擎里「伸出=发相对位移、回程=发负值」的假设因此保持不变。
 *
 * 模块会在每次电机到位确认后记录滑台绝对位置。原料视觉位置环可反复调用
 * SlideJogToPositionStart(target_mm)；它把绝对目标换算成相对位移。中途失败
 * 或急停后位置会标成未知，拒绝继续累计，必须人工回零并重新上电/立基准。
 *
 * 用法与主动作相同：Start 一次，主循环持续调用 ManipulatorAction_Update()。
 * 启动条件不满足时返回 false 且不改变任何状态：
 *   SlideJogToAlignStart —— 暂存区通用识别预置
 *   SlideJogToRoughAlignStart —— 粗加工区2号参考环预置，使用实车标定的
 *                                ROUGH_SLIDE_POS2_MM
 *   SlideJogToStorageAlignStart —— 暂存区2号参考环预置，使用独立的
 *                                  STORAGE_SLIDE_POS2_MM
 *   SlideJogToRawAlignStart —— 原料停靠后的补偿预伸
 *   SlideJogToPositionStart —— 当前位置可信，走到新的绝对目标
 *   SlideJogHomeStart       —— 从当前可信的非零位置回原点；若此前做过
 *                              视觉预下降，先抬回最高原点，再回缩滑台
 *
 * 名字里的 "Jog" 是沿用旧的滑台点动叫法：这两个入口都以「滑台挪一段」
 * 为目的，ToAlignStart 额外承担了上面第 1 步的云台预转。
 *
 * 到位后由本模块调用 Manipulator_Slide_SetAlign()/SetHome() 更新基准；
 * 预置/点动失败会 Manipulator_StopAll()，位置基准随之作废（须人工归零）。
 * 主动作与本状态机互斥：预置进行中 start 主动作会返回 false，反之亦然。
 */
bool ManipulatorAction_SlideJogToAlignStart(void);
bool ManipulatorAction_SlideJogToRoughAlignStart(void);
bool ManipulatorAction_SlideJogToStorageAlignStart(void);
bool ManipulatorAction_SlideJogToRawAlignStart(void);
bool ManipulatorAction_SlideJogToPositionStart(float target_mm);
bool ManipulatorAction_SlideJogHomeStart(void);

/*
 * 人工确认滑台已在机械原点后，重建 0 mm 软件坐标。
 * 本函数绝不会主动回零；滑台不在机械原点时禁止调用，否则后续绝对距离会
 * 整体偏移。主要供带确认口令的 VOFA 标定状态机使用。
 */
bool ManipulatorAction_SlideJogRebaseHome(void);

float ManipulatorAction_SlideJogGetPositionMm(void);
bool ManipulatorAction_SlideJogPositionKnown(void);

/*
 * 视觉预下降状态（暂存区识别用；原料区不做预下降）。
 * 返回已经实际施加到电机5上的下降量，负数=当前比最高原点低；
 * 未预下降或已由后续动作接管时返回 0。
 */
float ManipulatorAction_SlideJogGetVisionDropMm(void);
void ManipulatorAction_SlideJogClearVisionDrop(void);

bool ManipulatorAction_SlideJogIsBusy(void);
bool ManipulatorAction_SlideJogIsDone(void);
bool ManipulatorAction_SlideJogHasError(void);

/*
 * 点动状态镜像（非 static 全局，Keil Watch 直接输名字即可）：
 *   robot_manip_jog_state       ManipulatorSlideJogState_t 当前值
 *                               （=5 是 MANIP_JOG_ROTATE，即云台预转中）
 *   manip_debug_jog_fail_reason 点动失败原因（MANIP_FAIL_xxx），成功/Init 清零
 */
extern volatile uint8_t robot_manip_jog_state;
extern volatile uint8_t manip_debug_jog_fail_reason;
extern volatile float robot_manip_slide_position_mm;
extern volatile uint8_t robot_manip_slide_position_valid;

/* 视觉预下降镜像：drop_mm 负数=比最高原点低，applied=1 表示尚未抬回。 */
extern volatile float robot_manip_vision_drop_mm;
extern volatile uint8_t robot_manip_vision_drop_applied;

bool ManipulatorAction_IsBusy(void);
bool ManipulatorAction_IsDone(void);
bool ManipulatorAction_HasError(void);

ManipulatorActionState_t ManipulatorAction_GetState(void);

#endif
