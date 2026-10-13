#include "manipulator_action.h"

#include "can.h"
#include "emm_adapter.h"
#include "emm_v5.h"
#include "manipulator.h"
#include "robot_config.h"
#include "servo.h"

#include <stddef.h>

/*
 * 机械臂动作由“取料高度”和“夹取方向”共同决定：
 *
 * source决定电机5下降高度：
 *   GROUND -> MANIP_PICKUP_GROUND_MM
 *   TABLE  -> MANIP_PICKUP_TABLE_MM
 *
 * position决定夹爪旋转舵机方向：
 *   POSITION_1 -> CarouselServo_ToPickupPos1()
 *   POSITION_2 -> CarouselServo_ToPickup()
 *   POSITION_3 -> CarouselServo_ToPickupPos3()
 *
 * 调用示例：
 *   ManipulatorAction_StartGroundToTrayAt(
 *       MANIP_PICKUP_POSITION_1);
 *
 * Start只调用一次，ManipulatorAction_Update()必须在主循环持续调用。
 * 电机5、6使用相对位移，执行前必须位于规定机械原点。
 *
 * 区域化入口：
 *   上面的 source/position 是旧接口的便捷封装，内部统一转成 ManipPose_t。
 *   需要按区域单独标定（原料区/粗加工区/暂存区、叠放层）时，改用
 *   ManipulatorAction_StartToTrayAtPose() / StartTrayToGroundAtPose()，
 *   位姿表定义在 robot_config.h 的 ZONE_*_POSE。
 */
 
#define MOTOR_MANIPULATOR_MASK \
    ((uint8_t)(MOTOR_GIMBAL_MASK | MOTOR_SLIDE_MASK))

/* manip_debug_confirm_src 的位定义（低 4 位保留给电机位掩码）。 */
#define MANIP_CONFIRM_GIMBAL_TF    0x10U /* 电机5 经 0x02 标志边沿确认 */
#define MANIP_CONFIRM_SLIDE_TF     0x20U /* 电机6 经 0x02 标志边沿确认 */
typedef enum
{
    MOTOR_WAITING = 0,
    MOTOR_REACHED,
    MOTOR_FAULT,
    MOTOR_TIMEOUT
} MotorWaitResult_t;

typedef enum
{
    MANIP_FLOW_NONE = 0,
    MANIP_FLOW_TO_TRAY,
    MANIP_FLOW_FROM_TRAY
} ManipulatorFlow_t;

volatile static ManipulatorActionState_t action_state = MANIP_ACTION_IDLE;
static ManipulatorFlow_t active_flow = MANIP_FLOW_NONE;

/*
 * 动作“外部端”参数：Start 时按传入的 ManipPose_t 一次性算好，
 * 状态机内只读，不再出现硬编码距离。
 *   TO_TRAY  （外部 -> 料盘）用 active_pickup_*
 *   FROM_TRAY（料盘 -> 外部）用 active_place_*
 * 回程一律取去程的反号。
 */
static uint16_t active_pickup_carousel_us = CAROUSEL_PICKUP_POS2_US;
static float active_pickup_slide_mm = MANIP_PICKUP_SLIDE_MM;
static float active_pickup_slide_return_mm = (-MANIP_PICKUP_SLIDE_MM);
static float active_pickup_gimbal_mm = MANIP_PICKUP_GROUND_MM;
static float active_pickup_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

static uint16_t active_place_carousel_us = CAROUSEL_PICKUP_POS2_US;
static float active_place_slide_mm = MANIP_PICKUP_SLIDE_MM;
static float active_place_slide_return_mm = (-MANIP_PICKUP_SLIDE_MM);
static float active_place_gimbal_mm = MANIP_PICKUP_GROUND_MM;
static float active_place_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

/* 失败现场记录：全局可见，Keil Watch 直接观察，Start/Init 时清零。 */
volatile uint8_t manip_debug_fail_state = 0U;
volatile uint8_t manip_debug_fail_reason = MANIP_FAIL_NONE;
volatile uint8_t manip_debug_fail_motor = 0U;
volatile uint8_t manip_debug_confirm_src = 0U;
volatile uint8_t manip_debug_fail_status = 0U;

/*
 * 状态镜像：非 static 全局，Keil Watch 直接输变量名即可观察，
 * 不必写「manipulator_action.c 反斜杠 action_state」这种全限定名。
 * 只读，不参与任何判断，由 sync_action_state() 刷新。
 */
volatile uint8_t robot_manip_action_state =
    (uint8_t)MANIP_ACTION_IDLE;
volatile uint8_t robot_manip_active_flow =
    (uint8_t)MANIP_FLOW_NONE;

static uint8_t pending_fail_reason = MANIP_FAIL_NONE;
static uint8_t pending_fail_motor = 0U;

static void record_pending_fail(uint8_t reason, uint8_t motor_bits);
static void clear_fail_debug(void);

static uint32_t state_start_ms = 0U;
static uint32_t motor_start_ms = 0U;
static uint32_t last_status_query_ms = 0U;

/* 把当前状态与流程方向刷新到 Watch 可见的镜像变量。 */
static void sync_action_state(void)
{
    robot_manip_action_state = (uint8_t)action_state;
    robot_manip_active_flow = (uint8_t)active_flow;
}

static void enter_state(
    ManipulatorActionState_t new_state,
    uint32_t now_ms)
{
    action_state = new_state;
    state_start_ms = now_ms;
    sync_action_state();
}

static void action_fail(uint32_t now_ms)
{
    /* 先冻结现场再转 ERROR，否则失败时的状态本身会被覆盖。 */
    manip_debug_fail_state = (uint8_t)action_state;
    manip_debug_fail_reason = pending_fail_reason;
    manip_debug_fail_motor = pending_fail_motor;
    pending_fail_reason = MANIP_FAIL_NONE;
    pending_fail_motor = 0U;

    /* 失败电机的最后状态字节：bit0=使能，bit2/3=堵转（现场证据）。 */
    if (manip_debug_fail_motor != 0U)
    {
        uint8_t id;

        for (id = 1U; id <= MOTOR_SLIDE_ID; ++id)
        {
            uint8_t bit = (uint8_t)(1U << (id - 1U));
            uint8_t status;
            uint32_t tick;

            if (((manip_debug_fail_motor & bit) != 0U) &&
                can_motor_status_get(id, &status, &tick))
            {
                manip_debug_fail_status = status;
                break;
            }
        }
    }

    Manipulator_StopAll();
    enter_state(MANIP_ACTION_ERROR, now_ms);
}

static void prepare_motor_feedback(uint8_t motor_mask)
{
    if ((motor_mask & MOTOR_GIMBAL_MASK) != 0U)
    {
        emm_prepare_position_feedback(MOTOR_GIMBAL_ID);
        manip_debug_confirm_src &=
            (uint8_t)~MANIP_CONFIRM_GIMBAL_TF;
    }

    if ((motor_mask & MOTOR_SLIDE_MASK) != 0U)
    {
        emm_prepare_position_feedback(MOTOR_SLIDE_ID);
        manip_debug_confirm_src &=
            (uint8_t)~MANIP_CONFIRM_SLIDE_TF;
    }
}

/*
 * 每 MANIP_PLACE_STATUS_PERIOD_MS 拉一次电机反馈：
 *   0x3A S_FLAG —— 到位标志路线。
 * 发送失败沿用底盘做法：不作判据，等下一周期重试。
 */
static void query_motor_feedback(
    uint8_t motor_mask,
    uint32_t now_ms)
{
    if ((now_ms - last_status_query_ms) <
        MANIP_PLACE_STATUS_PERIOD_MS)
    {
        return;
    }

    last_status_query_ms = now_ms;

    if ((motor_mask & MOTOR_GIMBAL_MASK) != 0U)
    {
        (void)emm_read_status(MOTOR_GIMBAL_ID);
    }

    if ((motor_mask & MOTOR_SLIDE_MASK) != 0U)
    {
        (void)emm_read_status(MOTOR_SLIDE_ID);
    }
}

/*
 * 到位判定沿用底盘逻辑：每台电机在守卫窗之后返回的新鲜 0x3A
 * 应答中 Prf_TF(bit1)=1。未到位则继续等待，直到超时转 ERROR。
 */
static MotorWaitResult_t wait_motor_position(
    uint8_t motor_mask,
    uint32_t now_ms)
{
    uint8_t confirmed_mask;
    query_motor_feedback(motor_mask, now_ms);

    /* 故障判据收窄：仅 0x08 堵转保护锁存算致命故障，
     * 落座压料瞬间的 0x04 堵转检测警告不抢占到位确认。 */
    if (emm_any_stall_protect_after(
            motor_mask,
            motor_start_ms))
    {
        record_pending_fail(
            MANIP_FAIL_FAULT,
            emm_faulted_mask_after(
                motor_mask,
                motor_start_ms));
        return MOTOR_FAULT;
    }

    confirmed_mask = emm_reached_mask_after(
        motor_mask,
        motor_start_ms + MANIP_MOTOR_FEEDBACK_GUARD_MS);

    manip_debug_confirm_src |= confirmed_mask; /* bit4=5号, bit5=6号 */

    if (confirmed_mask == motor_mask)
    {
        return MOTOR_REACHED;
    }

    if ((now_ms - motor_start_ms) >=
        MANIP_PLACE_MOTOR_TIMEOUT_MS)
    {
        record_pending_fail(
            MANIP_FAIL_TIMEOUT,
            (uint8_t)(motor_mask & (uint8_t)~confirmed_mask));
        return MOTOR_TIMEOUT;
    }

    return MOTOR_WAITING;
}

static bool motor_wait_failed(MotorWaitResult_t result)
{
    return (result == MOTOR_FAULT) ||
           (result == MOTOR_TIMEOUT);
}

static void record_pending_fail(uint8_t reason, uint8_t motor_bits)
{
    pending_fail_reason = reason;
    pending_fail_motor = motor_bits;
}

static void clear_fail_debug(void)
{
    manip_debug_fail_state = 0U;
    manip_debug_fail_reason = MANIP_FAIL_NONE;
    manip_debug_fail_motor = 0U;
    pending_fail_reason = MANIP_FAIL_NONE;
    pending_fail_motor = 0U;
    manip_debug_confirm_src = 0U;
    manip_debug_fail_status = 0U;
}

/* 旧枚举 -> 标定脉宽（与 robot_config.h 的 CAROUSEL_*_US 一一对应）。 */
static bool position_to_carousel_us(
    ManipulatorPickupPosition_t position,
    uint16_t *us_out)
{
    switch (position)
    {
        case MANIP_PICKUP_POSITION_1:
            *us_out = CAROUSEL_PICKUP_POS1_US;
            return true;

        case MANIP_PICKUP_POSITION_2:
            *us_out = CAROUSEL_PICKUP_POS2_US;
            return true;

        case MANIP_PICKUP_POSITION_3:
            *us_out = CAROUSEL_PICKUP_POS3_US;
            return true;

        default:
            return false;
    }
}

static bool carousel_us_in_range(uint16_t us)
{
    return (us >= SERVO_MIN_US) && (us <= SERVO_MAX_US);
}

/* 夹爪旋转到指定标定脉宽；越界算失败，不驱动作动器。 */
static bool command_carousel_us(uint16_t us)
{
    if (!carousel_us_in_range(us))
    {
        return false;
    }

    CarouselServo_SetUs(us);
    return true;
}

void ManipulatorAction_Init(void)
{
    action_state = MANIP_ACTION_IDLE;
		active_flow = MANIP_FLOW_NONE;
	
    active_pickup_carousel_us = CAROUSEL_PICKUP_POS2_US;
    active_pickup_slide_mm = MANIP_PICKUP_SLIDE_MM;
    active_pickup_slide_return_mm = (-MANIP_PICKUP_SLIDE_MM);
    active_pickup_gimbal_mm = MANIP_PICKUP_GROUND_MM;
    active_pickup_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

    active_place_carousel_us = CAROUSEL_PICKUP_POS2_US;
    active_place_slide_mm = MANIP_PICKUP_SLIDE_MM;
    active_place_slide_return_mm = (-MANIP_PICKUP_SLIDE_MM);
    active_place_gimbal_mm = MANIP_PICKUP_GROUND_MM;
    active_place_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);
    
		state_start_ms = 0U;
    motor_start_ms = 0U;
    last_status_query_ms = 0U;

    clear_fail_debug();
		
}

/*
 * 外部 -> 料盘：把外部端位姿参数化后的启动入口。
 * 参数非法或动作未结束时返回 false，且不改变任何状态。
 */
bool ManipulatorAction_StartToTrayAtPose(const ManipPose_t *pickup_pose)
{
    float effective_mm;

    if (pickup_pose == NULL)
    {
        return false;
    }

    if ((action_state != MANIP_ACTION_IDLE) &&
        (action_state != MANIP_ACTION_DONE))
    {
        return false;
    }

    if (!carousel_us_in_range(pickup_pose->carousel_us))
    {
        return false;
    }

    if (pickup_pose->layer > MANIP_MAX_LAYER)
    {
        return false;
    }

    effective_mm = pickup_pose->gimbal_drop_mm +
                   ((float)pickup_pose->layer * MANIP_LAYER_STEP_MM);

    active_pickup_carousel_us = pickup_pose->carousel_us;
    active_pickup_slide_mm = pickup_pose->slide_extend_mm;
    active_pickup_slide_return_mm = -pickup_pose->slide_extend_mm;
    active_pickup_gimbal_mm = effective_mm;
    active_pickup_gimbal_return_mm = -effective_mm;

    state_start_ms = HAL_GetTick();
    motor_start_ms = 0U;
    last_status_query_ms = 0U;

    clear_fail_debug();

    Emm_V5_En_Control(
        MOTOR_GIMBAL_ID,
        true,
        false);

    Emm_V5_En_Control(
        MOTOR_SLIDE_ID,
        true,
        false);

    action_state =
        MANIP_GROUND_TO_TRAY_PREPARE;
    active_flow = MANIP_FLOW_TO_TRAY;

    return true;
}

/*启动夹取物料（旧接口：按 source + position 组装默认 pose 后转发）*/
bool ManipulatorAction_StartToTrayAt(
    ManipulatorPickupSource_t source,
    ManipulatorPickupPosition_t position)
{
    ManipPose_t pose;

    switch (source)
    {
        case MANIP_PICKUP_SOURCE_GROUND:
            pose.gimbal_drop_mm = MANIP_PICKUP_GROUND_MM;
            break;

        case MANIP_PICKUP_SOURCE_TABLE:
            pose.gimbal_drop_mm = MANIP_PICKUP_TABLE_MM;
            break;

        default:
            return false;
    }

    if (!position_to_carousel_us(position, &pose.carousel_us))
    {
        return false;
    }

    pose.slide_extend_mm = MANIP_PICKUP_SLIDE_MM;
    pose.layer = 0U;

    return ManipulatorAction_StartToTrayAtPose(&pose);
}

bool ManipulatorAction_StartToTray(
    ManipulatorPickupSource_t source)
{
    return ManipulatorAction_StartToTrayAt(
        source,
        MANIP_PICKUP_POSITION_2);
}


bool ManipulatorAction_StartGroundToTray(void)
{
    return ManipulatorAction_StartGroundToTrayAt(
        MANIP_PICKUP_POSITION_2);
}

bool ManipulatorAction_StartTableToTray(void)
{
    return ManipulatorAction_StartTableToTrayAt(
        MANIP_PICKUP_POSITION_2);
}

bool ManipulatorAction_StartGroundToTrayAt(ManipulatorPickupPosition_t position)
{
    return ManipulatorAction_StartToTrayAt(
        MANIP_PICKUP_SOURCE_GROUND,
        position);
}

bool ManipulatorAction_StartTableToTrayAt(ManipulatorPickupPosition_t position)
{
    return ManipulatorAction_StartToTrayAt(
        MANIP_PICKUP_SOURCE_TABLE,
        position);
}

/*
 * 料盘 -> 外部：把外部端位姿参数化后的启动入口。
 * 参数非法（空指针 / 舵机脉宽越界 / 层号超限）或动作未结束时返回 false，
 * 且不改变任何状态。
 */
bool ManipulatorAction_StartTrayToGroundAtPose(const ManipPose_t *place_pose)
{
    float effective_mm;

    if (place_pose == NULL)
    {
        return false;
    }

    if ((action_state != MANIP_ACTION_IDLE) &&
        (action_state != MANIP_ACTION_DONE))
    {
        return false;
    }

    if (!carousel_us_in_range(place_pose->carousel_us))
    {
        return false;
    }

    if (place_pose->layer > MANIP_MAX_LAYER)
    {
        return false;
    }

    effective_mm = place_pose->gimbal_drop_mm +
                   ((float)place_pose->layer * MANIP_LAYER_STEP_MM);

    active_place_carousel_us = place_pose->carousel_us;
    active_place_slide_mm = place_pose->slide_extend_mm;
    active_place_slide_return_mm = -place_pose->slide_extend_mm;
    active_place_gimbal_mm = effective_mm;
    active_place_gimbal_return_mm = -effective_mm;

    clear_fail_debug();

    state_start_ms = HAL_GetTick();
    motor_start_ms = 0U;
    last_status_query_ms = 0U;

    Emm_V5_En_Control(
        MOTOR_GIMBAL_ID,
        true,
        false);

    Emm_V5_En_Control(
        MOTOR_SLIDE_ID,
        true,
        false);

    action_state =
        MANIP_TRAY_TO_GROUND_ROTATE_TO_TRAY;

    active_flow = MANIP_FLOW_FROM_TRAY;

    return true;
}

/* 旧接口：按 position 组装默认 pose 后转发。 */
bool ManipulatorAction_StartTrayToGroundAt(
    ManipulatorPickupPosition_t position)
{
    ManipPose_t pose;

    if (!position_to_carousel_us(position, &pose.carousel_us))
    {
        return false;
    }

    pose.slide_extend_mm = MANIP_PICKUP_SLIDE_MM;
    pose.gimbal_drop_mm = MANIP_PICKUP_GROUND_MM;
    pose.layer = 0U;

    return ManipulatorAction_StartTrayToGroundAtPose(&pose);
}

bool ManipulatorAction_StartTrayToGround(void)
{
    return ManipulatorAction_StartTrayToGroundAt(
        MANIP_PICKUP_POSITION_2);
}

static void update_to_tray(uint32_t now_ms)
{
    MotorWaitResult_t motor_result;

    switch (action_state)
    {
        case MANIP_ACTION_IDLE:
        case MANIP_ACTION_DONE:
        case MANIP_ACTION_ERROR:
            break;

        /*
         * 同时完成两个准备动作：
         * 1. 夹爪旋转到地面夹取方向；
         * 2. 张开夹爪。
         */
        case MANIP_GROUND_TO_TRAY_PREPARE:
            if (!command_carousel_us(
                    active_pickup_carousel_us))
            {
                record_pending_fail(
                    MANIP_FAIL_STATE,
                    0U);
                action_fail(now_ms);
                break;
            }

            GripperServo_Open_Large();

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_PREPARE,
                now_ms);
            break;


        case MANIP_GROUND_TO_TRAY_WAIT_PREPARE:
						if (CarouselServo_IsDone() &&GripperServo_IsDone())
            {
                enter_state(
                    MANIP_GROUND_TO_TRAY_MOVE_TO_GROUND,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(MANIP_FAIL_SERVO, 0U);
                action_fail(now_ms);
            }
            break;

        /*
         * 电机5下降到地面夹取高度。
         * 电机6同时向前伸出。
         */
        case MANIP_GROUND_TO_TRAY_MOVE_TO_GROUND:
            prepare_motor_feedback(
                MOTOR_MANIPULATOR_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    active_pickup_gimbal_mm,
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC) ||
                !Manipulator_Slide_MoveMm(
                    active_pickup_slide_mm,
                    MANIP_SLIDE_RPM,
                    MANIP_SLIDE_ACC))
            {
                record_pending_fail(MANIP_FAIL_SEND,
                                    MOTOR_MANIPULATOR_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_GROUND,
                now_ms);
            break;

        case MANIP_GROUND_TO_TRAY_WAIT_GROUND:
            motor_result = wait_motor_position(
                MOTOR_MANIPULATOR_MASK,
                now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                enter_state(
                    MANIP_GROUND_TO_TRAY_CLOSE,
                    now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;

        /* 闭合夹爪。 */
        case MANIP_GROUND_TO_TRAY_CLOSE:
            GripperServo_Close();

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_CLOSE,
                now_ms);
            break;

				case MANIP_GROUND_TO_TRAY_WAIT_CLOSE:
						if (GripperServo_IsDone())
						{
								enter_state(
										MANIP_GROUND_TO_TRAY_RETURN_GIMBAL,
										now_ms);
						}
						else if ((now_ms - state_start_ms) >=
										MANIP_PLACE_MOTOR_TIMEOUT_MS)
						{
								action_fail(now_ms);
						}
						break;

				/*
				* 夹紧物料后，电机5先上升回最高原点。
				*/
				case MANIP_GROUND_TO_TRAY_RETURN_GIMBAL:
						prepare_motor_feedback(MOTOR_GIMBAL_MASK);
				
						motor_start_ms = now_ms;
						last_status_query_ms = now_ms;
				
						if (!Manipulator_Gimbal_MoveMm(
										active_pickup_gimbal_return_mm,
										MANIP_GIMBAL_RPM,
										MANIP_GIMBAL_ACC))
						{
								record_pending_fail(
										MANIP_FAIL_SEND,
										MOTOR_GIMBAL_MASK);
				
								action_fail(now_ms);
								break;
						}
				
						enter_state(
								MANIP_GROUND_TO_TRAY_WAIT_RETURN_GIMBAL,
								now_ms);
						break;

				case MANIP_GROUND_TO_TRAY_WAIT_RETURN_GIMBAL:
						motor_result = wait_motor_position(
								MOTOR_GIMBAL_MASK,
								now_ms);
				
						if (motor_result == MOTOR_REACHED)
						{
								enter_state(
										MANIP_GROUND_TO_TRAY_RETRACT_SLIDE,
										now_ms);
						}
						else if (motor_wait_failed(motor_result))
						{
								action_fail(now_ms);
						}
						break;
						
				/*电机5已经回到最高原点，电机6开始后缩*/
				case MANIP_GROUND_TO_TRAY_RETRACT_SLIDE:
						prepare_motor_feedback(MOTOR_SLIDE_MASK);
				
						motor_start_ms = now_ms;
						last_status_query_ms = now_ms;
				
						if (!Manipulator_Slide_MoveMm(
										active_pickup_slide_return_mm,
										MANIP_SLIDE_RPM,
										MANIP_SLIDE_ACC))
						{
								record_pending_fail(
										MANIP_FAIL_SEND,
										MOTOR_SLIDE_MASK);
				
								action_fail(now_ms);
								break;
						}
				
						enter_state(
								MANIP_GROUND_TO_TRAY_WAIT_RETRACT_SLIDE,
								now_ms);
						break;
						
				case MANIP_GROUND_TO_TRAY_WAIT_RETRACT_SLIDE:
						motor_result = wait_motor_position(
								MOTOR_SLIDE_MASK,
								now_ms);
				
						if (motor_result == MOTOR_REACHED)
						{
								enter_state(
										MANIP_GROUND_TO_TRAY_ROTATE_TO_PLACE,
										now_ms);
						}
						else if (motor_wait_failed(motor_result))
						{
								action_fail(now_ms);
						}
						break;						
				
        /* 夹爪旋转到物料盘放置方向。 */
        case MANIP_GROUND_TO_TRAY_ROTATE_TO_PLACE:
            CarouselServo_ToPlace();

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_ROTATE,
                now_ms);
            break;


        case MANIP_GROUND_TO_TRAY_WAIT_ROTATE:
            if (CarouselServo_IsDone())
            {
                enter_state(
                    MANIP_GROUND_TO_TRAY_MOVE_TO_TRAY,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(MANIP_FAIL_SERVO, 0U);
                action_fail(now_ms);
            }
            break;

        /* 电机5下降到物料盘高度。 */
        case MANIP_GROUND_TO_TRAY_MOVE_TO_TRAY:
            prepare_motor_feedback(MOTOR_GIMBAL_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    MANIP_PICKUP_TRAY_MM,
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC))
            {
                record_pending_fail(MANIP_FAIL_SEND,
                                    MOTOR_GIMBAL_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_TRAY,
                now_ms);
            break;

        case MANIP_GROUND_TO_TRAY_WAIT_TRAY:
            motor_result = wait_motor_position(
                MOTOR_GIMBAL_MASK,
                now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                enter_state(
                    MANIP_GROUND_TO_TRAY_OPEN,
                    now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;

        /* 张开夹爪，将物料释放到物料盘。 */
        case MANIP_GROUND_TO_TRAY_OPEN:
            GripperServo_Open_Small();

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_OPEN,
                now_ms);
            break;

				case MANIP_GROUND_TO_TRAY_WAIT_OPEN:
						if (GripperServo_IsDone())
						{
								enter_state(MANIP_GROUND_TO_TRAY_RETURN_HOME,now_ms);
						}
						else if ((now_ms - state_start_ms) >=MANIP_PLACE_MOTOR_TIMEOUT_MS)
						{
								action_fail(now_ms);
						}
						break;

        /* 电机5从料盘高度返回最高原点。 */
        case MANIP_GROUND_TO_TRAY_RETURN_HOME:
            prepare_motor_feedback(MOTOR_GIMBAL_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    (-MANIP_PICKUP_TRAY_MM),
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC))
            {
                record_pending_fail(MANIP_FAIL_SEND,
                                    MOTOR_GIMBAL_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_HOME,
                now_ms);
            break;

        case MANIP_GROUND_TO_TRAY_WAIT_HOME:
            motor_result = wait_motor_position(
                MOTOR_GIMBAL_MASK,
                now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                enter_state(
                    MANIP_ACTION_DONE,
                    now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;

        default:
            record_pending_fail(MANIP_FAIL_STATE, 0U);
            action_fail(now_ms);
            break;
    }
}

static void update_from_tray(uint32_t now_ms)
{
    MotorWaitResult_t motor_result;

    switch (action_state)
    {
        /* 1. 夹爪旋转到物料托盘方向。 */
        case MANIP_TRAY_TO_GROUND_ROTATE_TO_TRAY:
            CarouselServo_ToPlace();

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_TRAY,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_TRAY:
            if (CarouselServo_IsDone())
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_MOVE_TO_TRAY,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(MANIP_FAIL_SERVO, 0U);
                action_fail(now_ms);
            }
            break;

        /* 2. 电机5从最高原点下降到物料托盘夹取高度。 */
        case MANIP_TRAY_TO_GROUND_MOVE_TO_TRAY:
            prepare_motor_feedback(MOTOR_GIMBAL_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    MANIP_PICKUP_TRAY_MM,
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC))
            {
                record_pending_fail(
                    MANIP_FAIL_SEND,
                    MOTOR_GIMBAL_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_TRAY,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_TRAY:
            motor_result = wait_motor_position(
                MOTOR_GIMBAL_MASK,
                now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_CLOSE,
                    now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;

        /* 3. 闭合夹爪，从物料托盘夹住物料。 */
        case MANIP_TRAY_TO_GROUND_CLOSE:
            GripperServo_Close();

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_CLOSE,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_CLOSE:
            if (GripperServo_IsDone())
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_RETURN_HOME,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(MANIP_FAIL_SERVO, 0U);
                action_fail(now_ms);
            }
            break;

        /* 4. 电机5从物料托盘高度上升回最高原点。 */
        case MANIP_TRAY_TO_GROUND_RETURN_HOME:
            prepare_motor_feedback(MOTOR_GIMBAL_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    (-MANIP_PICKUP_TRAY_MM),
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC))
            {
                record_pending_fail(
                    MANIP_FAIL_SEND,
                    MOTOR_GIMBAL_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_HOME,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_HOME:
            motor_result = wait_motor_position(
                MOTOR_GIMBAL_MASK,
                now_ms);

						if (motor_result == MOTOR_REACHED)
						{
								enter_state(
										MANIP_TRAY_TO_GROUND_ROTATE_TO_GROUND,
										now_ms);
						}
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;
						
        /*
         * 5.1 夹爪旋转舵机单独转到地面夹取方向。
         */
				case MANIP_TRAY_TO_GROUND_ROTATE_TO_GROUND:
						if (!command_carousel_us(
										active_place_carousel_us))
						{
								record_pending_fail(
										MANIP_FAIL_STATE,
										0U);
				
								action_fail(now_ms);
								break;
						}

						enter_state(
								MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_GROUND,
								now_ms);
						break;

        /*
         * 等待夹爪旋转舵机的软件平滑动作完成。
         */
        case MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_GROUND:
            if (CarouselServo_IsDone())
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_MOVE_TO_GROUND,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(
                    MANIP_FAIL_SERVO,
                    0U);
                action_fail(now_ms);
            }
            break;
						
 /*
         * 5.2 舵机到位后，电机5下降、电机6前伸。
         */
        case MANIP_TRAY_TO_GROUND_MOVE_TO_GROUND:
            prepare_motor_feedback(
                MOTOR_MANIPULATOR_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    active_place_gimbal_mm,
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC) ||
                !Manipulator_Slide_MoveMm(
                    active_place_slide_mm,
                    MANIP_SLIDE_RPM,
                    MANIP_SLIDE_ACC))
            {
                record_pending_fail(
                    MANIP_FAIL_SEND,
                    MOTOR_MANIPULATOR_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_GROUND,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_GROUND:
            motor_result = wait_motor_position(
                MOTOR_MANIPULATOR_MASK,
                now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_OPEN,
                    now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                action_fail(now_ms);
            }
            break;

        /* 6. 张开夹爪，将物料释放到地面目标位置。 */
        case MANIP_TRAY_TO_GROUND_OPEN:
            GripperServo_Open_Large();

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_OPEN,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_OPEN:
            if (GripperServo_IsDone())
            {
                enter_state(
                    MANIP_TRAY_TO_GROUND_FINAL_RETURN_GIMBAL,
                    now_ms);
            }
            else if ((now_ms - state_start_ms) >=
                     MANIP_PLACE_MOTOR_TIMEOUT_MS)
            {
                record_pending_fail(MANIP_FAIL_SERVO, 0U);
                action_fail(now_ms);
            }
            break;

				/*
				* 物料释放完成后，电机5先上升回最高原点。
				*/
				case MANIP_TRAY_TO_GROUND_FINAL_RETURN_GIMBAL:
						prepare_motor_feedback(MOTOR_GIMBAL_MASK);
				
						motor_start_ms = now_ms;
						last_status_query_ms = now_ms;
				
						if (!Manipulator_Gimbal_MoveMm(
										active_place_gimbal_return_mm,
										MANIP_GIMBAL_RPM,
										MANIP_GIMBAL_ACC))
						{
								record_pending_fail(
										MANIP_FAIL_SEND,
										MOTOR_GIMBAL_MASK);
				
								action_fail(now_ms);
								break;
						}
				
						enter_state(
								MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETURN_GIMBAL,
								now_ms);
						break;
						
				case MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETURN_GIMBAL:
						motor_result = wait_motor_position(
								MOTOR_GIMBAL_MASK,
								now_ms);
				
						if (motor_result == MOTOR_REACHED)
						{
								enter_state(
										MANIP_TRAY_TO_GROUND_FINAL_RETRACT_SLIDE,
										now_ms);
						}
						else if (motor_wait_failed(motor_result))
						{
								action_fail(now_ms);
						}
						break;				

				/*
				* 电机5到达最高原点后，电机6开始后缩。
				*/
				case MANIP_TRAY_TO_GROUND_FINAL_RETRACT_SLIDE:
						prepare_motor_feedback(MOTOR_SLIDE_MASK);
				
						motor_start_ms = now_ms;
						last_status_query_ms = now_ms;
				
						if (!Manipulator_Slide_MoveMm(
										active_place_slide_return_mm,
										MANIP_SLIDE_RPM,
										MANIP_SLIDE_ACC))
						{
								record_pending_fail(
										MANIP_FAIL_SEND,
										MOTOR_SLIDE_MASK);
				
								action_fail(now_ms);
								break;
						}
						enter_state(
								MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETRACT_SLIDE,
								now_ms);
						break;						

				case MANIP_TRAY_TO_GROUND_WAIT_FINAL_RETRACT_SLIDE:
						motor_result = wait_motor_position(
								MOTOR_SLIDE_MASK,
								now_ms);
				
						if (motor_result == MOTOR_REACHED)
						{
								enter_state(
										MANIP_ACTION_DONE,
										now_ms);
						}
						else if (motor_wait_failed(motor_result))
						{
								action_fail(now_ms);
						}
						break;						
						
        default:
            record_pending_fail(MANIP_FAIL_STATE, 0U);
            action_fail(now_ms);
            break;
    }
}

void ManipulatorAction_Update(uint32_t now_ms)
{
    /* 本帧不推进时也要刷镜像，断点停下看到的值才是最新的。 */
    sync_action_state();

    if ((action_state == MANIP_ACTION_IDLE) ||
        (action_state == MANIP_ACTION_DONE) ||
        (action_state == MANIP_ACTION_ERROR))
    {
        return;
    }

    switch (active_flow)
    {
        case MANIP_FLOW_TO_TRAY:
            update_to_tray(now_ms);
            break;

        case MANIP_FLOW_FROM_TRAY:
            update_from_tray(now_ms);
            break;

        case MANIP_FLOW_NONE:
        default:
            record_pending_fail(MANIP_FAIL_STATE, 0U);
            action_fail(now_ms);
            break;
    }
}

void ManipulatorAction_Abort(void)
{
    Manipulator_StopAll();

    if (ManipulatorAction_IsBusy())
    {
        manip_debug_fail_state = (uint8_t)action_state;
        manip_debug_fail_reason = MANIP_FAIL_ABORT;
        manip_debug_fail_motor = 0U;
        action_state = MANIP_ACTION_ERROR;
    }
}

bool ManipulatorAction_IsBusy(void)
{
    return (action_state != MANIP_ACTION_IDLE) &&
           (action_state != MANIP_ACTION_DONE) &&
           (action_state != MANIP_ACTION_ERROR);
}

bool ManipulatorAction_IsDone(void)
{
    return action_state == MANIP_ACTION_DONE;
}

bool ManipulatorAction_HasError(void)
{
    return action_state == MANIP_ACTION_ERROR;
}

ManipulatorActionState_t ManipulatorAction_GetState(void)
{
    return action_state;
}
