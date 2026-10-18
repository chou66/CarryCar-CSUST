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
static float active_pickup_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
static float active_pickup_slide_mm = MANIP_PICKUP_SLIDE_POS2_MM;
static float active_pickup_slide_return_mm = (-MANIP_PICKUP_SLIDE_POS2_MM);
static float active_pickup_gimbal_mm = MANIP_PICKUP_GROUND_MM;
static float active_pickup_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

static float active_place_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
static float active_place_slide_mm = MANIP_PICKUP_SLIDE_POS2_MM;
static float active_place_slide_return_mm = (-MANIP_PICKUP_SLIDE_POS2_MM);
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

/*
 * 滑台点动子状态机（视觉识别前的预伸 / 识别后的退回）。
 * 与主动作各自独立推进，共用同一套到位确认函数；因为两者互斥
 * （点动要求主动作 IDLE/DONE，主动作要求点动不在忙），所以
 * motor_start_ms / last_status_query_ms 可以共用。
 */
static volatile ManipulatorSlideJogState_t jog_state = MANIP_JOG_IDLE;

/* 本次点动的目标是不是原点：决定到位后把基准记成 HOME 还是 ALIGN。 */
static bool jog_to_home = false;

/*
 * 视觉闭环需要的不再只是 HOME/ALIGN 两点，而是已知的滑台绝对位置。
 * 电机仍接收相对位移；本模块只在每次到位确认后提交 target，因此中途
 * 失败或急停会立刻把位置标成未知，禁止继续累计相对位移。
 */
static float jog_current_position_mm = 0.0f;
static float jog_target_position_mm = 0.0f;
static bool jog_position_valid = true;

/*
 * 本次要走的滑台相对位移。start 时记下，等到 ROTATE 阶段结束（云台转完）
 * 才真正发出，所以必须存住。
 */
static float jog_delta_mm = 0.0f;

volatile uint8_t robot_manip_jog_state = (uint8_t)MANIP_JOG_IDLE;
volatile uint8_t manip_debug_jog_fail_reason = MANIP_FAIL_NONE;
volatile float robot_manip_slide_position_mm = 0.0f;
volatile uint8_t robot_manip_slide_position_valid = 1U;

/* 把当前状态与流程方向刷新到 Watch 可见的镜像变量。 */
static void sync_action_state(void)
{
    robot_manip_action_state = (uint8_t)action_state;
    robot_manip_active_flow = (uint8_t)active_flow;
    robot_manip_jog_state = (uint8_t)jog_state;
}

static void enter_jog_state(
    ManipulatorSlideJogState_t new_state,
    uint32_t now_ms)
{
    jog_state = new_state;
    state_start_ms = now_ms;
    robot_manip_jog_state = (uint8_t)new_state;
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

/* 旧枚举 -> 标定角度（与 robot_config.h 的 CAROUSEL_*_DEG 一一对应）。 */
static bool position_to_carousel_deg(
    ManipulatorPickupPosition_t position,
    float *deg_out)
{
    switch (position)
    {
        case MANIP_PICKUP_POSITION_1:
            *deg_out = CAROUSEL_PICKUP_POS1_DEG;
            return true;

        case MANIP_PICKUP_POSITION_2:
            *deg_out = CAROUSEL_PICKUP_POS2_DEG;
            return true;

        case MANIP_PICKUP_POSITION_3:
            *deg_out = CAROUSEL_PICKUP_POS3_DEG;
            return true;

        default:
            return false;
    }
}

/* 旧枚举 -> 该工位的前伸距离（与 robot_config.h 的 SLIDE_POS*_MM 一一对应）。 */
static bool position_to_slide_mm(
    ManipulatorPickupPosition_t position,
    float *mm_out)
{
    switch (position)
    {
        case MANIP_PICKUP_POSITION_1:
            *mm_out = MANIP_PICKUP_SLIDE_POS1_MM;
            return true;

        case MANIP_PICKUP_POSITION_2:
            *mm_out = MANIP_PICKUP_SLIDE_POS2_MM;
            return true;

        case MANIP_PICKUP_POSITION_3:
            *mm_out = MANIP_PICKUP_SLIDE_POS3_MM;
            return true;

        default:
            return false;
    }
}

static bool carousel_angle_in_range(float angle_deg)
{
    return (angle_deg >= 0.0f) &&
           (angle_deg <= CAROUSEL_TRAVEL_DEG);
}

/* 夹爪旋转到指定标定角度；越界算失败，不驱动作动器。 */
static bool command_carousel_angle(float angle_deg)
{
    if (!carousel_angle_in_range(angle_deg))
    {
        return false;
    }

    CarouselServo_SetAngle(angle_deg);
    return true;
}

void ManipulatorAction_Init(void)
{
    action_state = MANIP_ACTION_IDLE;
		active_flow = MANIP_FLOW_NONE;
	
    active_pickup_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
    active_pickup_slide_mm = MANIP_PICKUP_SLIDE_POS2_MM;
    active_pickup_slide_return_mm = (-MANIP_PICKUP_SLIDE_POS2_MM);
    active_pickup_gimbal_mm = MANIP_PICKUP_GROUND_MM;
    active_pickup_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

    active_place_carousel_deg = CAROUSEL_PICKUP_POS2_DEG;
    active_place_slide_mm = MANIP_PICKUP_SLIDE_POS2_MM;
    active_place_slide_return_mm = (-MANIP_PICKUP_SLIDE_POS2_MM);
    active_place_gimbal_mm = MANIP_PICKUP_GROUND_MM;
    active_place_gimbal_return_mm = (-MANIP_PICKUP_GROUND_MM);

    /* 预置/点动子状态机复位。 */
    jog_state = MANIP_JOG_IDLE;
    jog_to_home = false;
    jog_delta_mm = 0.0f;
    jog_current_position_mm = 0.0f;
    jog_target_position_mm = 0.0f;
    jog_position_valid = true;
    robot_manip_jog_state = (uint8_t)MANIP_JOG_IDLE;
    manip_debug_jog_fail_reason = MANIP_FAIL_NONE;
    robot_manip_slide_position_mm = 0.0f;
    robot_manip_slide_position_valid = 1U;

    /*
     * 冷启动假定滑台停在机械原点（本函数只在 main/RobotApp 初始化时调用）。
     * 若在滑台停于别处时调用本函数，基准会被错误地立成 HOME——重立基准
     * 请用 Manipulator_Slide_SetHome()，不要靠本函数。
     */
    Manipulator_Slide_SetHome();

		state_start_ms = 0U;
    motor_start_ms = 0U;
    last_status_query_ms = 0U;

    clear_fail_debug();

}

/*
 * TO_TRAY 两类入口共用的参数校验与准备：
 *   StartToTrayAtPose             —— 从原点出发，云台转位 + 滑台伸出
 *   StartToTrayFromAlignedPose    —— 视觉已对准，直接下降
 *
 * 返回 false 时不得改动任何动作状态；active_pickup_* 只在参数全部合法后
 * 才写入，保证“失败不改变状态”的既有约定。
 */
static bool prepare_to_tray_start(const ManipPose_t *pickup_pose)
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

    /* 与滑台点动互斥：预伸还没到位时不允许启动夹取动作。 */
    if (ManipulatorAction_SlideJogIsBusy())
    {
        return false;
    }

    if (!carousel_angle_in_range(pickup_pose->carousel_deg))
    {
        return false;
    }

    if (pickup_pose->layer > MANIP_MAX_LAYER)
    {
        return false;
    }

    effective_mm = pickup_pose->gimbal_drop_mm +
                   ((float)pickup_pose->layer * MANIP_LAYER_STEP_MM);

    active_pickup_carousel_deg = pickup_pose->carousel_deg;
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

    return true;
}

/*
 * 外部 -> 料盘：把外部端位姿参数化后的启动入口。
 * 参数非法或动作未结束时返回 false，且不改变任何状态。
 */
bool ManipulatorAction_StartToTrayAtPose(const ManipPose_t *pickup_pose)
{
    if (!prepare_to_tray_start(pickup_pose))
    {
        return false;
    }

    action_state =
        MANIP_GROUND_TO_TRAY_PREPARE;
    active_flow = MANIP_FLOW_TO_TRAY;

    return true;
}

/*
 * 原料区专用：视觉已经对准，直接从当前姿态下降抓取。
 *
 * 这里不再动云台和滑台：视觉预置已经把云台转到识别姿态并张开夹爪，
 * RawMaterialAlign 又把云台/滑台调到相机验证过的二维位置。因此本入口
 * 只校验“滑台位置确实等于传入位姿”，然后从直接下降状态开始。
 */
bool ManipulatorAction_StartToTrayFromAlignedPose(const ManipPose_t *pickup_pose)
{
    float current_mm;
    float target_mm;

    if (pickup_pose == NULL)
    {
        return false;
    }

    /* 没有可信的滑台绝对位置时不能走这条捷径。 */
    if (!ManipulatorAction_SlideJogPositionKnown())
    {
        return false;
    }

    current_mm = ManipulatorAction_SlideJogGetPositionMm();
    target_mm = pickup_pose->slide_extend_mm;

    if ((current_mm > (target_mm + 0.5f)) ||
        (current_mm < (target_mm - 0.5f)))
    {
        return false;
    }

    if (!prepare_to_tray_start(pickup_pose))
    {
        return false;
    }

    /*
     * 滑台已经停在识别/对准位。回程量必须按实测当前位置计算，
     * 这样夹取后的 RETRACT_SLIDE 才能准确回到 0 mm。
     */
    active_pickup_slide_mm = current_mm;
    active_pickup_slide_return_mm = -current_mm;

    action_state =
        MANIP_GROUND_TO_TRAY_DIRECT_DESCEND;
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

    if (!position_to_carousel_deg(position, &pose.carousel_deg))
    {
        return false;
    }

    if (!position_to_slide_mm(position, &pose.slide_extend_mm))
    {
        return false;
    }

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
 * 参数非法（空指针 / 舵机角度越界 / 层号超限）或动作未结束时返回 false，
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

    /* 与滑台点动互斥：预伸还没到位时不允许启动放置动作。 */
    if (ManipulatorAction_SlideJogIsBusy())
    {
        return false;
    }

    if (!carousel_angle_in_range(place_pose->carousel_deg))
    {
        return false;
    }

    if (place_pose->layer > MANIP_MAX_LAYER)
    {
        return false;
    }

    effective_mm = place_pose->gimbal_drop_mm +
                   ((float)place_pose->layer * MANIP_LAYER_STEP_MM);

    active_place_carousel_deg = place_pose->carousel_deg;
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

    if (!position_to_carousel_deg(position, &pose.carousel_deg))
    {
        return false;
    }

    if (!position_to_slide_mm(position, &pose.slide_extend_mm))
    {
        return false;
    }

    pose.gimbal_drop_mm = MANIP_PICKUP_GROUND_MM;
    pose.layer = 0U;

    return ManipulatorAction_StartTrayToGroundAtPose(&pose);
}

bool ManipulatorAction_StartTrayToGround(void)
{
    return ManipulatorAction_StartTrayToGroundAt(
        MANIP_PICKUP_POSITION_2);
}

/*
 * 主动作或点动把滑台成功退回机械原点后，统一同步：
 *   - Manipulator 层的 HOME 基准（供 ZoneTask 判断下一段能否启动）；
 *   - 点动模块的绝对位置与有效性（供下一次视觉预置继续累计）。
 * 只在“确实收到到位反馈”后调用；中途失败/急停仍走 UNKNOWN。
 */
static void commit_slide_home_position(void)
{
    Manipulator_Slide_SetHome();

    jog_current_position_mm = 0.0f;
    jog_target_position_mm = 0.0f;
    jog_position_valid = true;
    robot_manip_slide_position_mm = 0.0f;
    robot_manip_slide_position_valid = 1U;
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
            if (!command_carousel_angle(
                    active_pickup_carousel_deg))
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
								/*
								 * 无论本次动作从原点出发还是从视觉对准位
								 * 出发，滑台回程量都等于去程伸出量，因此
								 * 到位后确实回到机械原点，同步位置基准。
								 */
								commit_slide_home_position();
								enter_state(
										MANIP_GROUND_TO_TRAY_ROTATE_TO_PLACE,
										now_ms);
						}
						else if (motor_wait_failed(motor_result))
						{
								action_fail(now_ms);
						}
						break;						
				
        /*
         * 视觉对准后直接抓取：云台/滑台/夹爪已经由预置和视觉闭环摆好，
         * 这里只等舵机就绪，然后仅下降电机5。滑台不再先回零再重伸。
         */
        case MANIP_GROUND_TO_TRAY_DIRECT_DESCEND:
            if (!CarouselServo_IsDone() || !GripperServo_IsDone())
            {
                if ((now_ms - state_start_ms) >=
                    MANIP_PLACE_MOTOR_TIMEOUT_MS)
                {
                    record_pending_fail(MANIP_FAIL_SERVO, 0U);
                    action_fail(now_ms);
                }
                break;
            }

            prepare_motor_feedback(MOTOR_GIMBAL_MASK);

            motor_start_ms = now_ms;
            last_status_query_ms = now_ms;

            if (!Manipulator_Gimbal_MoveMm(
                    active_pickup_gimbal_mm,
                    MANIP_GIMBAL_RPM,
                    MANIP_GIMBAL_ACC))
            {
                record_pending_fail(MANIP_FAIL_SEND,
                                    MOTOR_GIMBAL_MASK);
                action_fail(now_ms);
                break;
            }

            enter_state(
                MANIP_GROUND_TO_TRAY_WAIT_DIRECT_DESCEND,
                now_ms);
            break;

        case MANIP_GROUND_TO_TRAY_WAIT_DIRECT_DESCEND:
            motor_result = wait_motor_position(
                MOTOR_GIMBAL_MASK,
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
            GripperServo_Open_Small();

            enter_state(
                MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_TRAY,
                now_ms);
            break;

        case MANIP_TRAY_TO_GROUND_WAIT_ROTATE_TO_TRAY:
            if (CarouselServo_IsDone() && GripperServo_IsDone())
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
						if (!command_carousel_angle(
										active_place_carousel_deg))
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
            GripperServo_Open_Small();

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

/* ---------------------------------------------------------------------------
 * 视觉预置与位置点动：
 *   通用/原料预置 = 云台先转到识别姿态并张开 -> 滑台到指定绝对位置
 *   视觉闭环点动   = 从已确认的位置走到新的绝对目标，不重复转云台
 *   退回原点       = 从当前已确认位置走回 0 mm
 * ------------------------------------------------------------------------- */

/* 启动公共前置条件：主动作不在跑，且上一次预置/点动已结束。 */
static bool slide_jog_can_start(void)
{
    if ((action_state != MANIP_ACTION_IDLE) &&
        (action_state != MANIP_ACTION_DONE))
    {
        return false;
    }

    return (jog_state == MANIP_JOG_IDLE) ||
           (jog_state == MANIP_JOG_DONE) ||
           (jog_state == MANIP_JOG_ERROR);
}

/*
 * 发滑台位移指令并切进 MOVE。ROTATE 阶段结束、以及不需要预转的退回段
 * 都走这里，保证「准备反馈 -> 记录起始时刻 -> 发指令」三步不重不漏。
 */
static bool slide_jog_send_move(float delta_mm)
{
    uint32_t now_ms = HAL_GetTick();

    if (delta_mm == 0.0f)
    {
        /*
         * 预伸量被标定成 0（滑台本来就该停在识别位）时不用发电机命令，
         * 否则 0 脉冲下等不到新的到位标志，会一直挂到超时。
         */
        if (jog_to_home)
        {
            commit_slide_home_position();
        }
        else
        {
            jog_current_position_mm = jog_target_position_mm;
            robot_manip_slide_position_mm = jog_current_position_mm;
            robot_manip_slide_position_valid = 1U;
            Manipulator_Slide_SetAlign();
        }

        enter_jog_state(MANIP_JOG_DONE, now_ms);
        return true;
    }

    prepare_motor_feedback(MOTOR_SLIDE_MASK);

    motor_start_ms = now_ms;
    last_status_query_ms = now_ms;

    if (!Manipulator_Slide_MoveMm(delta_mm, MANIP_SLIDE_RPM, MANIP_SLIDE_ACC))
    {
        /* 指令没发出去，滑台没动；位置基准仍然可信，留给调用方决定后续。 */
        manip_debug_jog_fail_reason = MANIP_FAIL_SEND;
        enter_jog_state(MANIP_JOG_ERROR, now_ms);
        return false;
    }

    manip_debug_jog_fail_reason = MANIP_FAIL_NONE;

    enter_jog_state(MANIP_JOG_MOVE, now_ms);

    return true;
}

static bool slide_jog_start_target(float target_mm, bool prepare_vision)
{
    uint32_t now_ms = HAL_GetTick();
    float delta_mm;

    if (!slide_jog_can_start() || !jog_position_valid ||
        (target_mm < 0.0f) ||
        (target_mm > MANIP_SLIDE_SOFT_MAX_MM))
    {
        return false;
    }

    jog_target_position_mm = target_mm;
    jog_to_home = (target_mm == 0.0f);
    delta_mm = target_mm - jog_current_position_mm;
    jog_delta_mm = delta_mm;

    if (!prepare_vision)
    {
        /* 视觉闭环点动 / 退回原点都不重复改变云台。 */
        return slide_jog_send_move(delta_mm);
    }

    /*
     * 伸到识别位之前先把云台转到识别姿态并张开夹爪——顺序不能反。
     * 这里只发舵机目标，到位由 ROTATE 状态确认，确认过才发电机指令，
     * 免得夹爪还在转的时候滑台已经伸出去。
     * 目标值与 K230Align 发检测命令前用的是同一个宏，两处必须一致。
     */
    CarouselServo_SetAngle(K230_ALIGN_CAROUSEL_ANGLE_DEG);
    GripperServo_Open_Large();

    manip_debug_jog_fail_reason = MANIP_FAIL_NONE;

    enter_jog_state(MANIP_JOG_ROTATE, now_ms);

    return true;
}

static void update_slide_jog(uint32_t now_ms)
{
    MotorWaitResult_t motor_result;

    switch (jog_state)
    {
        case MANIP_JOG_ROTATE:
            /*
             * 等云台转到位 + 夹爪张开到位。舵机没有位置反馈，只靠平滑
             * 步进的 DONE 判定；超时（同一个 K230_ALIGN_ROTATE_TIMEOUT_MS，
             * 本来就是同一个动作、同一个时间常数）则按到位继续，
             * 与 K230Align 的 ROTATE 处理保持一致。
             */
            if ((CarouselServo_IsDone() && GripperServo_IsDone()) ||
                ((now_ms - state_start_ms) >= K230_ALIGN_ROTATE_TIMEOUT_MS))
            {
                (void)slide_jog_send_move(jog_delta_mm);
            }
            break;

        case MANIP_JOG_MOVE:
            /* 指令已发（start 或 ROTATE 结束时），这里只切到等待态。 */
            enter_jog_state(MANIP_JOG_WAIT, now_ms);
            break;

        case MANIP_JOG_WAIT:
            motor_result = wait_motor_position(MOTOR_SLIDE_MASK, now_ms);

            if (motor_result == MOTOR_REACHED)
            {
                /* 到位即更新位置基准：退回原点 => HOME，预伸 => ALIGN。 */
                if (jog_to_home)
                {
                    commit_slide_home_position();
                }
                else
                {
                    Manipulator_Slide_SetAlign();
                    jog_current_position_mm = jog_target_position_mm;
                    jog_position_valid = true;
                    robot_manip_slide_position_mm = jog_current_position_mm;
                    robot_manip_slide_position_valid = 1U;
                }

                enter_jog_state(MANIP_JOG_DONE, now_ms);
            }
            else if (motor_wait_failed(motor_result))
            {
                /* 失败原因转存到点动自己的现场变量，免得污染主动作的记录。 */
                manip_debug_jog_fail_reason = pending_fail_reason;
                pending_fail_reason = MANIP_FAIL_NONE;
                pending_fail_motor = 0U;

                /* 半路停下的滑台位置不可知，位置基准作废。 */
                Manipulator_StopAll();
                jog_position_valid = false;
                robot_manip_slide_position_valid = 0U;
                enter_jog_state(MANIP_JOG_ERROR, now_ms);
            }
            break;

        default:
            break;
    }
}

bool ManipulatorAction_SlideJogToAlignStart(void)
{
    /* 预伸只能从原点出发，否则按固定量前伸会冲过头。 */
    if (!Manipulator_Slide_AtHome())
    {
        return false;
    }

    return slide_jog_start_target(MANIP_PICKUP_SLIDE_ALIGN_MM, true);
}

bool ManipulatorAction_SlideJogToRoughAlignStart(void)
{
    /*
     * 粗加工标定和正式任务的因果顺序必须一致：
     *   标定时信任“启停区2直达N2”的底盘停靠位，只移动齿条，测出相机
     *   正好看准2号圆环时的绝对长度 ROUGH_SLIDE_POS2_MM；
     *   正式比赛经过多轮行驶后到N2，再固定复现这组相机位姿，让K230的
     *   像素偏差去移动底盘，从而恢复到标定时那个准确位置。
     *
     * 云台仍使用 K230_ALIGN_CAROUSEL_ANGLE_DEG（当前205 deg），因为K230
     * 的 w/a/s/d 到车体坐标映射就是在这个识别朝向下定义的；2号标定只
     * 调齿条长度，1号/3号才联合调各自的云台角和齿条长度。
     */
    if (!Manipulator_Slide_AtHome())
    {
        return false;
    }

    return slide_jog_start_target(ROUGH_SLIDE_POS2_MM, true);
}

bool ManipulatorAction_SlideJogToStorageAlignStart(void)
{
    /*
     * 暂存区与粗加工区采用相同的“固定2号相机位姿 -> K230移动底盘”原则，
     * 但两处圆环到节点的实际距离不同，绝不能继续共用一个齿条长度。
     * 云台识别角仍固定为K230_ALIGN_CAROUSEL_ANGLE_DEG（当前205 deg），
     * 这里只换成暂存区独立标定得到的2号齿条绝对长度。
     */
    if (!Manipulator_Slide_AtHome())
    {
        return false;
    }

    return slide_jog_start_target(STORAGE_SLIDE_POS2_MM, true);
}

bool ManipulatorAction_SlideJogToRawAlignStart(void)
{
    /*
     * 原料区停靠后用补偿过的较短预伸。底盘已经向圆盘靠近 40 mm，滑台
     * 相对旧识别位少伸 40 mm，两项在世界坐标中基本抵消，因此第一次画面
     * 仍落在已经跑通的旧取像距离附近，而不是突然把相机推进圆盘。
     */
    if (!Manipulator_Slide_AtHome() ||
        !jog_position_valid ||
        (jog_current_position_mm != 0.0f))
    {
        return false;
    }

    return slide_jog_start_target(RAW_VISION_SLIDE_START_MM, true);
}

bool ManipulatorAction_SlideJogToPositionStart(float target_mm)
{
    /* 仅供已完成机械预置后的视觉位置环使用，不重复转云台/开夹爪。 */
    if (!jog_position_valid)
    {
        return false;
    }

    return slide_jog_start_target(target_mm, false);
}

bool ManipulatorAction_SlideJogHomeStart(void)
{
    /* 退回量按当前已确认位置计算，不再假定识别位固定为 115 mm。 */
    if (!Manipulator_Slide_AtAlign() || !jog_position_valid)
    {
        return false;
    }

    return slide_jog_start_target(0.0f, false);
}

bool ManipulatorAction_SlideJogRebaseHome(void)
{
    /*
     * 这里只重建“软件坐标”，不会驱动电机寻找机械原点。因此调用方必须先让
     * 操作者确认滑台已经物理缩到底；粗加工 VOFA 标定用 0x81 + 0xA5 二次
     * 确认后才会调用本函数，避免把任意位置误当作 0 mm。
     */
    if (!slide_jog_can_start() || ManipulatorAction_IsBusy())
    {
        return false;
    }

    Manipulator_Slide_SetHome();
    jog_state = MANIP_JOG_IDLE;
    jog_to_home = false;
    jog_delta_mm = 0.0f;
    jog_current_position_mm = 0.0f;
    jog_target_position_mm = 0.0f;
    jog_position_valid = true;
    robot_manip_jog_state = (uint8_t)MANIP_JOG_IDLE;
    manip_debug_jog_fail_reason = MANIP_FAIL_NONE;
    robot_manip_slide_position_mm = 0.0f;
    robot_manip_slide_position_valid = 1U;
    return true;
}

float ManipulatorAction_SlideJogGetPositionMm(void)
{
    return jog_current_position_mm;
}

bool ManipulatorAction_SlideJogPositionKnown(void)
{
    return jog_position_valid;
}

bool ManipulatorAction_SlideJogIsBusy(void)
{
    return (jog_state == MANIP_JOG_ROTATE) ||
           (jog_state == MANIP_JOG_MOVE) ||
           (jog_state == MANIP_JOG_WAIT);
}

bool ManipulatorAction_SlideJogIsDone(void)
{
    return jog_state == MANIP_JOG_DONE;
}

bool ManipulatorAction_SlideJogHasError(void)
{
    return jog_state == MANIP_JOG_ERROR;
}

void ManipulatorAction_Update(uint32_t now_ms)
{
    /* 本帧不推进时也要刷镜像，断点停下看到的值才是最新的。 */
    sync_action_state();

    /* 点动多在主动作 IDLE 时进行，必须排在下面那个提前 return 之前。 */
    update_slide_jog(now_ms);

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
    /* 中途截停后滑台位置不可知，StopAll 会同时把位置基准置为 UNKNOWN。 */
    Manipulator_StopAll();
    jog_position_valid = false;
    robot_manip_slide_position_valid = 0U;

    /*
     * ROTATE 也要一起收掉：此时滑台还没动，但 StopAll 已经把位置基准置成
     * 不可信（急停语义下不该假设滑台没被碰过），留着 ROTATE 会让后续任何
     * start 都被 slide_jog_can_start() 拒掉，表现为整机卡死。
     */
    if ((jog_state == MANIP_JOG_ROTATE) ||
        (jog_state == MANIP_JOG_MOVE) ||
        (jog_state == MANIP_JOG_WAIT))
    {
        manip_debug_jog_fail_reason = MANIP_FAIL_ABORT;
        jog_state = MANIP_JOG_ERROR;
        robot_manip_jog_state = (uint8_t)MANIP_JOG_ERROR;
    }

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
