#include "zone_task.h"

#include "k230_align.h"
#include "manipulator.h"
#include "manipulator_action.h"
#include "raw_material_align.h"
#include "robot_config.h"
#include "servo.h"

#include <stddef.h>

/*
 * 步骤类型：
 *   TO_TRAY   外部（原料台 / 工位 / 暂存位）-> 料盘
 *   FROM_TRAY 料盘 -> 外部
 */
typedef enum
{
    ZONE_STEP_TO_TRAY = 0,
    ZONE_STEP_FROM_TRAY
} ZoneStepFlow_t;

typedef struct
{
    ZoneStepFlow_t flow;
    uint8_t        tray_slot;   /* 料盘选择舵机槽位 0..2 */
    ManipPose_t    pose;        /* 外部端位姿（robot_config.h 的 ZONE_*_POSE） */
    uint8_t        align_color; /* 0=该步不做 DETECT 对位；1~6=开始动作前先按该
                                 * 颜色号做一次 DETECT 对位（转夹爪到统一的
                                 * 识别位 POS2 + 张开 -> 滑台前伸到识别位 ->
                                 * 发 detect + 旋转舵机跟踪），识别到该颜色才夹。
                                 * 目前只有原料区填非 0。
                                 * POS2 是起始姿态；对准完成的动态角度会覆盖
                                 * 本步 pose.carousel_deg。 */
    uint8_t        align_place_id; /* 0=该步不做机械臂视觉校准；1~3=开始动作前
                                    * 先按本步固定位姿伸出，再发 place,<位号>
                                    * 做与原料区相同的二维位置闭环。
                                    * 同一区域批次内每个位号只校准一次，之后
                                    * 的步骤直接复用校准结果。
                                    * 目前只用于粗加工区两批与暂存区第一批的
                                    * 1、3 号位；2 号位已由底盘 PLACE-2 对位，
                                    * 暂存区第二批（叠放）保留原固定位姿流程。 */
} ZoneStep_t;

typedef struct
{
    const ZoneStep_t *steps;
    uint8_t           step_count;
    const char       *name;     /* 调试用，串口/Watch 可读 */
} ZoneProgram_t;

/* ---------------------------------------------------------------------------
 * ★ 手工配置点 1：各区域 × 批次的步骤表
 *
 * 标定值全部来自 robot_config.h 的 ZONE_*_POSE，本文件不写机械尺寸数字。
 * 只改步骤顺序/槽位时改这里；只改高度/前伸/方向时改 robot_config.h。
 * ------------------------------------------------------------------------- */

/*
 * 原料区：三次取料都从取料位2（POS2）起步，依次放进料盘 1/2/3。
 *
 * 相机装在夹爪上，从 POS2 + RAW_VISION_SLIDE_START_MM 起始位取像；K230
 * 的切向偏差由旋转舵机消除，径向偏差由滑台消除。对准完成的动态角度和
 * 滑台位置都会保存。滑台退回机械原点后，正式夹取重放这两个目标值。
 *
 * ZONE_RAW_POS1_POSE / ZONE_RAW_POS3_POSE 因此只作为舵机安全边界的标定来源，
 * 不直接出现在原料区步骤表中。
 */
static const ZoneStep_t s_raw_batch0[3] =
{
    { ZONE_STEP_TO_TRAY, 0U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B1_COLOR_1, 0U },
    { ZONE_STEP_TO_TRAY, 1U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B1_COLOR_2, 0U },
    { ZONE_STEP_TO_TRAY, 2U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B1_COLOR_3, 0U }
};

/*
 * 第二批当前写死验证使用另外三种颜色4/5/6，避免再次抓取第一批的1/2/3。
 * 以后接入二维码时，只需把robot_config.h里的K230_RAW_ALIGN_B2_COLOR_n
 * 替换成任务码第三组解析结果，本状态机无需改动。
 */
static const ZoneStep_t s_raw_batch1[3] =
{
    { ZONE_STEP_TO_TRAY, 0U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B2_COLOR_1, 0U },
    { ZONE_STEP_TO_TRAY, 1U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B2_COLOR_2, 0U },
    { ZONE_STEP_TO_TRAY, 2U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B2_COLOR_3, 0U }
};

/*
 * 粗加工区：先把料盘三块放到三个工位，加工完再把三块取回料盘。
 *
 * 六步的 align_color 全是 0，这不是漏做视觉：RobotApp 在进入本区域前只用
 * 固定的 2 号圆环做一次 PLACE 底盘对位。对位完成以后，2 号位直接重放
 * ZONE_ROUGH_POS2_POSE，不移动底盘。
 *
 * 1、3 号位改为「先按现有固定位姿伸出 -> place,<位号> 二维闭环校准机械臂」：
 *   放料（FROM_TRAY）时先空夹爪校准，再退回原点用校准位姿放料；
 *   取回（TO_TRAY）时复用同一位号已经校准好的位姿，不重复校准。
 * 两种流程都不再启动底盘视觉。
 */
static const ZoneStep_t s_rough_batch0[6] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_ROUGH_POS1_POSE, 0U, 1U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_ROUGH_POS2_POSE, 0U, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_ROUGH_POS3_POSE, 0U, 3U },
    { ZONE_STEP_TO_TRAY,   0U, ZONE_ROUGH_POS1_POSE, 0U, 1U },
    { ZONE_STEP_TO_TRAY,   1U, ZONE_ROUGH_POS2_POSE, 0U, 0U },
    { ZONE_STEP_TO_TRAY,   2U, ZONE_ROUGH_POS3_POSE, 0U, 3U }
};

static const ZoneStep_t s_rough_batch1[6] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_ROUGH_POS1_POSE, 0U, 1U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_ROUGH_POS2_POSE, 0U, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_ROUGH_POS3_POSE, 0U, 3U },
    { ZONE_STEP_TO_TRAY,   0U, ZONE_ROUGH_POS1_POSE, 0U, 1U },
    { ZONE_STEP_TO_TRAY,   1U, ZONE_ROUGH_POS2_POSE, 0U, 0U },
    { ZONE_STEP_TO_TRAY,   2U, ZONE_ROUGH_POS3_POSE, 0U, 3U }
};

/*
 * 暂存区第一批：料盘三块放到暂存位第一层。
 * 与粗加工区相同，1、3 号位先用 place,<位号> 校准机械臂；2 号位由底盘
 * PLACE-2 对位后直接重放固定位姿。
 */
static const ZoneStep_t s_storage_batch0[3] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_STORAGE_POS1_POSE, 0U, 1U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_STORAGE_POS2_POSE, 0U, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_STORAGE_POS3_POSE, 0U, 3U }
};

/*
 * 暂存区第二批：叠放到第一层之上，保留原流程（不插入机械臂二维校准）。
 * 这里的 pose 是默认（同序号）占位，实际目标位由
 * s_storage_l2_target[] 在运行时重映射（见 start_step_action）。
 * 第二批的定位由 K230 的 STACK 模式（身份锚定 + 外圆定位）在 mission 层完成。
 */
static const ZoneStep_t s_storage_batch1[3] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_STORAGE_POS1_L2_POSE, 0U, 0U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_STORAGE_POS2_L2_POSE, 0U, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_STORAGE_POS3_L2_POSE, 0U, 0U }
};

/* ★ 手工配置点 2：第二批第 i 块叠到第一批的哪个暂存位。
 * 默认同序号；本版固定数组驱动，接任务码后按颜色填写。 */
static const uint8_t s_storage_l2_target[3] = { 0U, 1U, 2U };

/* 三个暂存位“叠放层”位姿，供 s_storage_l2_target 索引。 */
static const ManipPose_t s_storage_l2_pose[3] =
{
    ZONE_STORAGE_POS1_L2_POSE,
    ZONE_STORAGE_POS2_L2_POSE,
    ZONE_STORAGE_POS3_L2_POSE
};

/* [区域][批次] -> 步骤表。顺序与 ZoneId_t 一致，新增区域在末尾追加。 */
static const ZoneProgram_t s_zone_program[ZONE_ID_COUNT][2] =
{
    /* ZONE_RAW */
    {
        { s_raw_batch0, 3U, "RAW-B1" },
        { s_raw_batch1, 3U, "RAW-B2" }
    },
    /* ZONE_ROUGH */
    {
        { s_rough_batch0, 6U, "ROUGH-B1" },
        { s_rough_batch1, 6U, "ROUGH-B2" }
    },
    /* ZONE_STORAGE */
    {
        { s_storage_batch0, 3U, "STOR-B1" },
        { s_storage_batch1, 3U, "STOR-B2" }
    }
};

/* ------------------------------------------------------------------------- */

volatile static ZoneTaskState_t task_state = ZONE_TASK_IDLE;
static const ZoneProgram_t *active_program = NULL;
static ZoneId_t active_zone = ZONE_RAW;
static uint8_t active_batch = 0U;
static uint8_t task_index = 0U;
static uint32_t state_start_ms = 0U;
/* 原料区本步二维视觉闭环得到的动态夹取位姿。 */
static float raw_aligned_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
static float raw_aligned_slide_mm = RAW_VISION_SLIDE_START_MM;
static bool raw_aligned_pose_valid = false;

/*
 * 粗加工区/第一轮暂存区 1、3 号位的机械臂二维校准结果，按位号 1..3 缓存。
 * 同一批次内同一位置只校准一次：FROM_TRAY 用空夹爪先校准再放料，
 * TO_TRAY 取回时直接复用，避免重复伸出/校准和多余运动。
 * 每次 ZoneTask_Start 清空，不同批次（底盘停靠位可能不同）不串用。
 */
static float ring_aligned_carousel_deg[3] =
{
    K230_ALIGN_CAROUSEL_ANGLE_DEG,
    K230_ALIGN_CAROUSEL_ANGLE_DEG,
    K230_ALIGN_CAROUSEL_ANGLE_DEG
};
static float ring_aligned_slide_mm[3] = { 0.0f, 0.0f, 0.0f };
static bool ring_aligned_pose_valid[3] = { false, false, false };

/* 该步是否需要在动作前做一次 place,<位号> 机械臂校准。 */
static bool ring_align_needed(const ZoneStep_t *step)
{
    return (step->align_place_id >= 1U) &&
           (step->align_place_id <= 3U) &&
           !ring_aligned_pose_valid[step->align_place_id - 1U];
}

/* 1、3 号位校准前的预下降量：与 2 号参考环取像高度一致。 */
static float ring_align_pre_drop_mm(void)
{
    return (active_zone == ZONE_ROUGH) ? ROUGH_VISION_PRE_DROP_MM
                                       : STORAGE_VISION_PRE_DROP_MM;
}

static void reset_ring_pose_cache(void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        ring_aligned_carousel_deg[i] = K230_ALIGN_CAROUSEL_ANGLE_DEG;
        ring_aligned_slide_mm[i] = 0.0f;
        ring_aligned_pose_valid[i] = false;
    }
}

/*
 * 状态镜像：非 static 全局，Keil Watch 直接输变量名即可观察，
 * 不必写「zone_task.c 反斜杠 task_state」这种全限定名。
 * 只读，不参与任何判断，由 sync_task_state() 刷新。
 *
 *   robot_zone_task_state       ZONE_TASK_xxx 当前值
 *   robot_zone_task_step        当前步骤序号（0-based），卡住时看它
 *   robot_zone_task_step_count  本次序列总步数
 *   robot_zone_task_error_step  失败时停在第几步
 */
volatile uint8_t robot_zone_task_state = ZONE_TASK_IDLE;
volatile uint8_t robot_zone_task_step = 0U;
volatile uint8_t robot_zone_task_step_count = 0U;
volatile uint8_t robot_zone_task_error_step = 0U;
volatile uint8_t robot_zone_task_zone = (uint8_t)ZONE_RAW;
volatile uint8_t robot_zone_task_batch = 0U;

/*
 * 把任务状态、步骤序号与区域刷新到 Watch 可见的镜像变量。
 * 在 enter_state() 与 ZoneTask_Update() 每帧各刷一次，因此无论
 * 是状态转换还是 task_index 自增，镜像都不会落后。
 */
static void sync_task_state(void)
{
    robot_zone_task_state = (uint8_t)task_state;
    robot_zone_task_step = task_index;
    robot_zone_task_step_count = (active_program != NULL) ?
                                 active_program->step_count : 0U;
    robot_zone_task_zone = (uint8_t)active_zone;
    robot_zone_task_batch = active_batch;
}

static void enter_state(
    ZoneTaskState_t new_state,
    uint32_t now_ms)
{
    task_state = new_state;
    state_start_ms = now_ms;
    sync_task_state();
}

static void task_fail(uint32_t now_ms)
{
    /* 冻结现场：Watch 里 robot_zone_task_error_step 告诉你是第几步失败的。 */
    robot_zone_task_error_step = task_index;

    ManipulatorAction_Abort();
    K230Align_Abort();   /* 对位中途失败要停 K230 并清事件；未对位时无害 */
    RawMaterialAlign_Abort();
    enter_state(ZONE_TASK_ERROR, now_ms);
}

static bool task_can_start(void)
{
    if ((task_state != ZONE_TASK_IDLE) &&
        (task_state != ZONE_TASK_DONE))
    {
        return false;
    }

    if (ManipulatorAction_IsBusy() ||
        ManipulatorAction_HasError() ||
        ManipulatorAction_SlideJogIsBusy() ||
        K230Align_IsBusy() ||
        RawMaterialAlign_IsBusy())
    {
        return false;
    }

    /*
     * 滑台位置基准必须有效（确知停在原点）：原料区每步都要先前伸到识别位，
     * 这个固定量是相对原点算的。急停或上一轮失败后基准会作废，此时直接
     * 拒绝启动，而不是启动后才发现点动发不出去。
     */
    if (!Manipulator_Slide_AtHome())
    {
        return false;
    }

    return true;
}

/* 启动当前步骤对应的机械臂动作。 */
static bool start_step_action(void)
{
    const ZoneStep_t *step = &active_program->steps[task_index];
    ManipPose_t pose = step->pose;

    if ((active_zone == ZONE_STORAGE) && (active_batch == 1U))
    {
        /* 第二批：叠放到 s_storage_l2_target 指定的第一层暂存位。 */
        pose = s_storage_l2_pose[s_storage_l2_target[task_index]];
    }

    /*
     * 粗加工区/第一轮暂存区的 1、3 号位：用本批次已校准的云台角与齿条
     * 伸出量覆盖固定位姿；gimbal_drop_mm 与 layer 仍取步骤表。
     */
    if ((step->align_place_id >= 1U) &&
        (step->align_place_id <= 3U) &&
        ring_aligned_pose_valid[step->align_place_id - 1U])
    {
        pose.carousel_deg = ring_aligned_carousel_deg[step->align_place_id - 1U];
        pose.slide_extend_mm = ring_aligned_slide_mm[step->align_place_id - 1U];
    }

    if ((active_zone == ZONE_RAW) &&
        (step->align_color != 0U) &&
        raw_aligned_pose_valid)
    {
        /* 二维视觉闭环已经把相机中心对准目标；滑台和云台仍停在
         * 对准位，直接从这里下降抓取，不再回零后重新伸出。 */
        pose.carousel_deg = raw_aligned_carousel_deg;
        pose.slide_extend_mm = raw_aligned_slide_mm;

        if (step->flow == ZONE_STEP_TO_TRAY)
        {
            return ManipulatorAction_StartToTrayFromAlignedPose(&pose);
        }
    }

    if (step->flow == ZONE_STEP_TO_TRAY)
    {
        return ManipulatorAction_StartToTrayAtPose(&pose);
    }

    return ManipulatorAction_StartTrayToGroundAtPose(&pose);
}

void ZoneTask_Init(void)
{
    task_state = ZONE_TASK_IDLE;
    active_program = NULL;
    active_zone = ZONE_RAW;
    active_batch = 0U;
    task_index = 0U;
    state_start_ms = 0U;
    raw_aligned_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
    raw_aligned_slide_mm = RAW_VISION_SLIDE_START_MM;
    raw_aligned_pose_valid = false;
    reset_ring_pose_cache();
}

bool ZoneTask_Start(ZoneId_t zone, uint8_t batch)
{
    if (!task_can_start())
    {
        return false;
    }

    if (((uint8_t)zone >= (uint8_t)ZONE_ID_COUNT) || (batch > 1U))
    {
        return false;
    }

    active_zone = zone;
    active_batch = batch;
    active_program = &s_zone_program[zone][batch];
    task_index = 0U;
    raw_aligned_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
    raw_aligned_slide_mm = RAW_VISION_SLIDE_START_MM;
    raw_aligned_pose_valid = false;
    reset_ring_pose_cache();

    if (active_program->step_count == 0U)
    {
        active_program = NULL;
        return false;
    }

    enter_state(ZONE_TASK_MOVE_TRAY, HAL_GetTick());

    return true;
}

void ZoneTask_Update(uint32_t now_ms)
{
    const ZoneStep_t *step;
    uint8_t ring_index;

    /* 每帧刷新镜像：task_index 自增后 Watch 立刻可见。 */
    sync_task_state();

    switch (task_state)
    {
        case ZONE_TASK_IDLE:
        case ZONE_TASK_DONE:
        case ZONE_TASK_ERROR:
            break;

        case ZONE_TASK_MOVE_TRAY:
            if (active_program == NULL)
            {
                task_fail(now_ms);
                break;
            }

            step = &active_program->steps[task_index];
            TraySelectorServo_GotoSlot(step->tray_slot);

            enter_state(ZONE_TASK_WAIT_TRAY, now_ms);
            break;

        case ZONE_TASK_WAIT_TRAY:
            if ((now_ms - state_start_ms) >=
                MANIP_PLACE_TRAY_WAIT_MS)
            {
                /*
                 * 原料区每块料在夹之前都要重新识别一次颜色，
                 * 所以先做视觉对位；其他区域该字段为 0，直接开始动作。
                 * 识别统一在取料位2姿态下做，相机装在夹爪上，所以先转云台
                 * 到该姿态并张开夹爪，再把滑台前伸到该位的前伸量，让取像
                 * 姿态和取像距离都对上——顺序见 SLIDE_OUT 那一段。
                 */
                if (active_program->steps[task_index].align_color != 0U)
                {
                    raw_aligned_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
                    raw_aligned_slide_mm = RAW_VISION_SLIDE_START_MM;
                    raw_aligned_pose_valid = false;
                    enter_state(ZONE_TASK_SLIDE_OUT, now_ms);
                }
                else if (ring_align_needed(
                             &active_program->steps[task_index]))
                {
                    /*
                     * 粗加工区/第一轮暂存区 1、3 号位首次使用：先按该位
                     * 固定位姿伸出，再 place,<位号> 二维校准机械臂。
                     * 已校准过的位号直接复用，走正常的 START_ACTION。
                     */
                    enter_state(ZONE_TASK_RING_OUT, now_ms);
                }
                else
                {
                    enter_state(ZONE_TASK_START_ACTION, now_ms);
                }
            }
            break;

        case ZONE_TASK_ALIGN:
            if (active_program == NULL)
            {
                task_fail(now_ms);
                break;
            }

            step = &active_program->steps[task_index];

            /*
             * 云台姿态与滑台前伸都已经在 SLIDE_OUT 那一段做完了
             * （ManipulatorAction_SlideJogToRawAlignStart：先转云台到取料位2
             * 并张开，再把滑台前伸到识别位），这里只负责发 detect,<颜色号>
             * 并等识别到该颜色、微调到偏差容差内，才允许夹取。
             */
            if (!RawMaterialAlign_Start(step->align_color, now_ms))
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_ALIGN, now_ms);
            break;

        case ZONE_TASK_WAIT_ALIGN:
            if (RawMaterialAlign_HasError())
            {
                task_fail(now_ms);
            }
            else if (RawMaterialAlign_IsDone())
            {
                /*
                 * 这里保存的是“相机对准瞬间”的完整二维机械位姿。随后滑台
                 * 必须先退回 HOME，正式夹取动作才能沿用原有的相对位移模型；
                 * StartToTrayAtPose 会从 HOME 重新走到这组角度/伸出量，因而
                 * 抓取中心与视觉中心保持一致。
                 */
                raw_aligned_carousel_deg =
                    RawMaterialAlign_GetCarouselDeg();
                raw_aligned_slide_mm =
                    RawMaterialAlign_GetSlideMm();
                raw_aligned_pose_valid = true;
                /*
                 * 对位完成时滑台已经停在相机验证过的伸出量上，云台也已经
                 * 在对准角度、夹爪保持张开。原料区不再先退回原点再重新
                 * 伸出，直接进入抓取动作，由 StartToTrayFromAlignedPose
                 * 从当前姿态下降电机5。
                 */
                enter_state(ZONE_TASK_START_ACTION, now_ms);
            }
            break;

        /*
         * 识别前：先把夹爪转到识别姿态（取料位2）并张开，再把滑台从原点
         * 前伸到识别位。顺序固定为「云台先转、滑台后伸」：夹爪在缩回状态
         * 下回转，比伸出滑台后再转更不容易和外围干涉；两件都就位后才
         * 发 detect，所以这段纯属取像准备，识别结果不影响它。
         */
        case ZONE_TASK_SLIDE_OUT:
            if (!ManipulatorAction_SlideJogToRawAlignStart())
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_SLIDE_OUT, now_ms);
            break;

        case ZONE_TASK_WAIT_SLIDE_OUT:
            if (ManipulatorAction_SlideJogHasError())
            {
                task_fail(now_ms);
            }
            else if (ManipulatorAction_SlideJogIsDone())
            {
                enter_state(ZONE_TASK_ALIGN, now_ms);
            }
            break;

        /* 识别后：滑台从识别位退回原点。 */
        case ZONE_TASK_SLIDE_HOME:
            if (!ManipulatorAction_SlideJogHomeStart())
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_SLIDE_HOME, now_ms);
            break;

        case ZONE_TASK_WAIT_SLIDE_HOME:
            if (ManipulatorAction_SlideJogHasError())
            {
                task_fail(now_ms);
            }
            else if (ManipulatorAction_SlideJogIsDone())
            {
                enter_state(ZONE_TASK_START_ACTION, now_ms);
            }
            break;

        /*
         * 粗加工区/第一轮暂存区 1、3 号位校准：先按该位现有固定位姿伸出
         * （云台角 + 齿条长度 + 与 2 号参考环相同的预下降），让相机看到
         * 目标圆环；再用 place,<位号> 做与原料区相同的二维位置闭环。
         */
        case ZONE_TASK_RING_OUT:
            step = &active_program->steps[task_index];

            if (!ManipulatorAction_SlideJogToRingAlignStart(
                    step->pose.carousel_deg,
                    step->pose.slide_extend_mm,
                    ring_align_pre_drop_mm()))
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_RING_OUT, now_ms);
            break;

        case ZONE_TASK_WAIT_RING_OUT:
            if (ManipulatorAction_SlideJogHasError())
            {
                task_fail(now_ms);
            }
            else if (ManipulatorAction_SlideJogIsDone())
            {
                enter_state(ZONE_TASK_RING_ALIGN, now_ms);
            }
            break;

        case ZONE_TASK_RING_ALIGN:
            step = &active_program->steps[task_index];

            if (!RawMaterialAlign_StartPlace(step->align_place_id,
                                             step->pose.carousel_deg,
                                             now_ms))
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_RING_ALIGN, now_ms);
            break;

        case ZONE_TASK_WAIT_RING_ALIGN:
            if (RawMaterialAlign_HasError())
            {
                task_fail(now_ms);
            }
            else if (RawMaterialAlign_IsDone())
            {
                step = &active_program->steps[task_index];
                ring_index = (uint8_t)(step->align_place_id - 1U);

                /*
                 * 保存“相机对准瞬间”的云台角与齿条长度。滑台先抬回最高点、
                 * 退回原点，随后 START_ACTION 用这组值执行放置/取回：
                 * 放料时夹爪已经拿着料，不能停在校准位直接放。
                 */
                ring_aligned_carousel_deg[ring_index] =
                    RawMaterialAlign_GetCarouselDeg();
                ring_aligned_slide_mm[ring_index] =
                    RawMaterialAlign_GetSlideMm();
                ring_aligned_pose_valid[ring_index] = true;

                enter_state(ZONE_TASK_SLIDE_HOME, now_ms);
            }
            break;

        case ZONE_TASK_START_ACTION:
            if ((active_program == NULL) ||
                !start_step_action())
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_ACTION, now_ms);
            break;

        case ZONE_TASK_WAIT_ACTION:
            if (ManipulatorAction_HasError())
            {
                task_fail(now_ms);
            }
            else if (ManipulatorAction_IsDone())
            {
                if ((uint8_t)(task_index + 1U) <
                    active_program->step_count)
                {
                    task_index++;

                    enter_state(ZONE_TASK_MOVE_TRAY, now_ms);
                }
                else
                {
                    enter_state(ZONE_TASK_DONE, now_ms);
                }
            }
            break;

        default:
            task_fail(now_ms);
            break;
    }
}

void ZoneTask_Abort(void)
{
    bool was_busy = ZoneTask_IsBusy();

    ManipulatorAction_Abort();
    K230Align_Abort();
    RawMaterialAlign_Abort();

    if (was_busy)
    {
        task_state = ZONE_TASK_ERROR;
    }
}

bool ZoneTask_IsBusy(void)
{
    return (task_state != ZONE_TASK_IDLE) &&
           (task_state != ZONE_TASK_DONE) &&
           (task_state != ZONE_TASK_ERROR);
}

bool ZoneTask_IsDone(void)
{
    return task_state == ZONE_TASK_DONE;
}

bool ZoneTask_HasError(void)
{
    return task_state == ZONE_TASK_ERROR;
}

uint8_t ZoneTask_GetStep(void)
{
    return task_index;
}

uint8_t ZoneTask_GetStepCount(void)
{
    if (active_program == NULL)
    {
        return 0U;
    }

    return active_program->step_count;
}
