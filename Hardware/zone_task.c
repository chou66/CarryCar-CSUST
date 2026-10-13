#include "zone_task.h"

#include "k230_align.h"
#include "manipulator_action.h"
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
    uint8_t        align_color; /* 0=该步不做视觉对位；1~6=开始动作前先按该
                                 * 颜色号做一次 DETECT 对位（转夹爪到统一的
                                 * 识别位 POS2 + 张开 + 发 detect + 微调），
                                 * 识别到该颜色才夹。目前只有原料区填非 0。
                                 * 注意对位姿态与 pose.carousel_us 无关：
                                 * 识别统一 POS2，夹取才用本步 pose。 */
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

/* 原料区：原料台三个方向取料，依次放进料盘 1/2/3。 */
static const ZoneStep_t s_raw_batch0[3] =
{
    { ZONE_STEP_TO_TRAY, 0U, ZONE_RAW_POS1_POSE, K230_RAW_ALIGN_B1_COLOR_1 },
    { ZONE_STEP_TO_TRAY, 1U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B1_COLOR_2 },
    { ZONE_STEP_TO_TRAY, 2U, ZONE_RAW_POS3_POSE, K230_RAW_ALIGN_B1_COLOR_3 }
};

static const ZoneStep_t s_raw_batch1[3] =
{
    { ZONE_STEP_TO_TRAY, 0U, ZONE_RAW_POS1_POSE, K230_RAW_ALIGN_B2_COLOR_1 },
    { ZONE_STEP_TO_TRAY, 1U, ZONE_RAW_POS2_POSE, K230_RAW_ALIGN_B2_COLOR_2 },
    { ZONE_STEP_TO_TRAY, 2U, ZONE_RAW_POS3_POSE, K230_RAW_ALIGN_B2_COLOR_3 }
};

/* 粗加工区：先把料盘三块放到三个工位，加工完再把三块取回料盘。 */
static const ZoneStep_t s_rough_batch0[6] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_ROUGH_POS1_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_ROUGH_POS2_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_ROUGH_POS3_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   0U, ZONE_ROUGH_POS1_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   1U, ZONE_ROUGH_POS2_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   2U, ZONE_ROUGH_POS3_POSE, 0U }
};

static const ZoneStep_t s_rough_batch1[6] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_ROUGH_POS1_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_ROUGH_POS2_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_ROUGH_POS3_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   0U, ZONE_ROUGH_POS1_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   1U, ZONE_ROUGH_POS2_POSE, 0U },
    { ZONE_STEP_TO_TRAY,   2U, ZONE_ROUGH_POS3_POSE, 0U }
};

/* 暂存区第一批：料盘三块放到暂存位第一层。 */
static const ZoneStep_t s_storage_batch0[3] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_STORAGE_POS1_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_STORAGE_POS2_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_STORAGE_POS3_POSE, 0U }
};

/*
 * 暂存区第二批：叠放到第一层之上。
 * 这里的 pose 是默认（同序号）占位，实际目标位由
 * s_storage_l2_target[] 在运行时重映射（见 start_step_action）。
 */
static const ZoneStep_t s_storage_batch1[3] =
{
    { ZONE_STEP_FROM_TRAY, 0U, ZONE_STORAGE_POS1_L2_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 1U, ZONE_STORAGE_POS2_L2_POSE, 0U },
    { ZONE_STEP_FROM_TRAY, 2U, ZONE_STORAGE_POS3_L2_POSE, 0U }
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
        K230Align_IsBusy())
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
                 */
                if (active_program->steps[task_index].align_color != 0U)
                {
                    enter_state(ZONE_TASK_ALIGN, now_ms);
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
             * 三轮识别统一转到取料位2（K230_ALIGN_CAROUSEL_POS_US）并张开，
             * 再发 detect,<颜色号>；等识别到该颜色并微调到偏差容差内，
             * 才允许夹取。夹取那一下才用本步自己的 pose（POS1/POS2/POS3）。
             */
            if (!K230Align_Start(K230_ALIGN_MODE_DETECT,
                                 step->align_color,
                                 K230_ALIGN_CAROUSEL_POS_US,
                                 now_ms))
            {
                task_fail(now_ms);
                break;
            }

            enter_state(ZONE_TASK_WAIT_ALIGN, now_ms);
            break;

        case ZONE_TASK_WAIT_ALIGN:
            if (K230Align_HasError())
            {
                task_fail(now_ms);
            }
            else if (K230Align_IsDone())
            {
                enter_state(ZONE_TASK_START_ACTION, now_ms);
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
