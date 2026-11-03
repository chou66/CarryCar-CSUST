#include "raw_station_task.h"

#include "chassis_motion.h"
#include "manipulator.h"
#include "manipulator_action.h"
#include "raw_material_align.h"
#include "robot_config.h"
#include "zone_task.h"

/*
 * =============================================================================
 * 原料站事务层——状态流程与职责边界
 * =============================================================================
 *
 * RobotApp 只知道“启动某批原料任务、等待完成或失败”。本模块把原料区所有
 * 特殊流程收在一起，避免主任务状态机继续膨胀：
 *
 *   N8航向已对准
 *       -> 滑台前伸到原料识别位（车体不再做停靠平移）
 *       -> WAIT_PRESLIDE：等预伸到位
 *       -> START/RUN_ZONE：启动 ZoneTask，完成三次“识别—抓取—放料盘”
 *       -> DONE：RobotApp 再用原有航向 PD 校正，然后规划下一条边
 *
 * 为什么不再做底盘停靠：原先 40 mm 的车体靠近量已经并入
 * RAW_VISION_SLIDE_START_MM，由滑台伸出量承担；相机相对圆盘的世界位置
 * 不变，但整批任务前后少两次底盘起停，也少一次航向漂移来源。
 *
 * 调用/更新关系：RobotApp 每个主循环先更新 ManipulatorAction、
 * RawMaterialAlign、ZoneTask，再更新本模块。因此本模块只观察和启动子任务，
 * 不重复调用 ZoneTask_Update，防止同一状态机在一帧内推进两次。
 * =============================================================================
 */

static Chassis *s_chassis = NULL;
static RawStationTaskState_t s_state = RAW_STATION_IDLE;
static uint8_t s_batch = 0U;

volatile uint8_t robot_raw_station_state = RAW_STATION_IDLE;
volatile uint8_t robot_raw_station_error = RAW_STATION_ERROR_NONE;
volatile uint8_t robot_raw_station_batch = 0U;
/* 保留为 Watch 兼容变量：原料区取消底盘停靠后恒为 0。 */
volatile uint8_t robot_raw_station_docked = 0U;

static void enter_state(RawStationTaskState_t state)
{
    s_state = state;
    robot_raw_station_state = (uint8_t)state;
}

static bool manipulator_at_home_and_idle(void)
{
    /*
     * “主动作不忙”还不够：滑台点动可能正在视觉伸缩；必须同时确认物理
     * 基准明确位于 HOME，才能开始预伸，也才能保证任务结束后车体离开时
     * 夹爪不会扫到圆盘或车身。
     */
    return Manipulator_Slide_AtHome() &&
           !ManipulatorAction_IsBusy() &&
           !ManipulatorAction_SlideJogIsBusy();
}

static void task_fail(uint8_t error)
{
    /*
     * 失败后立即停止底盘（防止上层遗留的底盘命令）并中止仍在运行的子任务。
     * 错误状态保持到重新启动，RobotApp 会把它提升为整车任务错误，
     * 不在这里擅自尝试恢复机械原点。
     */
    robot_raw_station_error = error;

    if (s_chassis != NULL)
    {
        chassis_motion_stop(s_chassis);
    }

    /*
     * 到站预伸不是 ZoneTask 的子任务，可能仍在运行，
     * 必须在这里单独收掉，否则错误状态下车还在走、齿条还在伸。
     */
    if (ManipulatorAction_SlideJogIsBusy())
    {
        ManipulatorAction_Abort();
    }

    if (ZoneTask_IsBusy())
    {
        ZoneTask_Abort();
    }
    RawMaterialAlign_Abort();
    enter_state(RAW_STATION_ERROR);
}

void RawStationTask_Init(Chassis *chassis)
{
    s_chassis = chassis;
    s_state = RAW_STATION_IDLE;
    s_batch = 0U;
    robot_raw_station_state = RAW_STATION_IDLE;
    robot_raw_station_error = RAW_STATION_ERROR_NONE;
    robot_raw_station_batch = 0U;
    robot_raw_station_docked = 0U;
}

bool RawStationTask_Start(uint8_t batch, uint32_t now_ms)
{
    (void)now_ms;

    /* batch=0/1 分别对应第一批和第二批；任务忙或子任务未退出时拒绝重入。 */
    if ((s_chassis == NULL) || (batch > 1U) ||
        RawStationTask_IsBusy() || ZoneTask_IsBusy() ||
        RawMaterialAlign_IsBusy())
    {
        return false;
    }

    if (!manipulator_at_home_and_idle())
    {
        return false;
    }

    s_batch = batch;
    robot_raw_station_error = RAW_STATION_ERROR_NONE;
    robot_raw_station_batch = batch;
    robot_raw_station_docked = 0U;

    /*
     * 车体不再停靠，只把齿条伸到原料识别位。RAW_VISION_SLIDE_START_MM
     * 已经包含原先 40 mm 的车体靠近量，因此相机相对圆盘的世界位置与
     * 旧版停靠流程一致。
     *
     * 启动失败不在这里判错：进入 ZoneTask 后 SLIDE_OUT 仍会按原流程
     * 重新前伸，只是退回串行、多花一点时间。
     */
    (void)ManipulatorAction_SlideJogToRawAlignStart();
    enter_state(RAW_STATION_WAIT_PRESLIDE);
    return true;
}

void RawStationTask_Update(uint32_t now_ms)
{
    (void)now_ms;

    switch (s_state)
    {
        case RAW_STATION_IDLE:
        case RAW_STATION_DONE:
        case RAW_STATION_ERROR:
            break;

        /*
         * 到站时启动的滑台前伸可能还没收尾（云台预转的舵机稳定时间占
         * 大头），这里只等它完成；启动失败时没有动作在跑，本状态一帧
         * 通过，交给 ZoneTask 的 SLIDE_OUT 按原流程重新前伸。
         * 已经动起来却失败则必须报错：滑台半路停住后位置基准作废，继续
         * 跑 ZoneTask 只会在 SLIDE_OUT 处再失败一次，不如在这里冻结现场。
         */
        case RAW_STATION_WAIT_PRESLIDE:
            if (ManipulatorAction_SlideJogHasError())
            {
                task_fail(RAW_STATION_ERROR_PRESLIDE);
            }
            else if (!ManipulatorAction_SlideJogIsBusy())
            {
                enter_state(RAW_STATION_START_ZONE);
            }
            break;

        case RAW_STATION_START_ZONE:
            if (!ZoneTask_Start(ZONE_RAW, s_batch))
            {
                task_fail(RAW_STATION_ERROR_ZONE_START);
            }
            else
            {
                enter_state(RAW_STATION_RUN_ZONE);
            }
            break;

        case RAW_STATION_RUN_ZONE:
            if (ZoneTask_HasError())
            {
                task_fail(RAW_STATION_ERROR_ZONE);
            }
            else if (ZoneTask_IsDone())
            {
                /*
                 * ZoneTask 的每次抓取理论上都会把滑台送回原点，但离开前仍
                 * 再验一次，不能仅凭“子任务 DONE”就假设机构一定安全。
                 */
                if (!manipulator_at_home_and_idle())
                {
                    task_fail(RAW_STATION_ERROR_HOME);
                    break;
                }

                enter_state(RAW_STATION_DONE);
            }
            break;

        default:
            task_fail(RAW_STATION_ERROR_PARAM);
            break;
    }
}

void RawStationTask_Abort(void)
{
    if (!RawStationTask_IsBusy())
    {
        return;
    }

    if (s_chassis != NULL)
    {
        chassis_motion_stop(s_chassis);
    }
    if (ManipulatorAction_SlideJogIsBusy())
    {
        ManipulatorAction_Abort();
    }
    if (ZoneTask_IsBusy())
    {
        ZoneTask_Abort();
    }
    RawMaterialAlign_Abort();
    robot_raw_station_error = RAW_STATION_ERROR_ABORTED;
    enter_state(RAW_STATION_ERROR);
}

bool RawStationTask_IsBusy(void)
{
    return (s_state != RAW_STATION_IDLE) &&
           (s_state != RAW_STATION_DONE) &&
           (s_state != RAW_STATION_ERROR);
}

bool RawStationTask_IsDone(void)
{
    return s_state == RAW_STATION_DONE;
}

bool RawStationTask_HasError(void)
{
    return s_state == RAW_STATION_ERROR;
}
