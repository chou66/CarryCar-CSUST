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
 *       -> DOCKING：机械臂确认收回后，车体 +X 固定靠近 40 mm，并在同一
 *                    窗口启动滑台前伸到原料识别位（两者并行）
 *       -> DOCK_SETTLE：等待底盘停稳
 *       -> WAIT_PRESLIDE：等并行的滑台前伸到位（通常已提前完成）
 *       -> START/RUN_ZONE：启动 ZoneTask，完成三次“识别—抓取—放料盘”
 *       -> UNDOCKING：再次确认机械臂在原点，车体 -X 固定退回 40 mm
 *       -> UNDOCK_SETTLE
 *       -> DONE：RobotApp 再用原有航向 PD 校正，然后规划下一条边
 *
 * 为什么仍保留固定底盘停靠：它只在整批任务前后各运动一次，位移可逆；三块
 * 物料之间的跟踪全部交给云台和滑台，避免逐块横移误差累积。
 *
 * 停靠与滑台前伸为什么可以同时进行：RAW_VISION_SLIDE_START_MM 已经把车体
 * 靠近的 RAW_DOCK_BODY_X_M 从齿条伸出量里扣除，两项在世界坐标中互相抵消。
 * 同时运动时相机相对圆盘的径向位置从 0 单调走向旧版 115 mm 取像位，
 * 不会越过已经跑通的取像距离；除此之外的任何“机械臂动作未结束/滑台不在
 * 原点”仍然禁止移动底盘。
 *
 * 调用/更新关系：RobotApp 每个主循环先更新 ManipulatorAction、
 * RawMaterialAlign、ZoneTask，再更新本模块。因此本模块只观察和启动子任务，
 * 不重复调用 ZoneTask_Update，防止同一状态机在一帧内推进两次。
 * =============================================================================
 */

static Chassis *s_chassis = NULL;
static RawStationTaskState_t s_state = RAW_STATION_IDLE;
static uint8_t s_batch = 0U;
static uint32_t s_deadline_ms = 0U;

volatile uint8_t robot_raw_station_state = RAW_STATION_IDLE;
volatile uint8_t robot_raw_station_error = RAW_STATION_ERROR_NONE;
volatile uint8_t robot_raw_station_batch = 0U;
volatile uint8_t robot_raw_station_docked = 0U;

static void enter_state(RawStationTaskState_t state)
{
    s_state = state;
    robot_raw_station_state = (uint8_t)state;
}

static bool manipulator_safe_for_chassis(void)
{
    /*
     * “主动作不忙”还不够：滑台点动可能正在视觉伸缩；必须同时确认物理
     * 基准明确位于 HOME，才能保证靠近/撤离时夹爪不会扫到圆盘或车身。
     */
    return Manipulator_Slide_AtHome() &&
           !ManipulatorAction_IsBusy() &&
           !ManipulatorAction_SlideJogIsBusy();
}

static void task_fail(uint8_t error)
{
    /*
     * 失败后立即停止底盘并中止仍在运行的子任务。错误状态保持到重新启动，
     * RobotApp 会把它提升为整车任务错误，不在这里擅自尝试恢复机械原点。
     */
    robot_raw_station_error = error;

    if (s_chassis != NULL)
    {
        chassis_motion_stop(s_chassis);
    }

    /*
     * 停靠平移窗口里可能还有正在进行的滑台前伸：它不是 ZoneTask 的子任务，
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
    s_deadline_ms = 0U;
    robot_raw_station_state = RAW_STATION_IDLE;
    robot_raw_station_error = RAW_STATION_ERROR_NONE;
    robot_raw_station_batch = 0U;
    robot_raw_station_docked = 0U;
}

bool RawStationTask_Start(uint8_t batch, uint32_t now_ms)
{
    /* batch=0/1 分别对应第一批和第二批；任务忙或子任务未退出时拒绝重入。 */
    if ((s_chassis == NULL) || (batch > 1U) ||
        RawStationTask_IsBusy() || ZoneTask_IsBusy() ||
        RawMaterialAlign_IsBusy())
    {
        return false;
    }

    if (!manipulator_safe_for_chassis())
    {
        return false;
    }

    s_batch = batch;
    s_deadline_ms = 0U;
    robot_raw_station_error = RAW_STATION_ERROR_NONE;
    robot_raw_station_batch = batch;
    robot_raw_station_docked = 0U;

#if RAW_DOCK_ENABLE
    /*
     * 视觉识别前必须先把齿条伸到识别位。这里把它和车体 +X 停靠平移放到
     * 同一个窗口一起启动：滑台只动电机6，底盘只动四个轮电机，两者互不
     * 抢轴；相机相对圆盘的径向位置由“车体靠近量 + 齿条伸出量”共同决定，
     * 而 RAW_VISION_SLIDE_START_MM 已扣掉 RAW_DOCK_BODY_X_M，因此并行
     * 过程中始终不超过旧版直接伸到 115 mm 的取像位。
     *
     * 启动失败不在这里判错：到站后 ZoneTask 的 SLIDE_OUT 仍会按原流程
     * 重新前伸，只是退回串行、多花一点时间。
     */
    (void)ManipulatorAction_SlideJogToRawAlignStart();

    chassis_motion_move_x(s_chassis,
                          RAW_DOCK_BODY_X_M,
                          RAW_DOCK_SPEED_MPS,
                          now_ms);
    enter_state(RAW_STATION_DOCKING);
#else
    enter_state(RAW_STATION_START_ZONE);
#endif
    return true;
}

void RawStationTask_Update(uint32_t now_ms)
{
    switch (s_state)
    {
        case RAW_STATION_IDLE:
        case RAW_STATION_DONE:
        case RAW_STATION_ERROR:
            break;

        case RAW_STATION_DOCKING:
            if (chassis_motion_failed(s_chassis))
            {
                task_fail(RAW_STATION_ERROR_DOCK);
            }
            else if (chassis_motion_done(s_chassis))
            {
                robot_raw_station_docked = 1U;
                s_deadline_ms = now_ms + RAW_DOCK_SETTLE_MS;
                enter_state(RAW_STATION_DOCK_SETTLE);
            }
            break;

        case RAW_STATION_DOCK_SETTLE:
            if ((int32_t)(now_ms - s_deadline_ms) >= 0)
            {
                enter_state(RAW_STATION_WAIT_PRESLIDE);
            }
            break;

        /*
         * 停靠窗口里并行启动的滑台前伸可能比底盘停靠慢（云台预转的
         * 舵机稳定时间占大头），这里只等它收尾；已经到位时本状态一帧通过。
         * 失败则必须报错：滑台半路停住后位置基准作废，继续跑 ZoneTask
         * 只会在 SLIDE_OUT 处再失败一次，不如在这里冻结现场。
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
                 * ZoneTask 的每次抓取理论上都会把滑台送回原点，但撤离前仍
                 * 再验一次，不能仅凭“子任务 DONE”就假设机构一定安全。
                 */
                if (!manipulator_safe_for_chassis())
                {
                    task_fail(RAW_STATION_ERROR_HOME);
                    break;
                }

#if RAW_DOCK_ENABLE
                chassis_motion_move_x(s_chassis,
                                      -RAW_DOCK_BODY_X_M,
                                      RAW_DOCK_SPEED_MPS,
                                      now_ms);
                enter_state(RAW_STATION_UNDOCKING);
#else
                enter_state(RAW_STATION_DONE);
#endif
            }
            break;

        case RAW_STATION_UNDOCKING:
            if (chassis_motion_failed(s_chassis))
            {
                task_fail(RAW_STATION_ERROR_UNDOCK);
            }
            else if (chassis_motion_done(s_chassis))
            {
                robot_raw_station_docked = 0U;
                s_deadline_ms = now_ms + RAW_DOCK_SETTLE_MS;
                enter_state(RAW_STATION_UNDOCK_SETTLE);
            }
            break;

        case RAW_STATION_UNDOCK_SETTLE:
            if ((int32_t)(now_ms - s_deadline_ms) >= 0)
            {
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

uint8_t RawStationTask_GetError(void)
{
    return robot_raw_station_error;
}
