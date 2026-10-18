#include "material_load_task.h"

#include "manipulator.h"
#include "manipulator_action.h"
#include "robot_config.h"
#include "servo.h"

static MaterialTaskMode_t task_mode = MATERIAL_TASK_MODE_NONE;
static MaterialTaskState_t task_state = MATERIAL_TASK_IDLE;
static ManipulatorPickupSource_t pickup_source =
    MANIP_PICKUP_SOURCE_GROUND;
static uint8_t task_index = 0U;
static uint32_t state_start_ms = 0U;

/*
 * LOAD:   position 1/2/3 -> tray slot 1/2/3.
 * UNLOAD: tray slot 1/2/3 -> position 1/2/3.
 */
static const ManipulatorPickupPosition_t position_plan[
    TRAY_SELECTOR_SLOT_COUNT] =
{
    MANIP_PICKUP_POSITION_1,
    MANIP_PICKUP_POSITION_2,
    MANIP_PICKUP_POSITION_3
};

static void enter_state(
    MaterialTaskState_t new_state,
    uint32_t now_ms)
{
    task_state = new_state;
    state_start_ms = now_ms;
}

static void task_fail(uint32_t now_ms)
{
    ManipulatorAction_Abort();
    enter_state(MATERIAL_TASK_ERROR, now_ms);
}

static bool task_can_start(void)
{
    if ((task_state != MATERIAL_TASK_IDLE) &&
        (task_state != MATERIAL_TASK_DONE))
    {
        return false;
    }

    if (ManipulatorAction_IsBusy() ||
        ManipulatorAction_HasError() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        return false;
    }

    /* 动作组假定滑台从机械原点出发；基准无效（急停/上次失败）时拒绝启动。 */
    if (!Manipulator_Slide_AtHome())
    {
        return false;
    }

    return true;
}

void MaterialLoadTask_Init(void)
{
    task_mode = MATERIAL_TASK_MODE_NONE;
    task_state = MATERIAL_TASK_IDLE;
    pickup_source = MANIP_PICKUP_SOURCE_GROUND;
    task_index = 0U;
    state_start_ms = 0U;
}

bool MaterialLoadTask_Start(
    ManipulatorPickupSource_t source)
{
    if (!task_can_start())
    {
        return false;
    }

    switch (source)
    {
        case MANIP_PICKUP_SOURCE_GROUND:
        case MANIP_PICKUP_SOURCE_TABLE:
            break;

        default:
            return false;
    }

    task_mode = MATERIAL_TASK_MODE_LOAD;
    pickup_source = source;
    task_index = 0U;

    enter_state(
        MATERIAL_TASK_MOVE_TRAY,
        HAL_GetTick());

    return true;
}

bool MaterialUnloadTask_Start(void)
{
    if (!task_can_start())
    {
        return false;
    }

    task_mode = MATERIAL_TASK_MODE_UNLOAD;
    task_index = 0U;

    enter_state(
        MATERIAL_TASK_MOVE_TRAY,
        HAL_GetTick());

    return true;
}

void MaterialLoadTask_Update(uint32_t now_ms)
{
    switch (task_state)
    {
        case MATERIAL_TASK_IDLE:
        case MATERIAL_TASK_DONE:
        case MATERIAL_TASK_ERROR:
            break;

        case MATERIAL_TASK_MOVE_TRAY:
            TraySelectorServo_GotoSlot(task_index);

            enter_state(
                MATERIAL_TASK_WAIT_TRAY,
                now_ms);
            break;

        case MATERIAL_TASK_WAIT_TRAY:
            if ((now_ms - state_start_ms) >=
                MANIP_PLACE_TRAY_WAIT_MS)
            {
                enter_state(
                    MATERIAL_TASK_START_ACTION,
                    now_ms);
            }
            break;

        case MATERIAL_TASK_START_ACTION:
            if (task_mode == MATERIAL_TASK_MODE_LOAD)
            {
                if (!ManipulatorAction_StartToTrayAt(
                        pickup_source,
                        position_plan[task_index]))
                {
                    task_fail(now_ms);
                    break;
                }
            }
            else if (task_mode == MATERIAL_TASK_MODE_UNLOAD)
            {
                if (!ManipulatorAction_StartTrayToGroundAt(
                        position_plan[task_index]))
                {
                    task_fail(now_ms);
                    break;
                }
            }
            else
            {
                task_fail(now_ms);
                break;
            }

            enter_state(
                MATERIAL_TASK_WAIT_ACTION,
                now_ms);
            break;

        case MATERIAL_TASK_WAIT_ACTION:
            if (ManipulatorAction_HasError())
            {
                task_fail(now_ms);
            }
            else if (ManipulatorAction_IsDone())
            {
                if (task_index <
                    (TRAY_SELECTOR_SLOT_COUNT - 1U))
                {
                    task_index++;

                    enter_state(
                        MATERIAL_TASK_MOVE_TRAY,
                        now_ms);
                }
                else
                {
                    /* Keep the tray at slot 3 after the third action. */
                    enter_state(
                        MATERIAL_TASK_DONE,
                        now_ms);
                }
            }
            break;

        default:
            task_fail(now_ms);
            break;
    }
}

void MaterialLoadTask_Abort(void)
{
    bool was_busy = MaterialLoadTask_IsBusy();

    ManipulatorAction_Abort();

    if (was_busy)
    {
        task_state = MATERIAL_TASK_ERROR;
    }
}

bool MaterialLoadTask_IsBusy(void)
{
    return (task_state != MATERIAL_TASK_IDLE) &&
           (task_state != MATERIAL_TASK_DONE) &&
           (task_state != MATERIAL_TASK_ERROR);
}

bool MaterialLoadTask_IsDone(void)
{
    return task_state == MATERIAL_TASK_DONE;
}

bool MaterialLoadTask_HasError(void)
{
    return task_state == MATERIAL_TASK_ERROR;
}

MaterialTaskMode_t MaterialLoadTask_GetMode(void)
{
    return task_mode;
}

MaterialTaskState_t MaterialLoadTask_GetState(void)
{
    return task_state;
}

uint8_t MaterialLoadTask_GetPickupIndex(void)
{
    return task_index;
}
