#ifndef __MATERIAL_LOAD_TASK_H
#define __MATERIAL_LOAD_TASK_H

#include "main.h"
#include "manipulator_action.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Material transfer direction handled by the shared task state machine.
 */
typedef enum
{
    MATERIAL_TASK_MODE_NONE = 0,
    MATERIAL_TASK_MODE_LOAD,
    MATERIAL_TASK_MODE_UNLOAD
} MaterialTaskMode_t;

/**
 * @brief Shared three-position material transfer task state.
 */
typedef enum
{
    MATERIAL_TASK_IDLE = 0,
    MATERIAL_TASK_MOVE_TRAY,
    MATERIAL_TASK_WAIT_TRAY,
    MATERIAL_TASK_START_ACTION,
    MATERIAL_TASK_WAIT_ACTION,
    MATERIAL_TASK_DONE,
    MATERIAL_TASK_ERROR
} MaterialTaskState_t;

void MaterialLoadTask_Init(void);

/**
 * @brief Load three materials from ground or table into tray slots 1, 2 and 3.
 */
bool MaterialLoadTask_Start(ManipulatorPickupSource_t source);

/**
 * @brief Unload tray slots 1, 2 and 3 to ground positions 1, 2 and 3.
 */
bool MaterialUnloadTask_Start(void);

void MaterialLoadTask_Update(uint32_t now_ms);
void MaterialLoadTask_Abort(void);

bool MaterialLoadTask_IsBusy(void);
bool MaterialLoadTask_IsDone(void);
bool MaterialLoadTask_HasError(void);

MaterialTaskMode_t MaterialLoadTask_GetMode(void);
MaterialTaskState_t MaterialLoadTask_GetState(void);
uint8_t MaterialLoadTask_GetPickupIndex(void);

#endif
