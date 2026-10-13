#ifndef __EMM_ADAPTER_H
#define __EMM_ADAPTER_H

#include "main.h"
#include <stdbool.h>


HAL_StatusTypeDef emm_vel(
    uint8_t id,
    uint8_t dir,
    uint16_t rpm,
    uint8_t acc,
    bool sync
);


HAL_StatusTypeDef emm_pos(
    uint8_t id,
    uint8_t dir,
    uint16_t speed,
    uint8_t acc,
    uint32_t clk,
    bool raF,
    bool snF
);


HAL_StatusTypeDef emm_stop(
    uint8_t id,
    bool sync
);


HAL_StatusTypeDef emm_read_status(
    uint8_t id
);


HAL_StatusTypeDef emm_sync_start(void);


void emm_prepare_position_feedback(
    uint8_t id
);


bool emm_all_position_reached_after(
    uint8_t mask,
    uint32_t start_ms
);


bool emm_any_position_fault_after(
    uint8_t mask,
    uint32_t start_ms
);


/* 机械臂专用：仅堵转保护锁存(0x08)算致命故障；落座压料瞬间的堵转检测警告(0x04)不算。 */
bool emm_any_stall_protect_after(
    uint8_t mask,
    uint32_t start_ms
);


/* mask 内 start_ms 之后已上报到位(0x02)的电机位集合。 */
uint8_t emm_reached_mask_after(
    uint8_t mask,
    uint32_t start_ms
);


/* mask 内 start_ms 之后上报堵转保护锁存位(0x08)的电机位集合。 */
uint8_t emm_faulted_mask_after(
    uint8_t mask,
    uint32_t start_ms
);


#endif
