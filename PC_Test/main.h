#ifndef __PC_TEST_MAIN_H
#define __PC_TEST_MAIN_H

#include <stdint.h>

/* ====================================================
 * CMSIS interrupt mock
 * ==================================================== */

#ifndef __get_PRIMASK
static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}
#endif

#ifndef __disable_irq
static inline void __disable_irq(void)
{
}
#endif

#ifndef __enable_irq
static inline void __enable_irq(void)
{
}
#endif


/* ====================================================
 * STM32 HAL UART mock
 * ==================================================== */

typedef struct
{
    uint32_t dummy;
} UART_HandleTypeDef;


typedef enum
{
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;


HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout
);

uint32_t HAL_GetTick(void);

#endif
