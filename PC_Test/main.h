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


/* ====================================================
 * STM32 HAL GPIO mock
 * ==================================================== */

typedef struct
{
    uint32_t dummy;
} GPIO_TypeDef;

extern GPIO_TypeDef *const GPIOA;
extern GPIO_TypeDef *const GPIOB;
extern GPIO_TypeDef *const GPIOC;
extern GPIO_TypeDef *const GPIOD;
extern GPIO_TypeDef *const GPIOE;

#define GPIO_PIN_0   ((uint16_t)0x0001U)
#define GPIO_PIN_1   ((uint16_t)0x0002U)
#define GPIO_PIN_2   ((uint16_t)0x0004U)
#define GPIO_PIN_3   ((uint16_t)0x0008U)
#define GPIO_PIN_4   ((uint16_t)0x0010U)
#define GPIO_PIN_5   ((uint16_t)0x0020U)
#define GPIO_PIN_6   ((uint16_t)0x0040U)
#define GPIO_PIN_7   ((uint16_t)0x0080U)

#define GPIO_PIN_RESET       0x00000000U
#define GPIO_PIN_SET         0x00000001U
#define GPIO_MODE_INPUT      0x00000000U
#define GPIO_MODE_OUTPUT_PP  0x00000001U
#define GPIO_NOPULL          0x00000000U
#define GPIO_PULLUP          0x00000001U
#define GPIO_SPEED_FREQ_LOW  0x00000000U

typedef struct
{
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

#endif
