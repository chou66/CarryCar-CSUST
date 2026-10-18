#include <stdio.h>
#include <string.h>

#include "k230.h"
#include "usart.h"

/*
 * 第二轮暂存区串口协议回归测试。
 * 这里只验证最底层收发：target,n和place,n必须携带目标编号，stack命令必须
 * 原样发出。更上层K230Align再用K230_EVENT_TARGET决定是否放行底盘偏差。
 */
UART_HandleTypeDef huart4;

static uint32_t s_now_ms;
static char s_tx[32];
static int s_pass;
static int s_fail;

uint32_t HAL_GetTick(void)
{
    return s_now_ms;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout)
{
    (void)huart;
    (void)timeout;
    if (size >= sizeof(s_tx)) size = (uint16_t)(sizeof(s_tx) - 1U);
    memcpy(s_tx, data, size);
    s_tx[size] = '\0';
    return HAL_OK;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static uint32_t feed_line(const char *line)
{
    while (*line != '\0')
    {
        K230_FeedByte((uint8_t)*line++);
    }
    K230_Process();
    return K230_TakeEvents();
}

int main(void)
{
    uint32_t events;

    K230_Init();

    s_now_ms = 10U;
    events = feed_line("target,2\n");
    check(events == K230_EVENT_TARGET && g_k230_status.target_id == 2U,
          "target,2 records the selected storage position");

    s_now_ms = 20U;
    events = feed_line("place,2\n");
    check((events & (K230_EVENT_TARGET | K230_EVENT_PLACE)) ==
          (K230_EVENT_TARGET | K230_EVENT_PLACE),
          "place,2 carries identity and completion in one line");
    check(g_k230_status.target_id == 2U,
          "tagged place keeps target id 2");

    s_now_ms = 30U;
    events = feed_line("target,4\n");
    check(events == K230_EVENT_NONE && k230_invalid_line_count == 1U,
          "out-of-range storage target is rejected");

    s_now_ms = 40U;
    events = feed_line("stopped\n");
    check(events == K230_EVENT_STOPPED,
          "stopped acknowledges that K230 cleared its active task");

    memset(s_tx, 0, sizeof(s_tx));
    check(K230_StartStack(2U, 2U) == HAL_OK &&
          strcmp(s_tx, "stack,2,2\n") == 0,
          "STM32 emits the stack position and anchor color");

    check(K230_StartStack(0U, 2U) == HAL_ERROR &&
          K230_StartStack(2U, 7U) == HAL_ERROR,
          "invalid stack parameters are rejected before UART transmit");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
