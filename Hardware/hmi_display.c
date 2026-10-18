#include "hmi_display.h"
#include "hmi_protocol.h"
#include "usart.h"
#include <string.h>

#define HMI_TX_TIMEOUT_MS 50U
#define HMI_FAST_UPDATE_MS 200U

static uint32_t s_last_fast_update;

static void append_char(char *buf, uint16_t cap, uint16_t *pos, char c)
{
    if (*pos + 1U >= cap) return;
    buf[(*pos)++] = c;
    buf[*pos] = '\0';
}

static void append_str(char *buf, uint16_t cap, uint16_t *pos, const char *s)
{
    if (!s) return;
    while (*s) append_char(buf, cap, pos, *s++);
}

static void append_u32(char *buf, uint16_t cap, uint16_t *pos, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0U;
    if (v == 0U) { append_char(buf, cap, pos, '0'); return; }
    while (v && n < sizeof(tmp))
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n) append_char(buf, cap, pos, tmp[--n]);
}

static void append_s10(char *buf, uint16_t cap, uint16_t *pos, float x)
{
    int32_t q = (int32_t)(x * 10.0f + (x >= 0.0f ? 0.5f : -0.5f));
    uint32_t mag;
    if (q < 0)
    {
        append_char(buf, cap, pos, '-');
        mag = (uint32_t)(-q);
    }
    else mag = (uint32_t)q;
    append_u32(buf, cap, pos, mag / 10U);
    append_char(buf, cap, pos, '.');
    append_char(buf, cap, pos, (char)('0' + (mag % 10U)));
}

static void send_text(const char *object, const char *text)
{
    char cmd[128] = {0};
    uint16_t p = 0U;
    append_str(cmd, sizeof(cmd), &p, object);
    append_str(cmd, sizeof(cmd), &p, ".txt=\"");
    append_str(cmd, sizeof(cmd), &p, text ? text : "");
    append_char(cmd, sizeof(cmd), &p, '"');
    HMI_SendCommand(cmd);
}

static void send_u8(const char *object, uint8_t value)
{
    char cmd[96] = {0};
    uint16_t p = 0U;
    append_str(cmd, sizeof(cmd), &p, object);
    append_str(cmd, sizeof(cmd), &p, ".txt=\"");
    append_u32(cmd, sizeof(cmd), &p, value);
    append_char(cmd, sizeof(cmd), &p, '"');
    HMI_SendCommand(cmd);
}

static void send_float1(const char *object, float value)
{
    char cmd[96] = {0};
    uint16_t p = 0U;
    append_str(cmd, sizeof(cmd), &p, object);
    append_str(cmd, sizeof(cmd), &p, ".txt=\"");
    append_s10(cmd, sizeof(cmd), &p, value);
    append_char(cmd, sizeof(cmd), &p, '"');
    HMI_SendCommand(cmd);
}

void HMI_SendCommand(const char *cmd)
{
    static const uint8_t end[3] = {0xFFU, 0xFFU, 0xFFU};
    uint16_t len = 0U;
    if (!cmd) return;
    while (cmd[len] != '\0') ++len;

    /*
     * HMI 端口：USART3。
     * PB10 / USART3_TX -> HMI RX；接收方向见 app_uart.c 的 PE15 / USART3_RX。
     */
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)cmd, len, HMI_TX_TIMEOUT_MS);
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)end, 3U, HMI_TX_TIMEOUT_MS);
}

void HMI_Display_Init(void)
{
    s_last_fast_update = 0U;
}

void HMI_Page_Config(void)     { HMI_SendCommand("page page0"); }
void HMI_Page_ModeSelect(void) { HMI_SendCommand("page page3"); }
void HMI_Page_Race(void)       { HMI_SendCommand("page page4"); }
void HMI_Page_Debug(void)      { HMI_SendCommand("page page5"); }

void HMI_Race_SetTaskCode(const char *code) { send_text("page4.tTaskCode", code); }
void HMI_Race_SetStage(const char *stage)   { send_text("page4.tStage", stage); }
void HMI_Race_SetAction(const char *action) { send_text("page4.tAction", action); }
void HMI_Race_SetColor(const char *color)   { send_text("page4.tColor", color); }
void HMI_Race_SetResult(const char *result) { send_text("page4.tResult", result); }

void HMI_Debug_SetTaskCode(const char *code) { send_text("page5.tDbgTask", code); }
void HMI_Debug_SetTargetYaw(float yaw_deg)   { send_float1("page5.tTargetYaw", yaw_deg); }
void HMI_Debug_SetCurrentYaw(float yaw_deg)  { send_float1("page5.tCurrentYaw", yaw_deg); }
void HMI_Debug_SetCurrentNode(uint8_t node)  { send_u8("page5.tCurrentNode", node); }
void HMI_Debug_SetTargetNode(uint8_t node)   { send_u8("page5.tTargetNode", node); }

void HMI_Debug_SetPath(const uint8_t *path, uint8_t length)
{
    char text[48] = {0};
    uint16_t p = 0U;
    uint8_t i;

    if (!path || length == 0U)
    {
        send_text("page5.tPath", "NO PATH");
        return;
    }

    for (i = 0U; i < length; ++i)
    {
        if (i) append_char(text, sizeof(text), &p, '>');
        append_u32(text, sizeof(text), &p, path[i]);
    }
    send_text("page5.tPath", text);
}

void HMI_Debug_SetSpeed(float speed)
{
    send_float1("page5.tSpeed", speed);
}

void HMI_Display_Periodic(uint32_t now_ms,
                          float current_yaw,
                          float target_yaw,
                          float current_speed)
{
    if (HMI_Mode_Get() != HMI_MODE_DEBUG)
        return;
    if ((uint32_t)(now_ms - s_last_fast_update) < HMI_FAST_UPDATE_MS)
        return;
    s_last_fast_update = now_ms;

    HMI_Debug_SetCurrentYaw(current_yaw);
    HMI_Debug_SetTargetYaw(target_yaw);
    HMI_Debug_SetSpeed(current_speed);
}
