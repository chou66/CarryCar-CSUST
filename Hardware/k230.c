#include "k230.h"
#include "usart.h"
#include <string.h>

#define K230_RING_SIZE       256U
#define K230_LINE_MAX         64U

static volatile uint8_t s_ring[K230_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static char s_line[K230_LINE_MAX];
static uint8_t s_line_length;
static uint8_t s_drop_line;

static uint32_t s_pending_events;

K230_Status_t g_k230_status;
volatile K230_TaskState_t g_k230_task_state = K230_TASK_IDLE;
K230_TaskResult_t g_k230_task_result;

volatile uint32_t k230_rx_byte_count;
volatile uint32_t k230_ring_overflow_count;
volatile uint32_t k230_valid_line_count;
volatile uint32_t k230_invalid_line_count;


/*
 * 整数解析函数：只取整数部分。
 *
 * K230 实测偏差帧会带小数（如 w,215.25），而本函数结尾校验要求字符串
 * 正好停在 '\0'，遇到点号会判定整行非法并丢弃，于是偏差帧一帧都进不来，
 * 对位只剩 grab 这一条路可走。这里把小数点及其后的小数位一并吃掉，
 * 只保留整数部分；被舍掉的 0.25 相对 K230_ALIGN_MM_PER_PX 的标定误差
 * 可以忽略。
 */
static uint8_t parse_unsigned(
    const char *text,
    uint32_t *value)
{
    uint32_t result = 0U;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        has_digit = 1U;
        result = result * 10U
               + (uint32_t)(*text - '0');
        text++;
    }

    /* 容忍小数部分：小数点后必须是数字，否则仍按非法行处理。 */
    if ((*text == '.') && (text[1] >= '0') && (text[1] <= '9'))
    {
        text++;

        while ((*text >= '0') && (*text <= '9'))
        {
            text++;
        }
    }

    if ((!has_digit) || (*text != '\0'))
    {
        return 0U;
    }

    *value = result;
    return 1U;
}

/*错误文本保存*/
static void save_error_text(const char *text)
{
    uint8_t index = 0U;

    while ((text[index] != '\0') &&
           (index < (K230_ERROR_TEXT_MAX - 1U)))
    {
        g_k230_status.error_text[index] =
            text[index];
        index++;
    }

    g_k230_status.error_text[index] = '\0';
}

/*完整行解析函数*/
static void process_line(const char *line)
{
    uint32_t value;
    uint32_t event = K230_EVENT_NONE;

    if ((line == NULL) || (*line == '\0'))
    {
        return;
    }

    /* 颜色编号：1～6。 */
    if ((line[0] >= '1') &&
        (line[0] <= '6') &&
        (line[1] == '\0'))
    {
        g_k230_status.color_id =
            (uint8_t)(line[0] - '0');

        event = K230_EVENT_COLOR;
    }
    /* 方向偏差：a,18 / d,7 / w,6 / s,10。 */
    else if (((line[0] == 'a') ||
              (line[0] == 'd') ||
              (line[0] == 'w') ||
              (line[0] == 's')) &&
             (line[1] == ',') &&
             parse_unsigned(&line[2], &value))
    {
        g_k230_status.direction = line[0];
        g_k230_status.offset_px = (int32_t)value;

        /*
         * 按轴另存一份并盖时间戳。一帧两行（d,136.00 / w,102.50）会在
         * 同一个 K230_Process() 里先后被解析，上面两个字段只留最后一行，
         * 所以 w/s 与 a/d 必须分开存，上层才配得成一对正交偏差。
         */
        if ((line[0] == 'w') || (line[0] == 's'))
        {
            g_k230_status.axis_x_dir = line[0];
            g_k230_status.axis_x_px = (int32_t)value;
            g_k230_status.axis_x_ms = HAL_GetTick();
        }
        else
        {
            g_k230_status.axis_y_dir = line[0];
            g_k230_status.axis_y_px = (int32_t)value;
            g_k230_status.axis_y_ms = HAL_GetTick();
        }

        event = K230_EVENT_OFFSET;
    }
    else if (strcmp(line, "grab") == 0)
    {
        event = K230_EVENT_GRAB;
    }
    else if (strcmp(line, "place") == 0)
    {
        event = K230_EVENT_PLACE;
    }
    else if (strncmp(line, "error,", 6U) == 0)
    {
        save_error_text(&line[6]);
        event = K230_EVENT_ERROR;
    }
    else if (strcmp(line, "heartbeat,1") == 0)
    {
        event = K230_EVENT_HEARTBEAT;
    }
    else
    {
        k230_invalid_line_count++;
        return;
    }

    g_k230_status.valid = 1U;
    g_k230_status.last_event = event;
    g_k230_status.timestamp_ms = HAL_GetTick();
    g_k230_status.message_count++;

    s_pending_events |= event;
    k230_valid_line_count++;
}

void K230_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_line_length = 0U;
    s_drop_line = 0U;
    s_pending_events = K230_EVENT_NONE;

    memset(&g_k230_status,
           0,
           sizeof(g_k230_status));

    g_k230_task_state = K230_TASK_IDLE;
    memset(&g_k230_task_result,
           0,
           sizeof(g_k230_task_result));

    k230_rx_byte_count = 0U;
    k230_ring_overflow_count = 0U;
    k230_valid_line_count = 0U;
    k230_invalid_line_count = 0U;
}

//字节接收
void K230_FeedByte(uint8_t byte)
{
    uint16_t next;

    k230_rx_byte_count++;

    next = (uint16_t)(
        (s_head + 1U) &
        (K230_RING_SIZE - 1U));

    if (next == s_tail)
    {
        k230_ring_overflow_count++;
        return;
    }

    s_ring[s_head] = byte;
    s_head = next;
}

void K230_Process(void)
{
    while (s_tail != s_head)
    {
        uint8_t byte = s_ring[s_tail];

        s_tail = (uint16_t)(
            (s_tail + 1U) &
            (K230_RING_SIZE - 1U));

        if ((byte == '\r') || (byte == '\n'))
        {
            if ((!s_drop_line) &&
                (s_line_length > 0U))
            {
                s_line[s_line_length] = '\0';
                process_line(s_line);
            }

            s_line_length = 0U;
            s_drop_line = 0U;
            continue;
        }

        if (s_drop_line)
        {
            continue;
        }

        /* 只接受可打印ASCII字符。 */
        if ((byte < 0x20U) || (byte > 0x7EU))
        {
            s_line_length = 0U;
            s_drop_line = 1U;
            continue;
        }

        if (s_line_length <
            (K230_LINE_MAX - 1U))
        {
            s_line[s_line_length++] =
                (char)byte;
        }
        else
        {
            /*
             * 超长行一直丢弃到下一个换行，
             * 防止把超长消息的后半段当成新命令。
             */
            s_line_length = 0U;
            s_drop_line = 1U;
        }
    }
}

uint32_t K230_TakeEvents(void)
{
    uint32_t events = s_pending_events;

    s_pending_events = K230_EVENT_NONE;
    return events;
}

uint8_t K230_GetStatus(K230_Status_t *out)
{
    if ((out == NULL) ||
        (!g_k230_status.valid))
    {
        return 0U;
    }

    *out = g_k230_status;
    return 1U;
}

uint8_t K230_IsOnline(uint32_t now_ms)
{
    if (!g_k230_status.valid)
    {
        return 0U;
    }

    return ((uint32_t)(
        now_ms -
        g_k230_status.timestamp_ms)
        <= K230_DATA_TIMEOUT_MS) ?
        1U : 0U;
}

HAL_StatusTypeDef K230_StartDetect(
    uint8_t color_id)
{
    char command[] = "detect,0\n";

    if ((color_id < 1U) ||
        (color_id > 6U))
    {
        return HAL_ERROR;
    }

    command[7] =
        (char)('0' + color_id);

    return K230_SendString(command);
}

HAL_StatusTypeDef K230_StartPlace(
    uint8_t position_id)
{
    char command[] = "place,0\n";

    if ((position_id < 1U) ||
        (position_id > 3U))
    {
        return HAL_ERROR;
    }

    command[6] =
        (char)('0' + position_id);

    return K230_SendString(command);
}

HAL_StatusTypeDef K230_Stop(void)
{
    return K230_SendString("stop\n");
}


HAL_StatusTypeDef K230_SendString(const char *text)
{
    uint16_t len = 0U;
    if (!text) return HAL_ERROR;
    while (text[len] != '\0') ++len;
    return HAL_UART_Transmit(&huart4, (uint8_t *)text, len, 100U);
}

static uint8_t k230_task_can_start(void)
{
    return (g_k230_task_state == K230_TASK_IDLE) ||
           (g_k230_task_state == K230_TASK_DONE) ||
           (g_k230_task_state == K230_TASK_ERROR);
}

static void k230_task_clear_result(
    K230_TaskMode_t mode,
    uint8_t requested_id,
    uint32_t now_ms)
{
    memset(&g_k230_task_result,
           0,
           sizeof(g_k230_task_result));

    g_k230_task_result.mode = mode;
    g_k230_task_result.requested_id = requested_id;
    g_k230_task_result.start_ms = now_ms;
    g_k230_task_result.start_message_count =
        g_k230_status.message_count;
    g_k230_task_result.last_message_count =
        g_k230_status.message_count;
}

static void k230_task_fail(uint8_t error_code)
{
    uint8_t index = 0U;

    g_k230_task_result.error_code = error_code;

    if (error_code == K230_TASK_ERROR_REMOTE)
    {
        while ((g_k230_status.error_text[index] != '\0') &&
               (index < (K230_ERROR_TEXT_MAX - 1U)))
        {
            g_k230_task_result.error_text[index] =
                g_k230_status.error_text[index];
            index++;
        }

        g_k230_task_result.error_text[index] = '\0';
    }

    g_k230_task_state = K230_TASK_ERROR;
}

uint8_t K230_TaskStartDetect(uint8_t color_id)
{
    uint32_t now_ms;

    if (!k230_task_can_start() ||
        (color_id < 1U) ||
        (color_id > 6U))
    {
        return 0U;
    }

    /* 先解析并清掉旧任务残留，防止旧grab/offset被本次任务使用。 */
    K230_Process();
    (void)K230_TakeEvents();

    now_ms = HAL_GetTick();
    k230_task_clear_result(
        K230_TASK_MODE_DETECT,
        color_id,
        now_ms);

    if (K230_StartDetect(color_id) != HAL_OK)
    {
        k230_task_fail(K230_TASK_ERROR_SEND);
        return 0U;
    }

    g_k230_task_state = K230_TASK_WAIT_RESULT;
    return 1U;
}

uint8_t K230_TaskStartPlace(uint8_t position_id)
{
    uint32_t now_ms;

    if (!k230_task_can_start() ||
        (position_id < 1U) ||
        (position_id > 3U))
    {
        return 0U;
    }

    K230_Process();
    (void)K230_TakeEvents();

    now_ms = HAL_GetTick();
    k230_task_clear_result(
        K230_TASK_MODE_PLACE,
        position_id,
        now_ms);

    if (K230_StartPlace(position_id) != HAL_OK)
    {
        k230_task_fail(K230_TASK_ERROR_SEND);
        return 0U;
    }

    g_k230_task_state = K230_TASK_WAIT_RESULT;
    return 1U;
}

void K230_TaskUpdate(uint32_t now_ms)
{
    uint32_t events;

    /* UART4中断只收字节；完整行解析始终在主循环完成。 */
    K230_Process();

    if (g_k230_task_state != K230_TASK_WAIT_RESULT)
    {
        return;
    }

    events = K230_TakeEvents();

    if ((events != K230_EVENT_NONE) &&
        (g_k230_status.message_count >
         g_k230_task_result.start_message_count))
    {
        g_k230_task_result.events_seen |= events;
        g_k230_task_result.last_message_count =
            g_k230_status.message_count;

        if ((events & K230_EVENT_COLOR) != 0U)
        {
            g_k230_task_result.color_id =
                g_k230_status.color_id;
            g_k230_task_result.color_valid = 1U;
        }

        if ((events & K230_EVENT_OFFSET) != 0U)
        {
            g_k230_task_result.direction =
                g_k230_status.direction;
            g_k230_task_result.offset_px =
                g_k230_status.offset_px;
            g_k230_task_result.offset_pending = 1U;
        }

        if ((events & K230_EVENT_ERROR) != 0U)
        {
            k230_task_fail(K230_TASK_ERROR_REMOTE);
            return;
        }

        if ((g_k230_task_result.mode ==
             K230_TASK_MODE_DETECT) &&
            ((events & K230_EVENT_GRAB) != 0U))
        {
            g_k230_task_state = K230_TASK_DONE;
            return;
        }

        if ((g_k230_task_result.mode ==
             K230_TASK_MODE_PLACE) &&
            ((events & K230_EVENT_PLACE) != 0U))
        {
            g_k230_task_state = K230_TASK_DONE;
            return;
        }
    }

    if ((now_ms - g_k230_task_result.start_ms) >=
        K230_TASK_TIMEOUT_MS)
    {
        (void)K230_Stop();
        k230_task_fail(K230_TASK_ERROR_TIMEOUT);
    }
}

void K230_TaskAbort(void)
{
    (void)K230_Stop();
    (void)K230_TakeEvents();
    g_k230_task_state = K230_TASK_IDLE;
}

uint8_t K230_TaskIsBusy(void)
{
    return (g_k230_task_state ==
            K230_TASK_WAIT_RESULT) ? 1U : 0U;
}

uint8_t K230_TaskIsDone(void)
{
    return (g_k230_task_state ==
            K230_TASK_DONE) ? 1U : 0U;
}

uint8_t K230_TaskHasError(void)
{
    return (g_k230_task_state ==
            K230_TASK_ERROR) ? 1U : 0U;
}

uint8_t K230_TaskTakeOffset(
    char *direction,
    int32_t *offset_px)
{
    if ((direction == NULL) ||
        (offset_px == NULL) ||
        (!g_k230_task_result.offset_pending))
    {
        return 0U;
    }

    *direction = g_k230_task_result.direction;
    *offset_px = g_k230_task_result.offset_px;
    g_k230_task_result.offset_pending = 0U;
    return 1U;
}

uint8_t K230_TaskGetResult(K230_TaskResult_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    *out = g_k230_task_result;
    return 1U;
}







