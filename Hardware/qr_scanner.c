#include "qr_scanner.h"
#include "task_code.h"
#include <string.h>

static char s_build[QR_CODE_MAX + 1U];
static char s_latest[QR_CODE_MAX + 1U];
static uint8_t s_len;
static volatile uint32_t s_publish_seq;
static uint32_t s_consumed_seq;
static uint32_t s_last_byte_ms;
static uint32_t s_last_frame_ms;

static void publish_frame(const char *frame, uint8_t len, uint32_t now_ms)
{
    if (!frame || len == 0U || len > QR_CODE_MAX) return;

    s_last_frame_ms = now_ms;

    /*
     * 扫码模块一直开启时会连续上报同一个二维码。相同内容只刷新在线时间，
     * 不增加发布序号，避免RobotApp把同一任务反复当成“新任务码”处理。
     */
    if (strlen(s_latest) == (size_t)len &&
        memcmp(s_latest, frame, (size_t)len) == 0)
    {
        return;
    }

    memcpy(s_latest, frame, (size_t)len);
    s_latest[len] = '\0';
    s_publish_seq++;
}

static void finish_frame(uint32_t now_ms)
{
    if (s_len == 0U) return;
    publish_frame(s_build, s_len, now_ms);
    s_len = 0U;
    s_build[0] = '\0';
}

/*
 * GM67/MJ6000链路没有二进制包头和包尾，不能等待某个固定帧头再接收。
 * 比赛任务码本身具有固定15字节结构，因此在连续字节流中用滑动窗口找出：
 *
 *   156+123+516+231
 *   156+123+516+231\r\n
 *   156+123+516+231156+123+516+231  （两次结果紧挨、没有分隔符）
 *
 * 这里只负责找出“长得像任务码”的边界；颜色是否重复、位置是否为123排列、
 * 两批颜色能否一一匹配等语义仍由task_code.c统一严格校验，避免协议层重复规则。
 */
static void extract_task_frames(uint32_t now_ms)
{
    uint8_t start = 0U;

    while ((uint8_t)(s_len - start) >= TASK_CODE_TEXT_LEN)
    {
        if (TaskCode_HasWireShape(&s_build[start]))
        {
            uint8_t consumed = (uint8_t)(start + TASK_CODE_TEXT_LEN);
            uint8_t remain = (uint8_t)(s_len - consumed);

            publish_frame(&s_build[start], TASK_CODE_TEXT_LEN, now_ms);
            if (remain > 0U)
                memmove(s_build, &s_build[consumed], remain);
            s_len = remain;
            s_build[s_len] = '\0';
            start = 0U;
        }
        else
        {
            start++;
        }
    }

    /*
     * 尚未找到完整帧时，只保留末尾14字节。更早的字节无论再拼多少新数据，
     * 都不可能成为一个新的15字节帧起点；丢弃它们可防止噪声把缓存撑满。
     */
    if (start > 0U)
    {
        uint8_t remain = (uint8_t)(s_len - start);
        if (remain > 0U) memmove(s_build, &s_build[start], remain);
        s_len = remain;
        s_build[s_len] = '\0';
    }
}

void QRScanner_Init(void)
{
    s_len = 0U;
    s_publish_seq = 0U;
    s_consumed_seq = 0U;
    s_last_byte_ms = 0U;
    s_last_frame_ms = 0U;
    s_build[0] = '\0';
    s_latest[0] = '\0';
}

void QRScanner_FeedByte(uint8_t byte, uint32_t now_ms)
{
    s_last_byte_ms = now_ms;

    if (byte == '\r' || byte == '\n' || byte == '\0')
    {
        finish_frame(now_ms);
        return;
    }

    /* 任务码只含可打印ASCII字符；除回车换行外的控制字节直接忽略。 */
    if (byte < 0x20U || byte > 0x7EU) return;

    if (s_len >= QR_CODE_MAX)
    {
        memmove(s_build, &s_build[1], QR_CODE_MAX - 1U);
        s_len = QR_CODE_MAX - 1U;
    }

    s_build[s_len++] = (char)byte;
    s_build[s_len] = '\0';
    extract_task_frames(now_ms);
}

void QRScanner_Update(uint32_t now_ms)
{
    /*
     * 固定15字节任务码会在收齐时立即发布；这里的50 ms静默只作为兼容兜底，
     * 用于接收带分隔但长度异常的报文，并让上层显示QR INVALID便于排查。
     */
    if (s_len > 0U && (uint32_t)(now_ms - s_last_byte_ms) >= 50U)
        finish_frame(now_ms);
}

static uint8_t copy_code_stable(char *out, uint16_t max_len,
                                uint32_t *copied_seq)
{
    uint32_t before;
    uint32_t after;
    uint16_t i = 0U;

    if (!out || max_len < 2U) return 0U;

    do
    {
        /*
         * 串口中断可能在主循环复制字符串时发布新帧。复制前后各读取一次
         * 发布序号；若发生变化就重新复制，避免拿到前后两帧拼接的半成品。
         */
        before = s_publish_seq;
        i = 0U;
        while (i + 1U < max_len && s_latest[i] != '\0')
        {
            out[i] = s_latest[i];
            ++i;
        }
        out[i] = '\0';
        after = s_publish_seq;
    }
    while (before != after);

    if (copied_seq) *copied_seq = after;
    return (i > 0U) ? 1U : 0U;
}

uint8_t QRScanner_GetNewCode(char *out, uint16_t max_len)
{
    uint32_t seq;

    if (s_publish_seq == s_consumed_seq) return 0U;
    if (!copy_code_stable(out, max_len, &seq)) return 0U;
    s_consumed_seq = seq;
    return 1U;
}

uint8_t QRScanner_GetLatestCode(char *out, uint16_t max_len)
{
    return copy_code_stable(out, max_len, NULL);
}

uint8_t QRScanner_IsOnline(uint32_t now_ms)
{
    return (s_last_frame_ms != 0U &&
            (uint32_t)(now_ms - s_last_frame_ms) <= 2000U) ? 1U : 0U;
}
