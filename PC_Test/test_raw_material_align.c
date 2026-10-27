#include <math.h>
#include <stdio.h>
#include <string.h>

#include "k230.h"
#include "raw_material_align.h"
#include "robot_config.h"

K230_Status_t g_k230_status;
volatile K230_TaskState_t g_k230_task_state = K230_TASK_IDLE;
K230_TaskResult_t g_k230_task_result;
volatile uint32_t k230_rx_byte_count;
volatile uint32_t k230_ring_overflow_count;
volatile uint32_t k230_valid_line_count;
volatile uint32_t k230_invalid_line_count;

static uint32_t s_now;
static uint32_t s_events;
static float s_slide_mm = RAW_VISION_SLIDE_START_MM;
static float s_slide_target_mm = RAW_VISION_SLIDE_START_MM;
static float s_carousel_target_deg = CAROUSEL_PICKUP_POS2_DEG;
static int s_slide_busy;
static int s_slide_error;
static int s_carousel_done = 1;
static int s_start_detect_count;
static int s_stop_count;
static int s_pass;
static int s_fail;

uint32_t HAL_GetTick(void) { return s_now; }

uint32_t K230_TakeEvents(void)
{
    uint32_t events = s_events;
    s_events = 0U;
    return events;
}

HAL_StatusTypeDef K230_StartDetect(uint8_t color_id)
{
    (void)color_id;
    s_start_detect_count++;
    return HAL_OK;
}

HAL_StatusTypeDef K230_Stop(void)
{
    s_stop_count++;
    return HAL_OK;
}

bool ManipulatorAction_SlideJogPositionKnown(void) { return true; }
bool ManipulatorAction_SlideJogIsBusy(void) { return s_slide_busy != 0; }
bool ManipulatorAction_SlideJogHasError(void) { return s_slide_error != 0; }
bool ManipulatorAction_SlideJogIsDone(void) { return s_slide_busy == 0; }
float ManipulatorAction_SlideJogGetPositionMm(void) { return s_slide_mm; }

bool ManipulatorAction_SlideJogToPositionStart(float target_mm)
{
    s_slide_target_mm = target_mm;
    s_slide_busy = 1;
    return true;
}

void CarouselServo_SetAngle(float angle_deg)
{
    s_carousel_target_deg = angle_deg;
    s_carousel_done = 0;
}

bool CarouselServo_IsDone(void) { return s_carousel_done != 0; }

static int nearf_local(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static void feed_pair(char x_dir, int32_t x_px,
                      char y_dir, int32_t y_px,
                      uint32_t now_ms)
{
    s_now = now_ms;
    g_k230_status.axis_x_dir = x_dir;
    g_k230_status.axis_x_px = x_px;
    g_k230_status.axis_x_ms = now_ms;
    g_k230_status.axis_y_dir = y_dir;
    g_k230_status.axis_y_px = y_px;
    g_k230_status.axis_y_ms = now_ms;
    s_events = K230_EVENT_OFFSET;
    RawMaterialAlign_Update(now_ms);
}

int main(void)
{
    memset(&g_k230_status, 0, sizeof(g_k230_status));
    RawMaterialAlign_Init();

    check(RawMaterialAlign_Start(1U, 0U),
          "Raw align starts from the compensated slide preset");
    check(RAW_VISION_TOLERANCE_PX == 4U,
          "STM32 mechanical pickup tolerance is intentionally relaxed to four pixels");
    check(s_start_detect_count == 1,
          "Start sends one detect command");

    s_now = 10U;
    g_k230_status.color_id = 1U;
    s_events = K230_EVENT_COLOR;
    RawMaterialAlign_Update(s_now);

    feed_pair('w', 20, 'a', 10, 20U);
    check(nearf_local(s_slide_target_mm,
                      RAW_VISION_SLIDE_START_MM + 10.0f, 0.01f),
          "Positive radial pixels extend the slide by PID output");
    check(nearf_local(s_carousel_target_deg,
                      CAROUSEL_PICKUP_POS2_DEG + 1.333f, 0.01f),
          "Positive tangential pixels increase carousel angle");

    s_slide_mm = s_slide_target_mm;
    s_slide_busy = 0;
    s_carousel_done = 1;
    s_now = 30U;
    RawMaterialAlign_Update(s_now);
    s_now = 130U;
    RawMaterialAlign_Update(s_now);
    check(s_start_detect_count == 2,
          "Settling completion immediately requests the next K230 frame");

    feed_pair('w', 4, 'a', 4, 140U);
    check(s_start_detect_count == 3,
          "A four-pixel verification frame is accepted without mechanical twitching");
    feed_pair('w', 4, 'a', 4, 150U);
    feed_pair('w', 4, 'a', 4, 160U);
    check(!RawMaterialAlign_IsDone() && RawMaterialAlign_IsBusy() &&
          robot_raw_align_state == RAW_ALIGN_STOPPING,
          "Three centered frames keep the dual-end completion decision but wait for stopped");
    check(s_stop_count == 1,
          "Completion starts the K230 stop handshake");

    /* 第一个 stopped 模拟丢包：100 ms 到期必须重发，不能直接开始机械夹取。 */
    s_now = 259U;
    RawMaterialAlign_Update(s_now);
    check(s_stop_count == 1,
          "Stop is not resent before the retry interval");
    s_now = 260U;
    RawMaterialAlign_Update(s_now);
    check(s_stop_count == 2 && robot_raw_align_stop_requests == 2U,
          "A lost stopped acknowledgment causes a periodic stop retry");

    s_now = 270U;
    s_events = K230_EVENT_STOPPED;
    RawMaterialAlign_Update(s_now);
    check(RawMaterialAlign_IsDone() &&
          robot_raw_align_exit == RAW_ALIGN_EXIT_TOLERANCE,
          "The STM32 tolerance path finishes only after stopped is acknowledged");

    /* 第二轮从159 mm开始，正向大误差最多只能到新的160 mm软件上限。 */
    s_slide_mm = 159.0f;
    s_slide_busy = 0;
    s_carousel_done = 1;
    s_now = 300U;
    check(RawMaterialAlign_Start(1U, s_now),
          "A completed controller can start the next material");

    /* 兼容旧版K230：busy不再杀死ZoneTask，而是stop确认后重发当前颜色。 */
    s_now = 310U;
    strcpy(g_k230_status.error_text, "busy");
    s_events = K230_EVENT_ERROR;
    RawMaterialAlign_Update(s_now);
    check(!RawMaterialAlign_HasError() &&
          robot_raw_align_state == RAW_ALIGN_STOPPING,
          "error,busy enters recoverable stop handshake instead of task error");

    s_now = 320U;
    s_events = K230_EVENT_STOPPED;
    RawMaterialAlign_Update(s_now);
    check(robot_raw_align_state == RAW_ALIGN_WAIT_OFFSET &&
          robot_raw_align_busy_recoveries == 1U,
          "Busy recovery restarts the requested detect after stopped");

    s_now = 330U;
    g_k230_status.color_id = 1U;
    s_events = K230_EVENT_COLOR;
    RawMaterialAlign_Update(s_now);
    feed_pair('w', 1000, 'a', 0, 340U);
    check(nearf_local(s_slide_target_mm, 160.0f, 0.01f),
          "Slide target is clamped at the requested 160 mm software limit");
    check(robot_raw_align_slide_limit == RAW_ALIGN_LIMIT_MAX,
          "Watch diagnostics report the maximum slide limit");

    /* K230 自己判定 grab 的另一条完成路径也必须保留，并共用 stopped 握手。 */
    s_slide_mm = RAW_VISION_SLIDE_START_MM;
    s_slide_busy = 0;
    s_carousel_done = 1;
    RawMaterialAlign_Init();
    s_now = 400U;
    check(RawMaterialAlign_Start(4U, s_now),
          "A fresh controller starts the K230-side grab decision path");
    s_now = 410U;
    g_k230_status.color_id = 4U;
    s_events = K230_EVENT_COLOR | K230_EVENT_GRAB;
    RawMaterialAlign_Update(s_now);
    check(robot_raw_align_state == RAW_ALIGN_STOPPING &&
          robot_raw_align_exit == RAW_ALIGN_EXIT_NONE,
          "K230 grab also waits for stopped before exposing DONE");
    s_now = 1910U;
    RawMaterialAlign_Update(s_now);
    check(!RawMaterialAlign_HasError() && RawMaterialAlign_IsBusy() &&
          robot_raw_align_state == RAW_ALIGN_STOPPING &&
          robot_raw_align_stop_requests == 2U &&
          robot_raw_align_stop_wait_ms == 1500U &&
          robot_raw_align_stop_ack_received == 0U,
          "Raw stop handshake keeps retrying beyond the old timeout");
    s_now = 1920U;
    s_events = K230_EVENT_STOPPED;
    RawMaterialAlign_Update(s_now);
    check(RawMaterialAlign_IsDone() &&
          robot_raw_align_exit == RAW_ALIGN_EXIT_GRAB &&
          robot_raw_align_stop_ack_received == 1U &&
          robot_raw_align_stop_ack_ms == 1510U &&
          robot_raw_align_stop_tx_failures == 0U,
          "A late stopped acknowledgment still completes the raw grab path");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
