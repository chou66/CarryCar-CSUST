#include <math.h>
#include <stdio.h>
#include <string.h>

#include "k230.h"
#include "k230_align.h"

K230_Status_t g_k230_status;
volatile K230_TaskState_t g_k230_task_state = K230_TASK_IDLE;
K230_TaskResult_t g_k230_task_result;

static uint32_t s_events;
static uint8_t s_stack_position;
static uint8_t s_stack_color;
static int s_stack_start_count;
static int s_place_start_count;
static int s_translate_count;
static int s_stop_count;
static float s_last_dx_m;
static float s_last_dy_m;
static int s_motion_done;
static int s_motion_failed;
static int s_pass;
static int s_fail;

uint32_t HAL_GetTick(void) { return 0U; }

uint32_t K230_TakeEvents(void)
{
    uint32_t events = s_events;
    s_events = 0U;
    return events;
}

HAL_StatusTypeDef K230_StartDetect(uint8_t color_id)
{
    (void)color_id;
    return HAL_OK;
}

HAL_StatusTypeDef K230_StartPlace(uint8_t position_id)
{
    (void)position_id;
    s_place_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef K230_StartStack(uint8_t position_id, uint8_t anchor_color_id)
{
    s_stack_position = position_id;
    s_stack_color = anchor_color_id;
    s_stack_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef K230_Stop(void)
{
    s_stop_count++;
    return HAL_OK;
}

void K230_TaskAbort(void)
{
}

void CarouselServo_SetAngle(float angle_deg) { (void)angle_deg; }
bool CarouselServo_IsDone(void) { return true; }
void GripperServo_Open_Large(void) {}
bool GripperServo_IsDone(void) { return true; }

void chassis_motion_translate(Chassis *chassis, float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms)
{
    (void)chassis;
    (void)speed_mps;
    (void)now_ms;
    s_last_dx_m = dx_m;
    s_last_dy_m = dy_m;
    s_translate_count++;
    s_motion_done = 0;
}

bool chassis_motion_done(const Chassis *chassis)
{
    (void)chassis;
    return s_motion_done != 0;
}

bool chassis_motion_failed(const Chassis *chassis)
{
    (void)chassis;
    return s_motion_failed != 0;
}

void chassis_motion_stop(Chassis *chassis) { (void)chassis; }

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static void set_offset_pair(char x_dir, int32_t x_px,
                            char y_dir, int32_t y_px,
                            uint32_t now_ms)
{
    g_k230_status.axis_x_dir = x_dir;
    g_k230_status.axis_x_px = x_px;
    g_k230_status.axis_x_ms = now_ms;
    g_k230_status.axis_y_dir = y_dir;
    g_k230_status.axis_y_px = y_px;
    g_k230_status.axis_y_ms = now_ms;
    s_events = K230_EVENT_OFFSET;
}

int main(void)
{
    Chassis chassis;

    memset(&chassis, 0, sizeof(chassis));
    memset(&g_k230_status, 0, sizeof(g_k230_status));
    K230Align_Init(&chassis);

    check(K230Align_StartStackPreset(2U, 2U, 0U),
          "STACK alignment accepts position 2 with color-2 anchor");
    K230Align_Update(0U);
    check(s_stack_start_count == 1 && s_stack_position == 2U &&
          s_stack_color == 2U && s_place_start_count == 0,
          "STACK mode emits stack,2,2 instead of ordinary place,2");

    set_offset_pair('w', 20, 'a', 10, 1U);
    K230Align_Update(1U);
    check(s_translate_count == 0,
          "offset before target identity never moves the chassis");

    g_k230_status.target_id = 1U;
    s_events = K230_EVENT_TARGET;
    K230Align_Update(2U);
    check(robot_k230_align_target_ok == 0U && s_translate_count == 0,
          "wrong target id keeps the STACK gate closed");

    g_k230_status.target_id = 2U;
    s_events = K230_EVENT_TARGET;
    K230Align_Update(3U);
    check(robot_k230_align_target_ok == 1U && s_translate_count == 0,
          "matching target opens the gate but discards older offsets");

    set_offset_pair('w', 20, 'a', 10, 4U);
    K230Align_Update(4U);
    check(s_translate_count == 1 &&
          fabsf(s_last_dx_m - 0.010f) < 0.0001f &&
          fabsf(s_last_dy_m - 0.005f) < 0.0001f,
          "fresh offset after target identity starts the expected body motion");

    /* 独立重启一轮，验证最终place,2可同时完成身份确认和收工。 */
    K230Align_Init(&chassis);
    check(K230Align_StartStackPreset(2U, 2U, 100U),
          "STACK alignment can start again after reinitialization");
    K230Align_Update(100U);
    g_k230_status.target_id = 2U;
    s_events = K230_EVENT_TARGET | K230_EVENT_PLACE;
    K230Align_Update(101U);
    check(!K230Align_IsDone() && K230Align_IsBusy() &&
          robot_k230_align_state == K230_ALIGN_STOPPING &&
          robot_k230_align_exit == K230_ALIGN_EXIT_NONE,
          "tagged place,2 waits for stopped before exposing DONE");
    check(s_stop_count == 1 && robot_k230_align_stop_requests == 1U,
          "successful tagged completion starts the stop handshake");

    K230Align_Update(200U);
    check(s_stop_count == 1,
          "stop is not resent before the retry interval");
    K230Align_Update(201U);
    check(s_stop_count == 2 && robot_k230_align_stop_requests == 2U,
          "a lost stopped acknowledgment causes a periodic stop retry");

    s_events = K230_EVENT_STOPPED;
    K230Align_Update(210U);
    check(K230Align_IsDone() && robot_k230_align_exit == K230_ALIGN_EXIT_PLACE,
          "matching stopped acknowledgment completes the alignment");

    check(K230Align_StartStackPreset(2U, 2U, 300U),
          "a completed controller can start another stop-handshake cycle");
    K230Align_Update(300U);
    g_k230_status.target_id = 2U;
    s_events = K230_EVENT_TARGET | K230_EVENT_PLACE;
    K230Align_Update(301U);
    K230Align_Update(1801U);
    check(!K230Align_HasError() && K230Align_IsBusy() &&
          robot_k230_align_state == K230_ALIGN_STOPPING &&
          robot_k230_align_stop_requests == 2U && s_stop_count == 4 &&
          robot_k230_align_stop_wait_ms == 1500U &&
          robot_k230_align_stop_ack_received == 0U,
          "missing stopped acknowledgment keeps retrying beyond the old timeout");
    s_events = K230_EVENT_STOPPED;
    K230Align_Update(1810U);
    check(K230Align_IsDone() &&
          robot_k230_align_exit == K230_ALIGN_EXIT_PLACE &&
          robot_k230_align_stop_ack_received == 1U &&
          robot_k230_align_stop_ack_ms == 1509U &&
          robot_k230_align_stop_tx_failures == 0U,
          "a late stopped acknowledgment still completes the alignment");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
