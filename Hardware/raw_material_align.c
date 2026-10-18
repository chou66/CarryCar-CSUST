#include "raw_material_align.h"

#include "k230.h"
#include "manipulator_action.h"
#include "robot_config.h"
#include "servo.h"

#include <string.h>

/*
 * =============================================================================
 * 原料圆盘二维视觉位置闭环——实现说明
 * =============================================================================
 *
 * 一、为什么单独做这个模块
 * --------------------------
 * 原来的 K230Align 通过麦克纳姆底盘平移消除像素误差。原料区若每抓一块料
 * 都移动底盘，会破坏 N8 节点的位置和航向基准，后续导航只能依赖估算补偿。
 * 本模块因此禁止驱动底盘，只使用安装相机的两个机械自由度完成对准：
 *
 *   K230 w/s 轴（px_x） -> 齿轮齿条伸缩，改变相机到圆盘的径向距离；
 *   K230 a/d 轴（px_y） -> 云台旋转，改变夹爪沿圆盘切向的指向。
 *
 * 二、这里的“PID”控制什么
 * ------------------------
 * 它是图像位置外环，不是电机速度环。K230 给像素位置误差，PID 输出本轮
 * 应增加/减少的机械位置：滑台输出单位为 mm，云台输出单位为 deg。滑台驱动
 * 器和舵机内部仍负责执行这个位置目标；直接加 RPM 只能缩短移动时间，不能
 * 决定该停在哪里，也不能解决图像居中问题。
 *
 * 三、一次闭环的时序
 * ------------------
 *   1. 上层先把云台转到 POS2、夹爪张开，并把滑台预伸到 75 mm；
 *   2. Start 清理旧串口事件，发送 detect,<颜色号>；
 *   3. WAIT_OFFSET 只接收执行器停稳后到达的新像素帧；
 *   4. 两轴 PID 同时计算下一组“云台绝对角度 + 滑台绝对位置”；
 *   5. MOVING 等两个执行器都到位，SETTLE 再留出机械稳定时间；
 *   6. 回到 WAIT_OFFSET 重新取像，直到连续多帧进入中心容差。
 *
 * 执行器运动期间收到的 grab/偏差不能用于下一轮，因为那一帧对应的是旧的
 * 相机姿态。只有停稳后重新建立的 s_offset_epoch_ms 之后的数据才有效。
 *
 * 四、安全策略
 * ------------
 *   - 云台只允许在 POS3..POS1 已标定角度之间运动；
 *   - 滑台虽有约 250 mm 机械行程，软件只开放 55..160 mm，保留末端余量；
 *   - 每轮位置增量单独钳位，防止像素比例或方向配置错误时一步冲到底；
 *   - 到达软件边界但仍未对准时继续观察旋转圆盘，不把它误判成电机故障；
 *   - 滑台中途失败后位置不再可信，由 manipulator_action 拒绝继续累计位移。
 *
 * 五、实车调参顺序
 * ----------------
 * 先看方向，再调比例，最后才考虑微分/积分：
 *   1. 若误差越调越大，只翻转 RAW_VISION_ANGLE_SIGN 或 SLIDE_SIGN；
 *   2. 方向正确后调 Kp，使每轮能明显靠近但不会来回跨过中心；
 *   3. 有明显跨越/抖动再增加 Kd；Ki 默认保持 0，旋转目标和机械限位场景
 *      很容易造成积分饱和；
 *   4. 最后再根据画面刷新率调整单步上限和稳定帧数。
 * =============================================================================
 */

typedef struct
{
    float kp;              /* 比例系数：像素误差 -> 本轮位置修正量 */
    float ki;              /* 积分系数；当前配置为 0，字段保留便于后续标定 */
    float kd;              /* 微分系数：抑制相邻图像误差的快速跳变 */
    float integral;        /* 像素误差随时间的积分，单位 px*s */
    float integral_limit;  /* 积分绝对值上限，防止长时间在限位处累积 */
    float previous_error;  /* 上一有效图像帧的误差，用于微分项 */
    uint32_t previous_ms;  /* 上一有效帧时刻，用真实帧间隔计算微分 */
    bool previous_valid;   /* false 表示首帧：首帧不计算微分，避免启动尖峰 */
} RawPositionPid_t;

/*
 * stop 握手有两种用途：
 *   COMPLETE     对准已由 STM32 或 K230 判定成功，确认停止后交给机械臂夹取；
 *   RECOVER_BUSY K230 仍卡在旧颜色任务，先清旧任务，再自动重发当前 detect。
 * 把两条路径收敛到同一个状态机，避免在多个分支里散落“发一次 stop 就算了”。
 */
typedef enum
{
    RAW_STOP_PURPOSE_NONE = 0,
    RAW_STOP_PURPOSE_COMPLETE,
    RAW_STOP_PURPOSE_RECOVER_BUSY
} RawStopPurpose_t;

static RawMaterialAlignState_t s_state = RAW_ALIGN_IDLE;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_offset_epoch_ms = 0U;
static uint8_t s_color_id = 0U;
static uint8_t s_color_ok = 0U;
static uint8_t s_grab_seen = 0U;
static uint8_t s_stable_frames = 0U;
static uint16_t s_iteration = 0U;
static RawStopPurpose_t s_stop_purpose = RAW_STOP_PURPOSE_NONE;
static uint8_t s_pending_exit_reason = RAW_ALIGN_EXIT_NONE;
static uint32_t s_stop_last_tx_ms = 0U;
/* 当前“已下发的绝对目标”。对准完成后由 ZoneTask 保存并用于正式抓取。 */
static float s_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
static float s_slide_mm = RAW_VISION_SLIDE_START_MM;

/* 本轮是否真的改变了对应执行器；未改变的轴不应阻塞 MOVING 状态。 */
static bool s_wait_carousel = false;
static bool s_wait_slide = false;

static RawPositionPid_t s_angle_pid;
static RawPositionPid_t s_slide_pid;

volatile uint8_t robot_raw_align_state = RAW_ALIGN_IDLE;
volatile uint8_t robot_raw_align_error = RAW_ALIGN_ERROR_NONE;
volatile uint8_t robot_raw_align_exit = RAW_ALIGN_EXIT_NONE;
volatile uint8_t robot_raw_align_color_ok = 0U;
volatile uint8_t robot_raw_align_grab = 0U;
volatile uint8_t robot_raw_align_stable_frames = 0U;
volatile uint16_t robot_raw_align_iteration = 0U;
volatile int32_t robot_raw_align_px_x = 0;
volatile int32_t robot_raw_align_px_y = 0;
volatile float robot_raw_align_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
volatile float robot_raw_align_slide_mm = RAW_VISION_SLIDE_START_MM;
volatile float robot_raw_align_angle_step_deg = 0.0f;
volatile float robot_raw_align_slide_step_mm = 0.0f;
volatile uint8_t robot_raw_align_angle_limit = RAW_ALIGN_LIMIT_NONE;
volatile uint8_t robot_raw_align_slide_limit = RAW_ALIGN_LIMIT_NONE;
volatile uint8_t robot_raw_align_have_x = 0U;
volatile uint8_t robot_raw_align_have_y = 0U;
volatile uint16_t robot_raw_align_detect_requests = 0U;
volatile uint16_t robot_raw_align_stop_requests = 0U;
volatile uint16_t robot_raw_align_stop_tx_failures = 0U;
volatile uint32_t robot_raw_align_stop_wait_ms = 0U;
volatile uint32_t robot_raw_align_stop_ack_ms = 0U;
volatile uint8_t robot_raw_align_stop_ack_received = 0U;
volatile uint16_t robot_raw_align_busy_recoveries = 0U;

static float raw_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint32_t raw_abs_px(int32_t value)
{
    return (uint32_t)((value < 0) ? -value : value);
}

static void pid_init(RawPositionPid_t *pid,
                     float kp, float ki, float kd, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->previous_error = 0.0f;
    pid->previous_ms = 0U;
    pid->previous_valid = false;
}

/*
 * 输出是下一次位置修正量，不是速度。导数项用帧间真实 dt；首帧不计算
 * 微分，长时间丢帧时将 dt 上限钳到 1 s，避免恢复瞬间的异常尖峰。
 */
static float pid_update(RawPositionPid_t *pid, float error, uint32_t now_ms)
{
    float dt = 0.0f;
    float derivative = 0.0f;
    float output;

    if (pid->previous_valid)
    {
        dt = (float)(now_ms - pid->previous_ms) * 0.001f;
        dt = raw_clampf(dt, 0.02f, 1.0f);
        pid->integral += error * dt;
        pid->integral = raw_clampf(pid->integral,
                                   -pid->integral_limit,
                                    pid->integral_limit);
        derivative = (error - pid->previous_error) / dt;
    }

    output = pid->kp * error +
             pid->ki * pid->integral +
             pid->kd * derivative;

    pid->previous_error = error;
    pid->previous_ms = now_ms;
    pid->previous_valid = true;
    return output;
}

static void enter_state(RawMaterialAlignState_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;
    robot_raw_align_state = (uint8_t)state;

    if (state == RAW_ALIGN_WAIT_OFFSET)
    {
        /* 只使用执行器停稳后收到的新图像。 */
        s_offset_epoch_ms = now_ms;
    }
}

static bool axis_line_usable(char direction, uint32_t line_ms)
{
    if (direction == '\0')
    {
        return false;
    }

    return ((int32_t)(line_ms - s_offset_epoch_ms) >= 0);
}

static bool color_ready(void)
{
#if K230_ALIGN_REQUIRE_COLOR
    return s_color_ok != 0U;
#else
    return true;
#endif
}

/* request_next_frame 会在 busy 恢复握手完成后被调用，先做本地前置声明。 */
static bool request_next_frame(uint32_t now_ms);

static void raw_align_fail(uint8_t error, uint32_t now_ms)
{
    (void)K230_Stop();
    robot_raw_align_error = error;
    robot_raw_align_exit = RAW_ALIGN_EXIT_FAIL;
    enter_state(RAW_ALIGN_ERROR, now_ms);
}

static void send_stop_request(uint32_t now_ms)
{
    /*
     * 即使本次 HAL_UART_Transmit 返回忙，也不立刻放弃：STOPPING 会按固定
     * 周期重试。计数表示尝试次数，实车若经常大于1可直接定位串口丢包。
     */
    if (K230_Stop() != HAL_OK)
    {
        if (robot_raw_align_stop_tx_failures < 65535U)
        {
            robot_raw_align_stop_tx_failures++;
        }
    }
    s_stop_last_tx_ms = now_ms;
    if (robot_raw_align_stop_requests < 65535U)
    {
        robot_raw_align_stop_requests++;
    }
}

static void begin_stop_handshake(RawStopPurpose_t purpose,
                                 uint8_t exit_reason,
                                 uint32_t now_ms)
{
    s_stop_purpose = purpose;
    s_pending_exit_reason = exit_reason;
    robot_raw_align_stop_tx_failures = 0U;
    robot_raw_align_stop_wait_ms = 0U;
    robot_raw_align_stop_ack_ms = 0U;
    robot_raw_align_stop_ack_received = 0U;
    enter_state(RAW_ALIGN_STOPPING, now_ms);
    send_stop_request(now_ms);
}

static void raw_align_done(uint8_t exit_reason, uint32_t now_ms)
{
    /*
     * “谁判定对准”保持原逻辑：K230 的 grab 或 STM32 连续容差帧都可到这里。
     * 这里只改变任务交接方式——收到 stopped 前不允许 ZoneTask 开始夹取，
     * 防止 K230 仍停留在绿色任务而拒绝下一块物料。
     */
    begin_stop_handshake(RAW_STOP_PURPOSE_COMPLETE,
                         exit_reason,
                         now_ms);
}

static bool can_start(void)
{
    return (s_state == RAW_ALIGN_IDLE) ||
           (s_state == RAW_ALIGN_DONE) ||
           (s_state == RAW_ALIGN_ERROR);
}

/*
 * 每消费完一帧、且执行器已经停稳时，立即主动请求下一帧。
 *
 * 实车 K230 更接近“一条 detect 返回一组结果”，并不会保证持续推流。旧版
 * 只靠 RAW_VISION_RESEND_MS 的 1 s 丢包重试取得下一帧，所以每次机械调整
 * 后都会明显停顿。现在正常闭环在这里立即重发；1 s 定时器只在本次命令
 * 丢失或 K230 没响应时兜底。
 */
static bool request_next_frame(uint32_t now_ms)
{
    /* 先建立新帧时间界，再发命令，避免极快返回的报文被误判成旧帧。 */
    robot_raw_align_have_x = 0U;
    robot_raw_align_have_y = 0U;
    enter_state(RAW_ALIGN_WAIT_OFFSET, now_ms);

    if (K230_StartDetect(s_color_id) != HAL_OK)
    {
        raw_align_fail(RAW_ALIGN_ERROR_SEND, now_ms);
        return false;
    }

    if (robot_raw_align_detect_requests < 65535U)
    {
        robot_raw_align_detect_requests++;
    }
    return true;
}

static void handle_stop_handshake(uint32_t events, uint32_t now_ms)
{
    robot_raw_align_stop_wait_ms = now_ms - s_state_start_ms;

    if ((events & K230_EVENT_STOPPED) != 0U)
    {
        robot_raw_align_stop_ack_ms = robot_raw_align_stop_wait_ms;
        robot_raw_align_stop_ack_received = 1U;

        if (s_stop_purpose == RAW_STOP_PURPOSE_COMPLETE)
        {
            robot_raw_align_exit = s_pending_exit_reason;
            s_stop_purpose = RAW_STOP_PURPOSE_NONE;
            enter_state(RAW_ALIGN_DONE, now_ms);
            return;
        }

        if (s_stop_purpose == RAW_STOP_PURPOSE_RECOVER_BUSY)
        {
            /*
             * busy 说明 K230 保存的是另一条旧任务。stopped 到达后必须清空
             * 旧颜色/旧 grab/连续帧计数，再从当前 s_color_id 重新开始；否则
             * 旧绿色帧可能被误当成下一块物料的有效结果。
             */
            s_stop_purpose = RAW_STOP_PURPOSE_NONE;
            s_color_ok = 0U;
            s_grab_seen = 0U;
            s_stable_frames = 0U;
            robot_raw_align_color_ok = 0U;
            robot_raw_align_grab = 0U;
            robot_raw_align_stable_frames = 0U;
            if (robot_raw_align_busy_recoveries < 65535U)
            {
                robot_raw_align_busy_recoveries++;
            }
            (void)request_next_frame(now_ms);
            return;
        }

        raw_align_fail(RAW_ALIGN_ERROR_PARAM, now_ms);
        return;
    }

    /*
     * STOPPING 期间可能还会收到旧任务排队中的 error,busy、偏差或颜色帧；
     * 它们都不再属于新的视觉周期，只等 stopped。确认丢包就按周期
     * 继续重发 stop，不再按固定时间退出。迟到的 stopped 仍可以恢复流程；
     * 链路完全断开时则安全停在 STOPPING，不会越过确认去抓取。
     */

    if ((now_ms - s_stop_last_tx_ms) >= RAW_VISION_STOP_RETRY_MS)
    {
        send_stop_request(now_ms);
    }
}

static void handle_offset_pair(int32_t px_x, int32_t px_y, uint32_t now_ms)
{
    float angle_step = 0.0f;
    float slide_step = 0.0f;
    float angle_requested;
    float slide_requested;
    float angle_target;
    float slide_target;

    robot_raw_align_px_x = px_x;
    robot_raw_align_px_y = px_y;
    robot_raw_align_angle_step_deg = 0.0f;
    robot_raw_align_slide_step_mm = 0.0f;
    robot_raw_align_angle_limit = RAW_ALIGN_LIMIT_NONE;
    robot_raw_align_slide_limit = RAW_ALIGN_LIMIT_NONE;

    /*
     * 不用单帧“碰巧居中”直接收工：圆盘仍在运动、图像也可能抖动，必须
     * 连续 RAW_VISION_STABLE_FRAMES 帧两轴都在容差内才确认。
     */
    if ((raw_abs_px(px_x) <= RAW_VISION_TOLERANCE_PX) &&
        (raw_abs_px(px_y) <= RAW_VISION_TOLERANCE_PX))
    {
        if (s_stable_frames < 255U)
        {
            s_stable_frames++;
        }
        robot_raw_align_stable_frames = s_stable_frames;

        if (s_stable_frames >= RAW_VISION_STABLE_FRAMES)
        {
            raw_align_done(RAW_ALIGN_EXIT_TOLERANCE, now_ms);
        }
        else
        {
            (void)request_next_frame(now_ms);
        }
        return;
    }

    s_stable_frames = 0U;
    robot_raw_align_stable_frames = 0U;

    /*
     * 两个环独立计算，但同一轮同时下发目标，因此斜向误差无需先调一轴、
     * 再调另一轴。容差内的轴本轮保持不动，避免无意义的小幅机械抖动。
     */
    if (raw_abs_px(px_y) > RAW_VISION_TOLERANCE_PX)
    {
        angle_step = RAW_VISION_ANGLE_SIGN *
                     pid_update(&s_angle_pid, (float)px_y, now_ms);
        angle_step = raw_clampf(angle_step,
                                -RAW_VISION_ANGLE_STEP_MAX_DEG,
                                 RAW_VISION_ANGLE_STEP_MAX_DEG);
    }

    if (raw_abs_px(px_x) > RAW_VISION_TOLERANCE_PX)
    {
        slide_step = RAW_VISION_SLIDE_SIGN *
                     pid_update(&s_slide_pid, (float)px_x, now_ms);
        slide_step = raw_clampf(slide_step,
                                -RAW_VISION_SLIDE_STEP_MAX_MM,
                                 RAW_VISION_SLIDE_STEP_MAX_MM);
    }

    /*
     * PID 给的是增量，执行器接口要的是绝对目标。先保留未钳位请求用于
     * Watch诊断，再得到真正下发的安全目标；到限位时不报电机故障。
     */
    angle_requested = s_carousel_deg + angle_step;
    slide_requested = s_slide_mm + slide_step;

    angle_target = raw_clampf(angle_requested,
                              RAW_VISION_CAROUSEL_MIN_DEG,
                              RAW_VISION_CAROUSEL_MAX_DEG);
    slide_target = raw_clampf(slide_requested,
                              RAW_VISION_SLIDE_MIN_MM,
                              RAW_VISION_SLIDE_MAX_MM);

    robot_raw_align_angle_limit =
        (angle_requested < RAW_VISION_CAROUSEL_MIN_DEG) ?
        RAW_ALIGN_LIMIT_MIN :
        ((angle_requested > RAW_VISION_CAROUSEL_MAX_DEG) ?
         RAW_ALIGN_LIMIT_MAX : RAW_ALIGN_LIMIT_NONE);
    robot_raw_align_slide_limit =
        (slide_requested < RAW_VISION_SLIDE_MIN_MM) ?
        RAW_ALIGN_LIMIT_MIN :
        ((slide_requested > RAW_VISION_SLIDE_MAX_MM) ?
         RAW_ALIGN_LIMIT_MAX : RAW_ALIGN_LIMIT_NONE);

    robot_raw_align_angle_step_deg = angle_target - s_carousel_deg;
    robot_raw_align_slide_step_mm = slide_target - s_slide_mm;

    s_wait_carousel = (angle_target != s_carousel_deg);
    s_wait_slide = (slide_target != s_slide_mm);

    if (s_wait_slide &&
        !ManipulatorAction_SlideJogToPositionStart(slide_target))
    {
        raw_align_fail(RAW_ALIGN_ERROR_SLIDE, now_ms);
        return;
    }

    if (s_wait_carousel)
    {
        CarouselServo_SetAngle(angle_target);
    }

    s_carousel_deg = angle_target;
    s_slide_mm = slide_target;
    robot_raw_align_carousel_deg = s_carousel_deg;
    robot_raw_align_slide_mm = s_slide_mm;

    if (s_wait_carousel || s_wait_slide)
    {
        if (s_iteration < 65535U)
        {
            s_iteration++;
        }
        robot_raw_align_iteration = s_iteration;
        enter_state(RAW_ALIGN_MOVING, now_ms);
    }
    else
    {
        /* 到达软件安全边界但尚未对准：继续观察旋转圆盘，不报假故障。 */
        (void)request_next_frame(now_ms);
    }
}

void RawMaterialAlign_Init(void)
{
    s_state = RAW_ALIGN_IDLE;
    s_state_start_ms = 0U;
    s_offset_epoch_ms = 0U;
    s_color_id = 0U;
    s_color_ok = 0U;
    s_grab_seen = 0U;
    s_stable_frames = 0U;
    s_iteration = 0U;
    s_stop_purpose = RAW_STOP_PURPOSE_NONE;
    s_pending_exit_reason = RAW_ALIGN_EXIT_NONE;
    s_stop_last_tx_ms = 0U;
    s_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
    s_slide_mm = RAW_VISION_SLIDE_START_MM;
    s_wait_carousel = false;
    s_wait_slide = false;

    pid_init(&s_angle_pid,
             RAW_VISION_ANGLE_KP,
             RAW_VISION_ANGLE_KI,
             RAW_VISION_ANGLE_KD,
             RAW_VISION_ANGLE_I_LIMIT_PX_S);
    pid_init(&s_slide_pid,
             RAW_VISION_SLIDE_KP,
             RAW_VISION_SLIDE_KI,
             RAW_VISION_SLIDE_KD,
             RAW_VISION_SLIDE_I_LIMIT_PX_S);

    robot_raw_align_state = RAW_ALIGN_IDLE;
    robot_raw_align_error = RAW_ALIGN_ERROR_NONE;
    robot_raw_align_exit = RAW_ALIGN_EXIT_NONE;
    robot_raw_align_color_ok = 0U;
    robot_raw_align_grab = 0U;
    robot_raw_align_stable_frames = 0U;
    robot_raw_align_iteration = 0U;
    robot_raw_align_px_x = 0;
    robot_raw_align_px_y = 0;
    robot_raw_align_carousel_deg = s_carousel_deg;
    robot_raw_align_slide_mm = s_slide_mm;
    robot_raw_align_angle_step_deg = 0.0f;
    robot_raw_align_slide_step_mm = 0.0f;
    robot_raw_align_angle_limit = RAW_ALIGN_LIMIT_NONE;
    robot_raw_align_slide_limit = RAW_ALIGN_LIMIT_NONE;
    robot_raw_align_have_x = 0U;
    robot_raw_align_have_y = 0U;
    robot_raw_align_detect_requests = 0U;
    robot_raw_align_stop_requests = 0U;
    robot_raw_align_stop_tx_failures = 0U;
    robot_raw_align_stop_wait_ms = 0U;
    robot_raw_align_stop_ack_ms = 0U;
    robot_raw_align_stop_ack_received = 0U;
    robot_raw_align_busy_recoveries = 0U;
}

bool RawMaterialAlign_Start(uint8_t color_id, uint32_t now_ms)
{
    /*
     * Start 不负责机械预置。调用者必须已经完成：云台 POS2、夹爪张开、
     * 滑台到 RAW_VISION_SLIDE_START_MM。这里只检查滑台位置是否仍可信，
     * 然后以实际软件记录的位置作为径向环起点。
     */
    if (!can_start() ||
        (color_id < 1U) || (color_id > 6U) ||
        !ManipulatorAction_SlideJogPositionKnown() ||
        ManipulatorAction_SlideJogIsBusy())
    {
        return false;
    }

    s_color_id = color_id;
    s_color_ok = 0U;
    s_grab_seen = 0U;
    s_stable_frames = 0U;
    s_iteration = 0U;
    s_stop_purpose = RAW_STOP_PURPOSE_NONE;
    s_pending_exit_reason = RAW_ALIGN_EXIT_NONE;
    s_stop_last_tx_ms = 0U;
    s_carousel_deg = K230_ALIGN_CAROUSEL_ANGLE_DEG;
    s_slide_mm = ManipulatorAction_SlideJogGetPositionMm();
    s_wait_carousel = false;
    s_wait_slide = false;

    pid_init(&s_angle_pid,
             RAW_VISION_ANGLE_KP,
             RAW_VISION_ANGLE_KI,
             RAW_VISION_ANGLE_KD,
             RAW_VISION_ANGLE_I_LIMIT_PX_S);
    pid_init(&s_slide_pid,
             RAW_VISION_SLIDE_KP,
             RAW_VISION_SLIDE_KI,
             RAW_VISION_SLIDE_KD,
             RAW_VISION_SLIDE_I_LIMIT_PX_S);

    robot_raw_align_error = RAW_ALIGN_ERROR_NONE;
    robot_raw_align_exit = RAW_ALIGN_EXIT_NONE;
    robot_raw_align_color_ok = 0U;
    robot_raw_align_grab = 0U;
    robot_raw_align_stable_frames = 0U;
    robot_raw_align_iteration = 0U;
    robot_raw_align_px_x = 0;
    robot_raw_align_px_y = 0;
    robot_raw_align_carousel_deg = s_carousel_deg;
    robot_raw_align_slide_mm = s_slide_mm;
    robot_raw_align_angle_step_deg = 0.0f;
    robot_raw_align_slide_step_mm = 0.0f;
    robot_raw_align_angle_limit = RAW_ALIGN_LIMIT_NONE;
    robot_raw_align_slide_limit = RAW_ALIGN_LIMIT_NONE;
    robot_raw_align_have_x = 0U;
    robot_raw_align_have_y = 0U;
    robot_raw_align_detect_requests = 0U;
    robot_raw_align_stop_requests = 0U;
    robot_raw_align_stop_tx_failures = 0U;
    robot_raw_align_stop_wait_ms = 0U;
    robot_raw_align_stop_ack_ms = 0U;
    robot_raw_align_stop_ack_received = 0U;
    robot_raw_align_busy_recoveries = 0U;

    /* 清掉上一块物料的残帧，再立即请求本轮第一帧。 */
    (void)K230_TakeEvents();
    if (!request_next_frame(now_ms))
    {
        return false;
    }
    return true;
}

void RawMaterialAlign_Update(uint32_t now_ms)
{
    uint32_t events;
    bool have_x;
    bool have_y;
    int32_t px_x;
    int32_t px_y;
    bool carousel_done;
    bool slide_done;

    if (!RawMaterialAlign_IsBusy())
    {
        return;
    }

    /*
     * 本模块忙时独占 K230 事件消费；通用 K230Align 此时一定不在运行。
     * g_k230_status 保存各轴最近值，events 只表示本周期出现了哪些新消息。
     */
    events = K230_TakeEvents();

    /* STOPPING 独占协议事件：在确认返回前，旧颜色/偏差/grab 一律丢弃。 */
    if (s_state == RAW_ALIGN_STOPPING)
    {
        handle_stop_handshake(events, now_ms);
        return;
    }

    if ((events & K230_EVENT_ERROR) != 0U)
    {
        if (strcmp(g_k230_status.error_text, "busy") == 0)
        {
            /*
             * 不再让一次旧任务残留直接杀死整套 ZoneTask。先可靠停止 K230，
             * 收到 stopped 后 handle_stop_handshake 会重发当前颜色 detect。
             */
            begin_stop_handshake(RAW_STOP_PURPOSE_RECOVER_BUSY,
                                 RAW_ALIGN_EXIT_NONE,
                                 now_ms);
        }
        else
        {
            raw_align_fail(RAW_ALIGN_ERROR_REMOTE, now_ms);
        }
        return;
    }

    if (((events & K230_EVENT_COLOR) != 0U) &&
        (g_k230_status.color_id == s_color_id))
    {
        s_color_ok = 1U;
        robot_raw_align_color_ok = 1U;
    }

    if ((events & K230_EVENT_GRAB) != 0U)
    {
        s_grab_seen = 1U;
        robot_raw_align_grab = 1U;
    }

    switch (s_state)
    {
        case RAW_ALIGN_WAIT_OFFSET:
            /*
             * 收工信号优先于偏差：K230 可能在最后一帧同时发送偏差和 grab，
             * 若先处理偏差就会启动下一次机械移动，并把有效 grab 当旧帧清掉。
             * grab 的协议含义是 K230 已判断可抓；没有 grab 时仍可依靠连续
             * 居中帧正常结束。
             */
            if (color_ready() && s_grab_seen)
            {
                raw_align_done(RAW_ALIGN_EXIT_GRAB, now_ms);
                break;
            }

            have_x = axis_line_usable(g_k230_status.axis_x_dir,
                                      g_k230_status.axis_x_ms);
            have_y = axis_line_usable(g_k230_status.axis_y_dir,
                                      g_k230_status.axis_y_ms);
            robot_raw_align_have_x = have_x ? 1U : 0U;
            robot_raw_align_have_y = have_y ? 1U : 0U;

            px_x = have_x ? ((g_k230_status.axis_x_dir == 'w') ?
                             g_k230_status.axis_x_px :
                             -g_k230_status.axis_x_px) : 0;
            px_y = have_y ? ((g_k230_status.axis_y_dir == 'a') ?
                             g_k230_status.axis_y_px :
                             -g_k230_status.axis_y_px) : 0;

            if (color_ready() &&
                ((have_x && have_y) ||
                 ((have_x || have_y) &&
                  ((now_ms - s_state_start_ms) >= RAW_VISION_PAIR_WINDOW_MS))))
            {
                s_grab_seen = 0U;
                robot_raw_align_grab = 0U;
                handle_offset_pair(px_x, px_y, now_ms);
                break;
            }

            if ((now_ms - s_state_start_ms) >= RAW_VISION_RESEND_MS)
            {
                if (K230_StartDetect(s_color_id) != HAL_OK)
                {
                    raw_align_fail(RAW_ALIGN_ERROR_SEND, now_ms);
                    break;
                }
                if (robot_raw_align_detect_requests < 65535U)
                {
                    robot_raw_align_detect_requests++;
                }
                s_state_start_ms = now_ms;
            }
            break;

        case RAW_ALIGN_MOVING:
            /* 两个位置执行器可以并行运动，但必须全部完成后才能重新取像。 */
            if (s_wait_slide && ManipulatorAction_SlideJogHasError())
            {
                raw_align_fail(RAW_ALIGN_ERROR_SLIDE, now_ms);
                break;
            }

            carousel_done = (!s_wait_carousel) || CarouselServo_IsDone();
            slide_done = (!s_wait_slide) || ManipulatorAction_SlideJogIsDone();

            if (carousel_done && slide_done)
            {
                /* 执行器移动时的图像已经过期，稳定后重新建立收帧时间界。 */
                s_grab_seen = 0U;
                robot_raw_align_grab = 0U;
                enter_state(RAW_ALIGN_SETTLE, now_ms);
            }
            break;

        case RAW_ALIGN_SETTLE:
            /* 舵机“到位”和结构完全静止不是一回事，留稳定时间再建立新帧界。 */
            if ((now_ms - s_state_start_ms) >= RAW_VISION_SETTLE_MS)
            {
                (void)request_next_frame(now_ms);
            }
            break;

        default:
            raw_align_fail(RAW_ALIGN_ERROR_PARAM, now_ms);
            break;
    }
}

void RawMaterialAlign_Abort(void)
{
    if (!RawMaterialAlign_IsBusy())
    {
        return;
    }

    (void)K230_Stop();
    (void)K230_TakeEvents();
    s_stop_purpose = RAW_STOP_PURPOSE_NONE;
    robot_raw_align_error = RAW_ALIGN_ERROR_ABORTED;
    robot_raw_align_exit = RAW_ALIGN_EXIT_ABORT;
    enter_state(RAW_ALIGN_ERROR, HAL_GetTick());
}

bool RawMaterialAlign_IsBusy(void)
{
    return (s_state != RAW_ALIGN_IDLE) &&
           (s_state != RAW_ALIGN_DONE) &&
           (s_state != RAW_ALIGN_ERROR);
}

bool RawMaterialAlign_IsDone(void)
{
    return s_state == RAW_ALIGN_DONE;
}

bool RawMaterialAlign_HasError(void)
{
    return s_state == RAW_ALIGN_ERROR;
}

float RawMaterialAlign_GetCarouselDeg(void)
{
    return s_carousel_deg;
}

float RawMaterialAlign_GetSlideMm(void)
{
    return s_slide_mm;
}

uint16_t RawMaterialAlign_GetIteration(void)
{
    return s_iteration;
}

uint8_t RawMaterialAlign_GetError(void)
{
    return robot_raw_align_error;
}
