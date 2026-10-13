#include "k230_align.h"

#include "chassis_motion.h"
#include "k230.h"
#include "robot_config.h"
#include "servo.h"

#include <math.h>

/*
 * K230 视觉对位状态机。
 *
 * 与 zone_task / material_load_task 同构：Start 只调用一次，
 * Update 由主循环持续调用，全部非阻塞。
 *
 * 偏差帧来自 k230.c 的 a,18 / d,7 / w,6 / s,10 行。K230 现在一帧发
 * 两行，一行属于 ±X 轴（w/s）、一行属于 ±Y 轴（a/d），两行合成为同一次
 * 斜向平移。k230.c 已按轴分别缓存（g_k230_status.axis_*_dir/_px/_ms），
 * 本模块直接按时间戳取本轮的那一对，因此不依赖 K230 的任务层，也不怕
 * 两行落在同一个主循环周期里互相覆盖。
 * （对位不使用 K230_TaskStartDetect 的“等 grab”模型：那条路径在
 *  grab 之后就不再消费新事件，拿不到后续的偏差帧。）
 */

float K230_ALIGN_MM_PER_PX = 1.0f;

static Chassis *s_chassis = NULL;

static K230AlignState_t s_state = K230_ALIGN_IDLE;
static uint32_t s_state_start_ms = 0U;

static K230AlignMode_t s_mode = K230_ALIGN_MODE_DETECT;
static uint8_t s_target_id = 0U;
static uint16_t s_carousel_us = K230_ALIGN_CAROUSEL_POS_US; /* 对位时的夹爪朝向 */
static uint8_t s_iteration = 0U;
static uint8_t s_color_ok = 0U;   /* DETECT：已确认 K230 识别到目标颜色 */
static uint8_t s_grab_seen = 0U;  /* DETECT：已收到 K230 的 grab（识别成功可直接夹） */
static uint8_t s_place_seen = 0U; /* PLACE：已收到 K230 的 place（已对准放置位） */

/*
 * 本轮偏差的收集起点（HAL_GetTick()）。
 * 只认这个时刻之后到达的轴偏差行，微调过程中和上一轮遗留的旧值都不会
 * 跨轮混进来——每次进 WAIT_OFFSET 把起点推到当前时刻即可，不必在每条
 * 离开分支里逐个清标志。初值 0 表示“开机以来收到的都算”，
 * 第一次进 WAIT_OFFSET 就会被覆盖。
 */
static uint32_t s_offset_epoch_ms = 0U;

/* 诊断量：Keil Watch 可观察。 */
volatile uint8_t robot_k230_align_mode = (uint8_t)K230_ALIGN_MODE_DETECT;
volatile uint8_t robot_k230_align_state = K230_ALIGN_IDLE;
volatile uint8_t robot_k230_align_iteration = 0U;
volatile uint8_t robot_k230_align_error = K230_ALIGN_ERROR_NONE;
volatile uint8_t robot_k230_align_exit = K230_ALIGN_EXIT_NONE;
volatile uint8_t robot_k230_align_color_ok = 0U;
volatile uint8_t robot_k230_align_grab = 0U;
volatile uint8_t robot_k230_align_place = 0U;
volatile char    robot_k230_align_last_dir = '\0';
volatile int32_t robot_k230_align_last_px = 0;
volatile int32_t robot_k230_align_px_x = 0;  /* 本轮合成用的 ±X 像素偏差，带符号 */
volatile int32_t robot_k230_align_px_y = 0;  /* 本轮合成用的 ±Y 像素偏差，带符号 */

static void enter_state(K230AlignState_t new_state, uint32_t now_ms)
{
    s_state = new_state;
    s_state_start_ms = now_ms;
    robot_k230_align_state = (uint8_t)new_state;

    /*
     * 每次进“等偏差”都以当前时刻为界重新收偏差。
     * 微调期间和上一轮收到的轴值时间戳早于这个界，自然失效，
     * 不用在每个离开分支里逐个清标志。
     */
    if (new_state == K230_ALIGN_WAIT_OFFSET)
    {
        s_offset_epoch_ms = now_ms;
    }
}

static void align_fail(uint8_t error_code, uint32_t now_ms)
{
    robot_k230_align_error = error_code;
    /* 结束方式统一记 FAIL，具体原因在 robot_k230_align_error。 */
    robot_k230_align_exit = K230_ALIGN_EXIT_FAIL;

    if (s_chassis != NULL)
    {
        chassis_motion_stop(s_chassis);
    }

    enter_state(K230_ALIGN_ERROR, now_ms);
}

static bool align_can_start(void)
{
    return (s_state == K230_ALIGN_IDLE) ||
           (s_state == K230_ALIGN_DONE) ||
           (s_state == K230_ALIGN_ERROR);
}

/*
 * 按当前模式发出启动命令：
 *   DETECT -> detect,<颜色号>   原料区去原料台找料
 *   PLACE  -> place,<位置号>    粗加工区/暂存区对准放置位
 * 只在 ROTATE 状态确认夹爪已就位之后调用，保证图像朝向已固定。
 */
static HAL_StatusTypeDef send_align_command(void)
{
    if (s_mode == K230_ALIGN_MODE_PLACE)
    {
        return K230_StartPlace(s_target_id);
    }

    return K230_StartDetect(s_target_id);
}

/* DETECT 用颜色号 1~6；PLACE 用位置号 1~3。 */
static bool target_id_valid(K230AlignMode_t mode, uint8_t target_id)
{
    if (mode == K230_ALIGN_MODE_PLACE)
    {
        return (target_id >= 1U) && (target_id <= 3U);
    }

    return (target_id >= 1U) && (target_id <= 6U);
}

/* 偏差像素取绝对值。 */
static uint32_t abs_px(int32_t px)
{
    return (uint32_t)((px < 0) ? (-px) : px);
}

/*
 * 判断某轴最新一行偏差是否属于本轮。
 * dir == '\0' 表示该轴还没收到过任何偏差行。
 * 时间戳用有符号差比较，避免 HAL_GetTick() 回绕时判错。
 */
static bool axis_line_usable(char dir, uint32_t line_ms)
{
    if (dir == '\0')
    {
        return false;
    }

    return ((int32_t)(line_ms - s_offset_epoch_ms) >= 0);
}

/*
 * 把一对正交偏差换算成车体位移（米），并做单次位移钳位。
 *
 * 符号约定（方向映射来自“夹取姿态下夹爪正向”到车体系的固定关系）：
 *   w 夹爪前 -> 车体右(+X)    s 夹爪后 -> 车体左(-X)
 *   a 夹爪左 -> 车体前(+Y)    d 夹爪右 -> 车体后(-Y)
 * k230.c 已把方向字符折算到这两根轴上：px_x > 0 是车体右(+X)，
 * px_y > 0 是车体前(+Y)。车朝偏差所指方向平移，即把目标移回夹爪正前方。
 *
 * 钳位作用在合成矢量的长度上。两个分量各自钳位会让斜向一步走出
 * sqrt(2) 倍于上限，正好是这道保险要防的情况；只有一个分量时，
 * 矢量钳位与旧版的单轴钳位完全等价。
 */
static void offset_to_body(int32_t px_x, int32_t px_y,
                           float *dx_m, float *dy_m)
{
    float dx = (float)px_x * K230_ALIGN_MM_PER_PX;
    float dy = (float)px_y * K230_ALIGN_MM_PER_PX;
    float len = sqrtf(dx * dx + dy * dy);

    if (len > K230_ALIGN_STEP_MAX_MM)
    {
        float scale = K230_ALIGN_STEP_MAX_MM / len;
        dx *= scale;
        dy *= scale;
    }

    *dx_m = dx * 0.001f;
    *dy_m = dy * 0.001f;
}

/*
 * 是否可以消费偏差帧。
 * DETECT 模式要求先确认识别到了目标颜色，避免对着错误颜色的料微调；
 * PLACE 模式没有颜色概念，直接放行。
 * K230_ALIGN_REQUIRE_COLOR 置 0 时，Start 里会直接把 s_color_ok 置 1，
 * 等价于关掉这道校验。
 */
static bool color_ready(void)
{
    if (s_mode != K230_ALIGN_MODE_DETECT)
    {
        return true;
    }

    return s_color_ok != 0U;
}

/*
 * 用一对正交偏差决定：对准了 / 用满次数 / 继续微调。
 * px_x / px_y 是带符号像素值，正方向分别是车体 +X（右）和 +Y（前）；
 * 某轴缺席时传 0，等价于“该轴偏差已在容差内”。
 */
static void handle_offset_pair(int32_t px_x, int32_t px_y, uint32_t now_ms)
{
    float dx_m = 0.0f;
    float dy_m = 0.0f;

    robot_k230_align_px_x = px_x;
    robot_k230_align_px_y = px_y;

    /* 两轴都进容差才算对准；缺席轴为 0，自动满足。 */
    if ((abs_px(px_x) <= K230_ALIGN_TOLERANCE_PX) &&
        (abs_px(px_y) <= K230_ALIGN_TOLERANCE_PX))
    {
        /* 已对准：让 K230 停止检测，否则偏差帧会继续占着 UART4。 */
        (void)K230_Stop();
        robot_k230_align_exit = K230_ALIGN_EXIT_TOLERANCE;

        enter_state(K230_ALIGN_DONE, now_ms);
        return;
    }

    if (s_iteration >= K230_ALIGN_MAX_ITER)
    {
        align_fail(K230_ALIGN_ERROR_ITER, now_ms);
        return;
    }

    offset_to_body(px_x, px_y, &dx_m, &dy_m);

    if ((dx_m == 0.0f) && (dy_m == 0.0f))
    {
        align_fail(K230_ALIGN_ERROR_PARAM, now_ms);
        return;
    }

    chassis_motion_translate(s_chassis, dx_m, dy_m,
                             K230_ALIGN_SPEED_MPS, now_ms);

    s_iteration++;
    robot_k230_align_iteration = s_iteration;

    enter_state(K230_ALIGN_MOVING, now_ms);
}

void K230Align_Init(Chassis *chassis)
{
    s_chassis = chassis;

    s_state = K230_ALIGN_IDLE;
    s_state_start_ms = 0U;
    s_mode = K230_ALIGN_MODE_DETECT;
    s_target_id = 0U;
    s_carousel_us = K230_ALIGN_CAROUSEL_POS_US;
    s_iteration = 0U;
    s_color_ok = 0U;
    s_grab_seen = 0U;
    s_place_seen = 0U;
    s_offset_epoch_ms = 0U;

    robot_k230_align_mode = (uint8_t)K230_ALIGN_MODE_DETECT;
    robot_k230_align_state = K230_ALIGN_IDLE;
    robot_k230_align_iteration = 0U;
    robot_k230_align_error = K230_ALIGN_ERROR_NONE;
    robot_k230_align_exit = K230_ALIGN_EXIT_NONE;
    robot_k230_align_color_ok = 0U;
    robot_k230_align_grab = 0U;
    robot_k230_align_place = 0U;
    robot_k230_align_last_dir = '\0';
    robot_k230_align_last_px = 0;
    robot_k230_align_px_x = 0;
    robot_k230_align_px_y = 0;
}

bool K230Align_Start(K230AlignMode_t mode, uint8_t target_id,
                     uint16_t carousel_us, uint32_t now_ms)
{
    if ((s_chassis == NULL) || (!align_can_start()))
    {
        return false;
    }

    if (!target_id_valid(mode, target_id))
    {
        return false;
    }

    s_mode = mode;
    s_target_id = target_id;
    s_carousel_us = carousel_us;
    s_iteration = 0U;
    s_color_ok = 0U;
    s_grab_seen = 0U;
    s_place_seen = 0U;
    s_offset_epoch_ms = 0U;

#if !K230_ALIGN_REQUIRE_COLOR
    /* 关掉颜色校验：不等颜色帧，收到偏差帧就微调。 */
    if (mode == K230_ALIGN_MODE_DETECT)
    {
        s_color_ok = 1U;
    }
#endif

    robot_k230_align_mode = (uint8_t)mode;
    robot_k230_align_iteration = 0U;
    robot_k230_align_error = K230_ALIGN_ERROR_NONE;
    robot_k230_align_exit = K230_ALIGN_EXIT_NONE;
    robot_k230_align_color_ok = s_color_ok;
    robot_k230_align_grab = 0U;
    robot_k230_align_place = 0U;
    robot_k230_align_last_dir = '\0';
    robot_k230_align_last_px = 0;
    robot_k230_align_px_x = 0;
    robot_k230_align_px_y = 0;

    /* 丢掉上一次对位遗留的事件，避免旧帧被当成本次的新偏差。 */
    (void)K230_TakeEvents();

    /*
     * 摄像头随夹爪旋转，所以先转到约定的基准姿态再开始检测。
     * 舵机是平滑步进的，这里只发目标，转到位由 ROTATE 状态确认，
     * 期间不发 detect/place——否则 K230 会在夹爪还在转的时候给出偏差。
     */
    CarouselServo_SetUs(s_carousel_us);

    /*
     * 同时张开夹爪：合拢的夹爪会挡住装在夹爪上的摄像头。
     * 用 Large 是为了和取料姿态完全一致——ManipulatorAction 的
     * MANIP_GROUND_TO_TRAY_PREPARE 同样是"转到取料方向 + Open_Large()"，
     * 这样对位看到的画面就是真正取料时看到的画面。
     * 两处必须一致，改一处要同步改另一处。
     */
    GripperServo_Open_Large();

    enter_state(K230_ALIGN_ROTATE, now_ms);
    return true;
}

void K230Align_Update(uint32_t now_ms)
{
    uint32_t events;
    uint8_t  have_x = 0U;
    uint8_t  have_y = 0U;
    int32_t  px_x = 0;
    int32_t  px_y = 0;

    if ((s_state == K230_ALIGN_IDLE) ||
        (s_state == K230_ALIGN_DONE) ||
        (s_state == K230_ALIGN_ERROR))
    {
        return;
    }

    events = K230_TakeEvents();

    /* K230 主动报错（error,xxx）：任何状态下都立即失败。 */
    if ((events & K230_EVENT_ERROR) != 0U)
    {
        align_fail(K230_ALIGN_ERROR_REMOTE, now_ms);
        return;
    }

    /*
     * DETECT 模式：确认 K230 报回的颜色就是本轮要找的颜色。
     * g_k230_status.color_id 是持久字段，配合 COLOR 事件位才表示“刚报上来的”。
     */
    if ((s_mode == K230_ALIGN_MODE_DETECT) &&
        ((events & K230_EVENT_COLOR) != 0U) &&
        (g_k230_status.color_id == s_target_id) &&
        (s_color_ok == 0U))
    {
        s_color_ok = 1U;
        robot_k230_align_color_ok = 1U;
    }

    /*
     * DETECT：K230 回 grab = “已识别到目标、可以抓”。
     * 对位这条路不走 K230_TaskStartDetect，grab 得在这里自己接住。
     * 有些固件识别成功时只回 grab，不给单独的颜色行、也不给偏差帧；
     * 那就只能靠这条路径收工，否则会一直空等到超时报错，
     * 现象就是“K230 已经识别到了，但机械臂不夹”。
     */
    if ((s_mode == K230_ALIGN_MODE_DETECT) &&
        ((events & K230_EVENT_GRAB) != 0U) &&
        (s_grab_seen == 0U))
    {
        s_grab_seen = 1U;
        robot_k230_align_grab = 1U;
    }

    /*
     * PLACE：K230 回 place = “已对准放置位、无需微调”。
     * 与 DETECT 的 grab 对称，是该模式下除“偏差进容差”外的另一条收工路。
     * PLACE 模式不报颜色，所以没有前置校验。
     */
    if ((s_mode == K230_ALIGN_MODE_PLACE) &&
        ((events & K230_EVENT_PLACE) != 0U) &&
        (s_place_seen == 0U))
    {
        s_place_seen = 1U;
        robot_k230_align_place = 1U;
    }

    /*
     * 偏差帧不在这里缓存：轴值由 k230.c 按轴分别保存（axis_*_dir/_px/_ms），
     * WAIT_OFFSET 直接按时间戳取本轮那一对。这里只留一份原始行的诊断镜像。
     */
    if ((events & K230_EVENT_OFFSET) != 0U)
    {
        robot_k230_align_last_dir = g_k230_status.direction;
        robot_k230_align_last_px = g_k230_status.offset_px;
    }

    switch (s_state)
    {
        case K230_ALIGN_ROTATE:
            if ((CarouselServo_IsDone() && GripperServo_IsDone()) ||
                ((now_ms - s_state_start_ms) >= K230_ALIGN_ROTATE_TIMEOUT_MS))
            {
                /* 转动期间可能进来的残帧不算数：进 WAIT_OFFSET 时会把
                 * 收集起点推到那一刻，早于它的轴值自动失效。 */
                s_grab_seen = 0U;
                s_place_seen = 0U;
                (void)K230_TakeEvents();

                if (send_align_command() != HAL_OK)
                {
                    align_fail(K230_ALIGN_ERROR_SEND, now_ms);
                    break;
                }

                enter_state(K230_ALIGN_WAIT_OFFSET, now_ms);
            }
            break;

        case K230_ALIGN_WAIT_OFFSET:
            have_x = axis_line_usable(g_k230_status.axis_x_dir,
                                      g_k230_status.axis_x_ms);
            have_y = axis_line_usable(g_k230_status.axis_y_dir,
                                      g_k230_status.axis_y_ms);

            px_x = have_x ? ((g_k230_status.axis_x_dir == 'w') ?
                             g_k230_status.axis_x_px :
                             (-g_k230_status.axis_x_px)) : 0;

            px_y = have_y ? ((g_k230_status.axis_y_dir == 'a') ?
                             g_k230_status.axis_y_px :
                             (-g_k230_status.axis_y_px)) : 0;

            /*
             * 偏差帧要过两道闸才消费：DETECT 下先确认目标颜色，
             * 颜色对了进来的偏差才拿去纠偏。PLACE 不报颜色，color_ready() 恒真。
             *
             * 两轴到齐就立刻合成一次斜向平移；只到一轴（另一轴偏差本来
             * 就是 0、K230 不发那一行）则等满配对窗口再按单轴走，免得
             * 每轮都空等一整个窗口。
             */
            if (color_ready() &&
                ((have_x && have_y) ||
                 ((have_x || have_y) &&
                  ((now_ms - s_state_start_ms) >= K230_ALIGN_PAIR_WINDOW_MS))))
            {
                s_grab_seen = 0U;
                s_place_seen = 0U;
                handle_offset_pair(px_x, px_y, now_ms);
                break;
            }

            /*
             * 两条“K230 说不用再调了”的路，按模式各认一条：
             *   DETECT 认 grab  —— 且必须先确认过目标颜色。否则没找到料时
             *                      K230 发的 grab 也会让流程直接进夹取，等于没识别；
             *   PLACE  认 place —— 该模式不报颜色，place 就是对位完成的信号。
             * 命中后主动发 stop，否则 K230 会继续占着 UART4 发偏差帧。
             */
            if (((s_mode == K230_ALIGN_MODE_DETECT) &&
                 (s_grab_seen != 0U) && color_ready()) ||
                ((s_mode == K230_ALIGN_MODE_PLACE) &&
                 (s_place_seen != 0U)))
            {
                s_grab_seen = 0U;
                s_place_seen = 0U;
                robot_k230_align_exit = (s_mode == K230_ALIGN_MODE_DETECT) ?
                                        K230_ALIGN_EXIT_GRAB :
                                        K230_ALIGN_EXIT_PLACE;
                (void)K230_Stop();
                enter_state(K230_ALIGN_DONE, now_ms);
                break;
            }

            /*
             * 这里刻意不做“超时失败”：K230 识别颜色、算出偏差要多久，由它自己决定，
             * 早报错会把“识别得慢”误判成“识别不到”。到点只补发一次命令催促，
             * 然后继续等——对“一条命令一帧结果”的固件，丢了首帧也能靠这次补发自愈。
             * 想结束对位只剩这几条路：偏差进容差、收到 grab/place、K230 报 error,xxx、
             * 或上层急停 / 区域任务失败。
             */
            if ((now_ms - s_state_start_ms) >= K230_ALIGN_OFFSET_TIMEOUT_MS)
            {
                if (send_align_command() != HAL_OK)
                {
                    align_fail(K230_ALIGN_ERROR_SEND, now_ms);
                    break;
                }

                /*
                 * 只把补发计时往后推，不动收集起点（epoch）：
                 * 已经收到的那一轴不该因为这次催促而作废。
                 */
                s_state_start_ms = now_ms;
            }
            break;

        case K230_ALIGN_MOVING:
            if (chassis_motion_failed(s_chassis))
            {
                align_fail(K230_ALIGN_ERROR_MOTION, now_ms);
                break;
            }

            if (chassis_motion_done(s_chassis))
            {
                /*
                 * 丢掉移动过程中的帧：只认停稳之后收到的偏差与收工信号。
                 * grab/place 也要清——微调那段时间拍到的画面已经作废，
                 * 留着会让刚停稳就在 WAIT_OFFSET 立刻收工。
                 */
                s_grab_seen = 0U;
                s_place_seen = 0U;
                enter_state(K230_ALIGN_SETTLE, now_ms);
            }
            break;

        case K230_ALIGN_SETTLE:
            if ((now_ms - s_state_start_ms) >= K230_ALIGN_SETTLE_MS)
            {
                enter_state(K230_ALIGN_WAIT_OFFSET, now_ms);
            }
            break;

        default:
            align_fail(K230_ALIGN_ERROR_PARAM, now_ms);
            break;
    }
}

void K230Align_Abort(void)
{
    /* 让 K230 停止检测并清掉事件缓冲；当前未在对位中时也无害。 */
    K230_TaskAbort();

    if (!K230Align_IsBusy())
    {
        return;
    }

    if (s_chassis != NULL)
    {
        chassis_motion_stop(s_chassis);
    }

    robot_k230_align_error = K230_ALIGN_ERROR_ABORTED;
    robot_k230_align_exit = K230_ALIGN_EXIT_ABORT;
    enter_state(K230_ALIGN_ERROR, HAL_GetTick());
}

bool K230Align_IsBusy(void)
{
    return (s_state != K230_ALIGN_IDLE) &&
           (s_state != K230_ALIGN_DONE) &&
           (s_state != K230_ALIGN_ERROR);
}

bool K230Align_IsDone(void)
{
    return s_state == K230_ALIGN_DONE;
}

bool K230Align_HasError(void)
{
    return s_state == K230_ALIGN_ERROR;
}

uint8_t K230Align_GetIteration(void)
{
    return s_iteration;
}

uint8_t K230Align_GetError(void)
{
    return robot_k230_align_error;
}
