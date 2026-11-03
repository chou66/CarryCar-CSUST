#include "chassis.h"
#include "chassis_config.h"
#include "emm_adapter.h"
#include <math.h>

#define CHASSIS_SYNC_GAP_MS                  3U
/* W1~W4+SYNC 或 STOP1~STOP4 最多允许占用状态机的时间。 */
#define CHASSIS_TX_TIMEOUT_MS               300U
#define CHASSIS_ALL_WHEELS_MASK              0x0FU

/* 位置模式：同步启动后稍等一下再读 0x3A，避免读到上一目标刚留下的状态。 */
#define CHASSIS_STATUS_QUERY_GUARD_MS         80U
/* 每次 update 最多发一个状态查询；查询间隔由该宏独立限频。 */
#define CHASSIS_STATUS_QUERY_INTERVAL_MS      10U

/* 位置模式动态超时：理论时间再留 50% + 1500 ms 固定余量。 */
#define CHASSIS_TIMEOUT_SCALE_NUM             3ULL
#define CHASSIS_TIMEOUT_SCALE_DEN             2ULL
#define CHASSIS_TIMEOUT_MARGIN_MS             1500ULL
#define CHASSIS_TIMEOUT_MIN_MS                2500ULL
#define CHASSIS_TIMEOUT_MAX_MS              120000ULL

#define CHASSIS_PI_F              3.14159265358979323846f
#define CHASSIS_DEG_TO_RAD_F      0.01745329251994329577f

static const WheelCommand *wheel_for_id(const MecanumWheelCommand *wheels, uint8_t id)
{
    switch (id) {
    case 1U: return &wheels->fl;
    case 2U: return &wheels->rl;
    case 3U: return &wheels->rr;
    default: return &wheels->fr;
    }
}

static void set_imu_motion_hint(Chassis *chassis, bool moving)
{
    if (chassis && chassis->imu_filter)
        hwt101_set_platform_moving(chassis->imu_filter, moving);
}

/*
 * 把“轮缘线速度对应的 RPM”反算成底盘角速度 wz（rad/s）。
 * 与 edge_motion / robot_app 里的换算完全一致，用于初始化漂移航向环的
 * 最大/最小输出。
 */
static float chassis_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
{
    float k;
    float factor;

    if (!chassis ||
        chassis->config.wheel_radius_m <= 0.0f ||
        chassis->config.gear_ratio <= 0.0f)
        return 0.0f;

    k = chassis->config.half_length_m + chassis->config.half_width_m;
    if (k <= 0.0f)
        return 0.0f;

    factor = 60.0f * chassis->config.gear_ratio /
             (2.0f * CHASSIS_PI_F * chassis->config.wheel_radius_m);
    if (factor <= 0.0f)
        return 0.0f;

    return (float)rpm / (k * factor);
}

static HAL_StatusTypeDef send_motion_to_wheel(const Chassis *chassis, uint8_t id)
{
    const WheelCommand *wheel = wheel_for_id(&chassis->active_wheels, id);

    if (chassis->active_mode == CHASSIS_MODE_POSITION) {
        /* 位置目标为 0 的轮子不参与本次位置动作。 */
        if (wheel->pulses == 0U) return HAL_OK;

        /*
         * raF 保持 0U：沿用已验证的相对位置模式；
         * snF=true：四轮先缓存，最后由 0xFF 同步启动。
         */
        return emm_pos(id, wheel->dir, wheel->rpm, chassis->acceleration,
                       wheel->pulses, 0U, true);
    }

    /*
     * 剖面模式的速度曲线已经由软件正弦加减速给出，EMM 自身必须 acc=0，
     * 否则驱动器斜坡会和软件曲线叠加，距离积分不再成立。
     * 直接速度模式（航向 PD 等短时命令）仍使用调用方设置的 acceleration。
     */
    {
        uint8_t acc = (chassis->active_mode == CHASSIS_MODE_PROFILE) ?
                      0U : chassis->acceleration;
        return emm_vel(id, wheel->dir, wheel->rpm, acc, true);
    }
}

static void advance_wheel_state(Chassis *chassis, uint32_t now_ms)
{
    chassis->tx_state = (ChassisTxState)((uint32_t)chassis->tx_state + 1U);
    chassis->next_tx_ms = now_ms + CHASSIS_SYNC_GAP_MS;
}

/*
 * EMM 固件 acc：
 *   acc=0 -> 直接按设定速度启动；
 *   acc!=0 -> 每 (256-acc)*50us 增/减 1 RPM。
 *
 * 恒速理论时间 + 加速时间 + 减速时间，再乘安全系数并加固定余量。
 * 动态时间只负责“最多允许多久”，不拿它判断“已经到位”。
 */
static uint32_t estimate_position_timeout_ms(const Chassis *chassis)
{
    uint64_t longest_ms = 0ULL;
    uint8_t id;

    if (chassis->config.motor_pulses_per_rev == 0U) {
        return (uint32_t)CHASSIS_TIMEOUT_MAX_MS;
    }

    for (id = 1U; id <= 4U; ++id) {
        const WheelCommand *wheel = wheel_for_id(&chassis->active_wheels, id);
        uint64_t travel_ms;
        uint64_t ramp_ms = 0ULL;
        uint64_t total_ms;
        uint64_t denominator;

        if (wheel->pulses == 0U) continue;
        if (wheel->rpm == 0U) return (uint32_t)CHASSIS_TIMEOUT_MAX_MS;

        denominator = (uint64_t)chassis->config.motor_pulses_per_rev * (uint64_t)wheel->rpm;
        travel_ms = ((uint64_t)wheel->pulses * 60000ULL + denominator - 1ULL) / denominator;

        if (chassis->acceleration != 0U) {
            /* 单边 ramp_ms = rpm * (256-acc) * 0.05 ms，乘 2 计入加速+减速。 */
            uint64_t ramp_ticks_50us =
                (uint64_t)wheel->rpm * (uint64_t)(256U - chassis->acceleration) * 2ULL;
            ramp_ms = (ramp_ticks_50us + 19ULL) / 20ULL;
        }

        total_ms = travel_ms + ramp_ms;
        if (total_ms > longest_ms) longest_ms = total_ms;
    }

    longest_ms = (longest_ms * CHASSIS_TIMEOUT_SCALE_NUM +
                  CHASSIS_TIMEOUT_SCALE_DEN - 1ULL) / CHASSIS_TIMEOUT_SCALE_DEN;
    longest_ms += CHASSIS_TIMEOUT_MARGIN_MS;

    if (longest_ms < CHASSIS_TIMEOUT_MIN_MS) longest_ms = CHASSIS_TIMEOUT_MIN_MS;
    if (longest_ms > CHASSIS_TIMEOUT_MAX_MS) longest_ms = CHASSIS_TIMEOUT_MAX_MS;
    return (uint32_t)longest_ms;
}

static uint8_t next_active_motor_id(uint8_t motor_mask, uint8_t from_id)
{
    uint8_t step;
    uint8_t id = from_id;

    if (id < 1U || id > 4U) id = 1U;

    for (step = 0U; step < 4U; ++step) {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        if ((motor_mask & bit) != 0U) return id;
        id = (id >= 4U) ? 1U : (uint8_t)(id + 1U);
    }
    return 0U;
}

static void begin_motion(Chassis *chassis, uint32_t now_ms)
{
    uint8_t id;

    chassis->active_mode = chassis->mode;

    if (chassis->active_mode == CHASSIS_MODE_POSITION) {
        mecanum_inverse_pos(&chassis->config, chassis->target_x, chassis->target_y,
                            chassis->target_z, chassis->speed_limit,
                            &chassis->active_wheels);

        chassis->motor_mask = 0U;
        for (id = 1U; id <= 4U; ++id) {
            if (wheel_for_id(&chassis->active_wheels, id)->pulses != 0U) {
                chassis->motor_mask |= (uint8_t)(1U << (id - 1U));
                emm_prepare_position_feedback(id);
            }
        }

        chassis->position_done = false;
        chassis->position_failed = false;

        if (chassis->motor_mask == 0U) {
            chassis->position_done = true;
            chassis->tx_state = CHASSIS_TX_IDLE;
            chassis->dirty = false;
            set_imu_motion_hint(chassis, false);
            return;
        }

        chassis->active_position_timeout_ms =
            (chassis->position_timeout_ms != 0U) ?
            chassis->position_timeout_ms : estimate_position_timeout_ms(chassis);
    } else {
        mecanum_inverse(&chassis->config, chassis->target_x, chassis->target_y,
                        chassis->target_z, &chassis->active_wheels);
        chassis->motor_mask = CHASSIS_ALL_WHEELS_MASK;
    }

    chassis->dirty = false;
    chassis->tx_state = CHASSIS_TX_WHEEL_1;
    chassis->next_tx_ms = now_ms;
    chassis->tx_deadline_ms = now_ms + CHASSIS_TX_TIMEOUT_MS;
}

static void begin_stop(Chassis *chassis, uint32_t now_ms)
{
    chassis->tx_state = CHASSIS_TX_STOP_1;
    chassis->next_tx_ms = now_ms;
    chassis->tx_deadline_ms = now_ms + CHASSIS_TX_TIMEOUT_MS;
}

static void fail_position_motion(Chassis *chassis, uint32_t now_ms)
{
    chassis->position_failed = true;
    chassis->position_done = false;
    chassis->emergency_stop = true;
    set_imu_motion_hint(chassis, false);
    begin_stop(chassis, now_ms);
}

static bool position_command_pending_or_active(const Chassis *chassis)
{
    if (chassis->dirty && chassis->mode == CHASSIS_MODE_POSITION) return true;

    if (chassis->active_mode != CHASSIS_MODE_POSITION) return false;

    return ((chassis->tx_state >= CHASSIS_TX_WHEEL_1 &&
             chassis->tx_state <= CHASSIS_TX_SYNC) ||
            chassis->tx_state == CHASSIS_TX_WAIT_POSITION);
}

static void fail_motion_dispatch(Chassis *chassis, uint32_t now_ms)
{
    if (chassis->mode == CHASSIS_MODE_PROFILE) {
        chassis->position_failed = true;
        chassis->position_done = false;
        chassis->profile_running = 0U;
        chassis->profile_finishing = 0U;
    } else if (chassis->active_mode == CHASSIS_MODE_POSITION) {
        chassis->position_failed = true;
        chassis->position_done = false;
    }

    chassis->tx_failed = true;
    chassis->dirty = false;
    chassis->velocity_watchdog_armed = false;
    chassis->emergency_stop = true;
    set_imu_motion_hint(chassis, false);
    begin_stop(chassis, now_ms);
}

static bool velocity_input_valid(float vx, float vy, float wz)
{
    return isfinite(vx) && isfinite(vy) && isfinite(wz);
}

static bool position_input_valid(float dx, float dy, float dtheta, float speed)
{
    return isfinite(dx) && isfinite(dy) && isfinite(dtheta) &&
           isfinite(speed) && speed > 0.0f;
}

/*
 * 只更新速度目标，不改变模式。
 * 剖面刷新和公开的速度命令共用这一段，避免剖面每次刷新都把模式改回
 * CHASSIS_MODE_VELOCITY。
 */
static void apply_velocity_targets(Chassis *chassis, float vx, float vy,
                                   float wz, uint32_t now_ms)
{
    chassis->target_x = vx;
    chassis->target_y = vy;
    chassis->target_z = wz;
    chassis->last_command_ms = now_ms;
    chassis->emergency_stop = false;
    chassis->velocity_watchdog_armed = true;
    chassis->dirty = true;
    set_imu_motion_hint(chassis, vx != 0.0f || vy != 0.0f || wz != 0.0f);
}

/*
 * 正弦加减速形状：
 *   0 -> ramp：0.5 - 0.5*cos(pi*t/ramp)
 *   ramp..total-ramp：1
 *   total-ramp -> total：镜像下降
 * 曲线在起点/终点速度为 0，对时间积分正好是“峰值 * (ramp + cruise)”。
 */
static float profile_speed_factor(const Chassis *chassis, uint32_t elapsed_ms)
{
    uint32_t ramp = chassis->profile_ramp_ms;
    uint32_t total = chassis->profile_total_ms;

    if (total == 0U || elapsed_ms >= total)
        return 0.0f;

    if (ramp == 0U)
        return 1.0f;

    if (chassis->profile_ramp_in && elapsed_ms < ramp)
        return 0.5f - 0.5f * cosf(CHASSIS_PI_F *
                                  (float)elapsed_ms / (float)ramp);

    if (chassis->profile_ramp_out && elapsed_ms + ramp > total) {
        float remain_ms = (float)(total - elapsed_ms);
        return 0.5f - 0.5f * cosf(CHASSIS_PI_F *
                                  remain_ms / (float)ramp);
    }

    return 1.0f;
}

/*
 * 剖面时间到。
 *
 * ramp_out=1（普通段）：先让速度曲线自然归零，再走统一的四轮 STOP 序列。
 * ramp_out=0（拐角衔接段）：不再停车，保持当前满速命令，由下一段剖面
 *   立即接管。速度看门狗仍然 armed，若下一段没有及时启动，150 ms 后
 *   底盘会自行 STOP，不会失控。
 */
static void finish_profile(Chassis *chassis)
{
    chassis->profile_running = 0U;

    if (!chassis->profile_ramp_out) {
        chassis->profile_finishing = 0U;
        chassis->dirty = false;
        chassis->position_done = true;
        chassis->position_failed = false;
        return;
    }

    chassis->profile_finishing = 1U;
    chassis->dirty = false;
    chassis->target_x = 0.0f;
    chassis->target_y = 0.0f;
    chassis->target_z = 0.0f;
    chassis->velocity_watchdog_armed = false;
    chassis->emergency_stop = true;
}

/*
 * 每个 CHASSIS_PROFILE_UPDATE_MS 刷新一次速度。
 * 漂移模式：用当前世界航向把世界速度旋到车体系，同时航向 PD 输出 wz；
 * 普通模式：车体系速度矢量整体乘正弦系数。
 */
static void profile_tick(Chassis *chassis, uint32_t now_ms)
{
    uint32_t elapsed;
    float factor;

    if (!chassis->profile_running)
        return;

    elapsed = (uint32_t)(now_ms - chassis->profile_start_ms);

    if (elapsed >= chassis->profile_total_ms) {
        finish_profile(chassis);
        return;
    }

    if ((int32_t)(now_ms - chassis->profile_next_update_ms) < 0)
        return;

    factor = profile_speed_factor(chassis, elapsed);

    /* 起步瞬间 factor=0，等下一拍再发第一帧非零速度，避免把 0 RPM 当命令刷出去。 */
    if (factor > 0.0001f) {
        if (chassis->profile_heading_enabled) {
            Hwt101 *imu = chassis->imu_filter;
            float yaw = chassis->profile_last_yaw_deg;
            float wz = 0.0f;
            float wx;
            float wy;
            float th;
            float cth;
            float sth;
            float path_yaw_deg = chassis->profile_target_yaw_deg;

            /*
             * 圆弧剖面：切线方向（=世界速度方向）与航向目标按固定角速度
             * 同步推进，车体中心走出半径 R = v/ω 的圆弧；航向 PD 只修
             * 实际航向与切线的残差，再叠加 ω 前馈。
             */
            if (chassis->profile_arc_enabled) {
                path_yaw_deg = chassis->profile_arc_start_yaw_deg +
                    chassis->profile_arc_yaw_rate_dps *
                    ((float)elapsed * 0.001f);
                /*
                 * path_yaw_deg 是车体中心的切线（行进方向）；倒车走弧时
                 * 车头锁在切线+180°，与倒车直线段的车头方向一致。
                 */
                heading_set_target(&chassis->profile_heading,
                                   chassis->profile_arc_reverse ?
                                   (path_yaw_deg + 180.0f) : path_yaw_deg);
            }

            if (imu && hwt101_world_zero_ready(imu) &&
                hwt101_is_fresh(imu, now_ms, CHASSIS_PROFILE_IMU_TIMEOUT_MS)) {
                yaw = hwt101_world_yaw_deg(imu);
                chassis->profile_last_yaw_deg = yaw;
                wz = heading_update_world_cw(
                    &chassis->profile_heading,
                    yaw,
                    HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps,
                    (float)CHASSIS_PROFILE_UPDATE_MS / 1000.0f);
            }

            if (chassis->profile_arc_enabled) {
                /* 世界航向 0=+Y、顺时针为正 => 速度矢量 = v*(sin, cos)。 */
                th = path_yaw_deg * CHASSIS_DEG_TO_RAD_F;
                wx = chassis->profile_arc_speed_mps * sinf(th);
                wy = chassis->profile_arc_speed_mps * cosf(th);
                /* 世界顺时针角速度为正，底盘 +w 逆时针为正。 */
                wz += -chassis->profile_arc_yaw_rate_dps *
                      CHASSIS_DEG_TO_RAD_F;
                cth = cosf(yaw * CHASSIS_DEG_TO_RAD_F);
                sth = sinf(yaw * CHASSIS_DEG_TO_RAD_F);
            } else {
                th = yaw * CHASSIS_DEG_TO_RAD_F;
                cth = cosf(th);
                sth = sinf(th);
                wx = chassis->profile_world_vx_mps;
                wy = chassis->profile_world_vy_mps;
            }

            wx *= factor;
            wy *= factor;
            /*
             * 航向修正也乘同一个正弦系数：EMM 在剖面模式下 acc=0，
             * 如果 wz 不渐变，起步第一帧就会从 0 跳到最大修正转速。
             * 两者同比例缩放，各轮转速始终是一条平滑的正弦曲线。
             */
            wz *= factor;

            /* 世界系 -> 车体系：车体 +Y 在世界系为 (sin,cos)，+X 为 (cos,-sin)。 */
            apply_velocity_targets(chassis,
                                   wx * cth - wy * sth,
                                   wx * sth + wy * cth,
                                   wz,
                                   now_ms);
        } else {
            apply_velocity_targets(chassis,
                                   chassis->profile_vx_mps * factor,
                                   chassis->profile_vy_mps * factor,
                                   chassis->profile_wz_radps * factor,
                                   now_ms);
        }
    }

    chassis->profile_next_update_ms = now_ms + CHASSIS_PROFILE_UPDATE_MS;
}

static void start_profile_ex(Chassis *chassis, uint32_t ramp_ms,
                             uint32_t total_ms, uint8_t ramp_in,
                             uint8_t ramp_out, uint32_t now_ms)
{
    chassis->mode = CHASSIS_MODE_PROFILE;
    /* 派发时按这个模式决定 EMM 的 acc：剖面必须 acc=0，避免斜坡叠加。 */
    chassis->active_mode = CHASSIS_MODE_PROFILE;
    chassis->profile_ramp_ms = ramp_ms;
    chassis->profile_total_ms = total_ms;
    chassis->profile_ramp_in = ramp_in ? 1U : 0U;
    chassis->profile_ramp_out = ramp_out ? 1U : 0U;

    chassis->profile_start_ms = now_ms;
    chassis->profile_next_update_ms = now_ms;
    chassis->profile_running = (chassis->profile_total_ms == 0U) ? 0U : 1U;
    chassis->profile_finishing = 0U;

    chassis->position_done = false;
    chassis->position_failed = false;
    chassis->tx_failed = false;
    chassis->velocity_watchdog_armed = false;
    chassis->emergency_stop = false;
    chassis->dirty = false;

    if (chassis->profile_total_ms == 0U) {
        /* 零位移：直接把完成标志给上层，不产生任何轮速命令。 */
        chassis->position_done = true;
        set_imu_motion_hint(chassis, false);
        return;
    }

    set_imu_motion_hint(chassis, true);
}

/* 普通剖面：两端都做正弦加减速，总时长 = ramp + cruise + ramp。 */
static void start_profile(Chassis *chassis, uint32_t ramp_ms, uint32_t cruise_ms,
                          uint32_t now_ms)
{
    chassis->profile_cruise_ms = cruise_ms;
    start_profile_ex(chassis, ramp_ms,
                     (uint32_t)(ramp_ms * 2U + cruise_ms),
                     1U, 1U, now_ms);
}

void chassis_init(Chassis *chassis, const MecanumConfig *config)
{
    *chassis = (Chassis){
        .config = *config,
        .acceleration = 100U,
        .watchdog_ms = 150U,
        .position_timeout_ms = 0U, /* 0 = 按行程动态估算 */
        .mode = CHASSIS_MODE_VELOCITY,
        .active_mode = CHASSIS_MODE_VELOCITY,
        .profile_ramp_in = 1U,
        .profile_ramp_out = 1U,
        .status_query_id = 1U,
        .tx_state = CHASSIS_TX_IDLE
    };

    heading_init(&chassis->profile_heading,
                 CHASSIS_DRIFT_YAW_KP,
                 CHASSIS_DRIFT_YAW_KD,
                 chassis_wz_for_rpm(chassis, CHASSIS_DRIFT_YAW_MAX_RPM),
                 CHASSIS_DRIFT_YAW_DEADBAND_DEG);
    heading_set_min_output(&chassis->profile_heading,
                           chassis_wz_for_rpm(chassis, CHASSIS_DRIFT_YAW_MIN_RPM));
}

void chassis_bind_imu_filter(Chassis *chassis, Hwt101 *imu)
{
    if (!chassis) return;
    chassis->imu_filter = imu;
    hwt101_set_platform_moving(imu, false);
}

void chassis_set_velocity(Chassis *chassis, float vx, float vy, float wz, uint32_t now_ms)
{
    /* NaN/Inf must never reach the kinematics or float-to-integer conversions. */
    if (!velocity_input_valid(vx, vy, wz)) {
        chassis_stop(chassis);
        return;
    }

    /*
     * 直接速度命令优先于剖面：收到后立即取消剖面状态，
     * 否则下一次 profile_tick 会把速度目标改回去。
     *
     * 被抢占的剖面必须报 failed：剩余距离已经无法还原，若只清 done
     * 不置 failed，等待“done 或 failed”的 EdgeMotion 会永远停在
     * TRANSLATING 里（车不动、也不报错）。
     */
    if (chassis->profile_running || chassis->profile_finishing) {
        chassis->profile_running = 0U;
        chassis->profile_finishing = 0U;
        chassis->position_failed = true;
        chassis->position_done = false;
    }

    /* 位置模式同理：剩余脉冲无法还原，必须让上层看到 failed。 */
    if (position_command_pending_or_active(chassis)) {
        chassis->position_failed = true;
        chassis->position_done = false;
        chassis->tx_state = CHASSIS_TX_IDLE;
    }

    chassis->mode = CHASSIS_MODE_VELOCITY;
    chassis->active_mode = CHASSIS_MODE_VELOCITY;
    chassis->tx_failed = false;
    apply_velocity_targets(chassis, vx, vy, wz, now_ms);
}

void chassis_set_position(Chassis *chassis, float dx, float dy, float dtheta,
                          float speed, uint32_t now_ms)
{
    /* speed 是正的幅值；NaN/Inf/0/负值都按失败处理。 */
    if (!position_input_valid(dx, dy, dtheta, speed)) {
        chassis_stop(chassis);
        chassis->position_done = false;
        chassis->position_failed = true;
        return;
    }

    /* 位置命令抢占正在执行的剖面/速度命令。 */
    if (chassis->profile_running || chassis->profile_finishing) {
        chassis->profile_running = 0U;
        chassis->profile_finishing = 0U;
    }
    /* 上一段 W1~SYNC 或位置等待可以直接被新目标覆盖；STOP 序列不打断。 */
    if (chassis->tx_state >= CHASSIS_TX_WHEEL_1 &&
        chassis->tx_state <= CHASSIS_TX_WAIT_POSITION) {
        chassis->tx_state = CHASSIS_TX_IDLE;
    }

    chassis->mode = CHASSIS_MODE_POSITION;
    chassis->active_mode = CHASSIS_MODE_POSITION;
    chassis->target_x = dx;
    chassis->target_y = dy;
    chassis->target_z = dtheta;
    chassis->speed_limit = speed;
    chassis->last_command_ms = now_ms;
    chassis->emergency_stop = false;
    chassis->velocity_watchdog_armed = false;
    chassis->tx_failed = false;
    chassis->position_done = false;
    chassis->position_failed = false;
    chassis->dirty = true;
    set_imu_motion_hint(chassis,
                        dx != 0.0f || dy != 0.0f || dtheta != 0.0f);
}

void chassis_set_profile(Chassis *chassis, float vx, float vy, float wz,
                         uint32_t ramp_ms, uint32_t cruise_ms, uint32_t now_ms)
{
    if (!velocity_input_valid(vx, vy, wz)) {
        chassis_stop(chassis);
        return;
    }

    /* 剖面抢占位置模式：剩余脉冲无法还原，上层必须看到 failed。 */
    if (position_command_pending_or_active(chassis)) {
        chassis->position_failed = true;
        chassis->position_done = false;
        chassis->tx_state = CHASSIS_TX_IDLE;
    }

    chassis->profile_vx_mps = vx;
    chassis->profile_vy_mps = vy;
    chassis->profile_wz_radps = wz;
    chassis->profile_world_vx_mps = 0.0f;
    chassis->profile_world_vy_mps = 0.0f;
    chassis->profile_heading_enabled = 0U;
    chassis->profile_arc_enabled = 0U;
    chassis->profile_arc_reverse = 0U;
    start_profile(chassis, ramp_ms, cruise_ms, now_ms);
}

void chassis_set_profile_world(Chassis *chassis,
                               float vx_world, float vy_world,
                               float target_world_yaw_deg,
                               uint32_t ramp_ms, uint32_t cruise_ms,
                               uint32_t now_ms)
{
    if (!velocity_input_valid(vx_world, vy_world, target_world_yaw_deg)) {
        chassis_stop(chassis);
        return;
    }

    chassis->profile_world_vx_mps = vx_world;
    chassis->profile_world_vy_mps = vy_world;
    chassis->profile_target_yaw_deg = target_world_yaw_deg;
    chassis->profile_last_yaw_deg = target_world_yaw_deg;

    if (chassis->imu_filter) {
        Hwt101 *imu = chassis->imu_filter;

        if (hwt101_world_zero_ready(imu)) {
            chassis->profile_last_yaw_deg = hwt101_world_yaw_deg(imu);
        }

        heading_set_target(&chassis->profile_heading, target_world_yaw_deg);
        chassis->profile_heading_enabled = 1U;
        /* 纯漂移路径没有独立的自转角速度，wz 全部由航向环给出。 */
        chassis->profile_vx_mps = 0.0f;
        chassis->profile_vy_mps = 0.0f;
        chassis->profile_wz_radps = 0.0f;
    } else {
        /*
         * 没有 IMU 时退化为车体系速度：调用方此时本来就把世界方向
         * 近似成车体方向（起步前已完成转向）。
         */
        chassis->profile_vx_mps = vx_world;
        chassis->profile_vy_mps = vy_world;
        chassis->profile_wz_radps = 0.0f;
        chassis->profile_heading_enabled = 0U;
    }

    chassis->profile_arc_enabled = 0U;
    chassis->profile_arc_reverse = 0U;
    start_profile(chassis, ramp_ms, cruise_ms, now_ms);
}

void chassis_set_profile_world_ex(Chassis *chassis,
                                  float vx_world, float vy_world,
                                  float target_world_yaw_deg,
                                  uint8_t ramp_in, uint8_t ramp_out,
                                  uint32_t ramp_ms, uint32_t cruise_ms,
                                  uint32_t now_ms)
{
    uint32_t total_ms;

    if (!velocity_input_valid(vx_world, vy_world, target_world_yaw_deg)) {
        chassis_stop(chassis);
        return;
    }

    chassis->profile_world_vx_mps = vx_world;
    chassis->profile_world_vy_mps = vy_world;
    chassis->profile_target_yaw_deg = target_world_yaw_deg;
    chassis->profile_last_yaw_deg = target_world_yaw_deg;
    chassis->profile_arc_enabled = 0U;
    chassis->profile_arc_reverse = 0U;

    if (chassis->imu_filter) {
        Hwt101 *imu = chassis->imu_filter;

        if (hwt101_world_zero_ready(imu)) {
            chassis->profile_last_yaw_deg = hwt101_world_yaw_deg(imu);
        }

        heading_set_target(&chassis->profile_heading, target_world_yaw_deg);
        chassis->profile_heading_enabled = 1U;
        chassis->profile_vx_mps = 0.0f;
        chassis->profile_vy_mps = 0.0f;
        chassis->profile_wz_radps = 0.0f;
    } else {
        chassis->profile_vx_mps = vx_world;
        chassis->profile_vy_mps = vy_world;
        chassis->profile_wz_radps = 0.0f;
        chassis->profile_heading_enabled = 0U;
    }

    total_ms = (uint32_t)((ramp_in ? ramp_ms : 0U) + cruise_ms +
                          (ramp_out ? ramp_ms : 0U));
    chassis->profile_cruise_ms = cruise_ms;
    start_profile_ex(chassis, ramp_ms, total_ms, ramp_in, ramp_out, now_ms);
}

void chassis_set_profile_world_arc_ex(Chassis *chassis,
                                      float start_world_yaw_deg,
                                      float yaw_rate_dps,
                                      float speed_mps,
                                      uint8_t reverse,
                                      uint32_t total_ms,
                                      uint32_t now_ms)
{
    float start_heading_deg;

    if (!velocity_input_valid(start_world_yaw_deg, yaw_rate_dps, speed_mps)) {
        chassis_stop(chassis);
        return;
    }

    reverse = reverse ? 1U : 0U;
    start_heading_deg = reverse ? (start_world_yaw_deg + 180.0f)
                                : start_world_yaw_deg;

    chassis->profile_arc_speed_mps = speed_mps;
    chassis->profile_arc_start_yaw_deg = start_world_yaw_deg;
    chassis->profile_arc_yaw_rate_dps = yaw_rate_dps;
    chassis->profile_arc_reverse = reverse;
    chassis->profile_target_yaw_deg = start_heading_deg;
    chassis->profile_last_yaw_deg = start_heading_deg;
    chassis->profile_world_vx_mps = 0.0f;
    chassis->profile_world_vy_mps = 0.0f;
    chassis->profile_vx_mps = 0.0f;
    chassis->profile_vy_mps = 0.0f;
    chassis->profile_wz_radps = 0.0f;

    if (chassis->imu_filter) {
        Hwt101 *imu = chassis->imu_filter;

        if (hwt101_world_zero_ready(imu)) {
            chassis->profile_last_yaw_deg = hwt101_world_yaw_deg(imu);
        }

        heading_set_target(&chassis->profile_heading, start_heading_deg);
        chassis->profile_heading_enabled = 1U;
    } else {
        /* 没有 IMU 无法把世界速度旋到车体系，圆弧退化为不动作，由上层报失败。 */
        chassis->profile_heading_enabled = 0U;
        chassis_stop(chassis);
        return;
    }

    chassis->profile_arc_enabled = 1U;
    chassis->profile_cruise_ms = total_ms;
    start_profile_ex(chassis, 0U, total_ms, 0U, 0U, now_ms);
}

void chassis_set_profile_world_arc(Chassis *chassis,
                                   float start_world_yaw_deg,
                                   float yaw_rate_dps,
                                   float speed_mps,
                                   uint32_t total_ms,
                                   uint32_t now_ms)
{
    chassis_set_profile_world_arc_ex(chassis, start_world_yaw_deg,
                                     yaw_rate_dps, speed_mps, 0U,
                                     total_ms, now_ms);
}

void chassis_stop(Chassis *chassis)
{
    /* 主动停车或速度超时后解除速度看门狗，避免重复触发 STOP。 */
    chassis->velocity_watchdog_armed = false;
    /* 显式 stop 允许从此前 TX fault 中重新尝试停车。 */
    chassis->tx_failed = false;

    if (chassis->profile_running || chassis->profile_finishing) {
        chassis->profile_running = 0U;
        chassis->profile_finishing = 0U;
        chassis->position_failed = true;
        chassis->position_done = false;
    }

    /*
     * 只中止“正在执行”的位置命令；已经到位的位置命令必须保留
     * position_done=true，否则上层会把成功当成失败。
     */
    if (position_command_pending_or_active(chassis)) {
        chassis->position_failed = true;
        chassis->position_done = false;
    }

    chassis->target_x = 0.0f;
    chassis->target_y = 0.0f;
    chassis->target_z = 0.0f;
    chassis->dirty = false;
    chassis->emergency_stop = true;
    set_imu_motion_hint(chassis, false);
}

bool chassis_position_done(const Chassis *chassis)
{
    return chassis->position_done;
}

bool chassis_position_failed(const Chassis *chassis)
{
    return chassis->position_failed;
}

bool chassis_busy(const Chassis *chassis)
{
    /* A requested STOP is busy immediately, even before the next update starts STOP_1.
     * If the STOP sequence itself times out, tx_failed releases busy so upper layers do not hang.
     */
    return chassis->dirty ||
           chassis->tx_state != CHASSIS_TX_IDLE ||
           (chassis->emergency_stop && !chassis->tx_failed) ||
           chassis->profile_running ||
           chassis->profile_finishing;
}

void chassis_update(Chassis *chassis, uint32_t now_ms)
{
    HAL_StatusTypeDef status;

    /* 软件速度看门狗：只有收到过速度命令后才 armed；超时只触发一次 STOP。 */
    if (chassis->velocity_watchdog_armed &&
        !chassis->emergency_stop && chassis->mode == CHASSIS_MODE_VELOCITY &&
        (uint32_t)(now_ms - chassis->last_command_ms) > chassis->watchdog_ms) {
        chassis_stop(chassis);
    }

    /*
     * 剖面刷新放在 STOP/派发检查之前：
     * 本拍若刚好到总时长，finish_profile() 会把 emergency_stop 置位，
     * 下面同一拍就能开始 STOP 序列。
     */
    if (chassis->mode == CHASSIS_MODE_PROFILE)
        profile_tick(chassis, now_ms);

    if (chassis->emergency_stop && !chassis->tx_failed &&
        (chassis->tx_state < CHASSIS_TX_STOP_1 || chassis->tx_state > CHASSIS_TX_STOP_4)) {
        begin_stop(chassis, now_ms);
    }

    if (chassis->tx_state == CHASSIS_TX_IDLE) {
        if (chassis->dirty && !chassis->emergency_stop) begin_motion(chassis, now_ms);
        return;
    }

    /* W1~W4/SYNC 连续发送失败时，不允许永久卡在 busy。 */
    if (chassis->tx_state >= CHASSIS_TX_WHEEL_1 &&
        chassis->tx_state <= CHASSIS_TX_SYNC &&
        (int32_t)(now_ms - chassis->tx_deadline_ms) >= 0) {
        fail_motion_dispatch(chassis, now_ms);
        return;
    }

    /* STOP 自身也设置超时；若 CAN 完全不可用，则退出发送状态机并锁存 tx_failed。 */
    if (chassis->tx_state >= CHASSIS_TX_STOP_1 &&
        chassis->tx_state <= CHASSIS_TX_STOP_4 &&
        (int32_t)(now_ms - chassis->tx_deadline_ms) >= 0) {
        chassis->tx_failed = true;
        chassis->dirty = false;
        chassis->tx_state = CHASSIS_TX_IDLE;
        chassis->profile_finishing = 0U;
        /*
         * STOP 都没发出去说明轮子可能仍在按最后一帧速度转，
         * 必须让上层看到失败，否则 EdgeMotion 会在 TRANSLATING 里等 done 永远等不到。
         */
        if (chassis->mode == CHASSIS_MODE_PROFILE ||
            chassis->active_mode == CHASSIS_MODE_POSITION) {
            chassis->position_failed = true;
            chassis->position_done = false;
        }
        return;
    }

    if ((int32_t)(now_ms - chassis->next_tx_ms) < 0) return;

    switch (chassis->tx_state) {
    case CHASSIS_TX_WHEEL_1:
    case CHASSIS_TX_WHEEL_2:
    case CHASSIS_TX_WHEEL_3:
    case CHASSIS_TX_WHEEL_4: {
        uint8_t id = (uint8_t)((uint32_t)chassis->tx_state -
                               (uint32_t)CHASSIS_TX_WHEEL_1 + 1U);
        status = send_motion_to_wheel(chassis, id);
        if (status == HAL_OK) advance_wheel_state(chassis, now_ms);
        return;
    }

    case CHASSIS_TX_SYNC:
        status = emm_sync_start();
        if (status != HAL_OK) return;

        if (chassis->active_mode == CHASSIS_MODE_POSITION) {
            chassis->position_start_ms = now_ms;
            chassis->position_feedback_not_before_ms =
                now_ms + CHASSIS_STATUS_QUERY_GUARD_MS;
            chassis->position_deadline_ms =
                now_ms + chassis->active_position_timeout_ms;
            chassis->next_status_query_ms = chassis->position_feedback_not_before_ms;
            chassis->status_query_id = 1U;
            chassis->tx_state = CHASSIS_TX_WAIT_POSITION;
            chassis->next_tx_ms = now_ms; /* WAIT_POSITION 自己管理查询节拍 */
        } else {
            chassis->tx_state = CHASSIS_TX_IDLE;
        }
        return;

    case CHASSIS_TX_WAIT_POSITION: {
        uint8_t query_id;

        /* E2/EE 或 0x3A 堵转保护：立即失败，不等到 timeout。 */
        if (emm_any_position_fault_after(chassis->motor_mask,
                                         chassis->position_start_ms)) {
            fail_position_motion(chassis, now_ms);
            return;
        }

        /*
         * 正常完成：所有参与运动的电机都在本次启动后主动查询到 0x3A，
         * 且 Prf_TF(bit1)=1。
         */
        if (emm_all_position_reached_after(chassis->motor_mask,
                                           chassis->position_feedback_not_before_ms)) {
            chassis->position_done = true;
            chassis->position_failed = false;
            chassis->tx_state = CHASSIS_TX_IDLE;
            set_imu_motion_hint(chassis, false);
            return;
        }

        /* 动态时间只做故障兜底，不把“时间到了”当成“已经到位”。 */
        if ((int32_t)(now_ms - chassis->position_deadline_ms) >= 0) {
            fail_position_motion(chassis, now_ms);
            return;
        }

        if ((int32_t)(now_ms - chassis->next_status_query_ms) < 0) return;

        query_id = next_active_motor_id(chassis->motor_mask,
                                        chassis->status_query_id);
        if (query_id == 0U) return;

        status = emm_read_status(query_id);
        if (status == HAL_OK) {
            chassis->status_query_id =
                (query_id >= 4U) ? 1U : (uint8_t)(query_id + 1U);
            chassis->next_status_query_ms =
                now_ms + CHASSIS_STATUS_QUERY_INTERVAL_MS;
        }
        /* HAL_BUSY 时不推进 ID，下一个 chassis_update() 周期重试当前电机。 */
        return;
    }

    case CHASSIS_TX_STOP_1:
    case CHASSIS_TX_STOP_2:
    case CHASSIS_TX_STOP_3:
    case CHASSIS_TX_STOP_4: {
        uint8_t id = (uint8_t)((uint32_t)chassis->tx_state -
                               (uint32_t)CHASSIS_TX_STOP_1 + 1U);
        status = emm_stop(id, false);
        if (status != HAL_OK) return;

        if (chassis->tx_state == CHASSIS_TX_STOP_4) {
            chassis->tx_state = CHASSIS_TX_IDLE;
            chassis->emergency_stop = false;
            if (chassis->profile_finishing) {
                chassis->profile_finishing = 0U;
                chassis->position_done = true;
                chassis->position_failed = false;
            }
            set_imu_motion_hint(chassis, false);
        } else {
            chassis->tx_state = (ChassisTxState)((uint32_t)chassis->tx_state + 1U);
            chassis->next_tx_ms = now_ms;
        }
        return;
    }

    default:
        fail_motion_dispatch(chassis, now_ms);
        return;
    }
}
