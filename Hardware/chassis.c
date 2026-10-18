#include "chassis.h"
#include "emm_v5.h"
#include <math.h>
#include "emm_adapter.h"

#define CHASSIS_SYNC_GAP_MS                  3U
/* W1~W4+SYNC 或 STOP1~STOP4 最多允许占用状态机的时间。 */
#define CHASSIS_TX_TIMEOUT_MS               300U
#define CHASSIS_ALL_WHEELS_MASK              0x0FU

/* 同步启动后稍等一下再读 0x3A，避免读到上一目标刚留下的状态。 */
#define CHASSIS_STATUS_QUERY_GUARD_MS         80U
/* 每次 update 最多发一个状态查询；查询间隔由该宏独立限频。 */
#define CHASSIS_STATUS_QUERY_INTERVAL_MS      10U

/* 动态超时：理论时间再留 50% + 1500 ms 固定余量。 */
#define CHASSIS_TIMEOUT_SCALE_NUM             3ULL
#define CHASSIS_TIMEOUT_SCALE_DEN             2ULL
#define CHASSIS_TIMEOUT_MARGIN_MS             1500ULL
#define CHASSIS_TIMEOUT_MIN_MS                2500ULL
#define CHASSIS_TIMEOUT_MAX_MS              120000ULL

static const WheelCommand *wheel_for_id(const MecanumWheelCommand *wheels, uint8_t id)
{
    switch (id) {
    case 1U: return &wheels->fl;
    case 2U: return &wheels->rl;
    case 3U: return &wheels->rr;
    default: return &wheels->fr;
    }
}

static HAL_StatusTypeDef send_motion_to_wheel(const Chassis *chassis, uint8_t id)
{
    const WheelCommand *wheel = wheel_for_id(&chassis->active_wheels, id);

    /* 位置目标为 0 的轮子不参与本次位置动作。 */
    if (chassis->active_mode == CHASSIS_MODE_POSITION && wheel->pulses == 0U) return HAL_OK;

    if (chassis->active_mode == CHASSIS_MODE_POSITION) {
        /*
         * raF 保持 0U：沿用当前工程/官方例程已验证的相对位置模式；
         * sync_wait=true：四轮先缓存，最后由 0xFF 同步启动。
         */
        return emm_pos(id, wheel->dir, wheel->rpm, chassis->acceleration,
                       wheel->pulses, 0U, true);
    }

    return emm_vel(id, wheel->dir, wheel->rpm, chassis->acceleration, true);
}

static void advance_wheel_state(Chassis *chassis, uint32_t now_ms)
{
    chassis->tx_state = (ChassisTxState)((uint32_t)chassis->tx_state + 1U);
    chassis->next_tx_ms = now_ms + CHASSIS_SYNC_GAP_MS;
}

/*
 * EMM Emm 固件 acc：
 *   acc=0 -> 直接按设定速度启动；
 *   acc!=0 -> 每 (256-acc)*50us 增/减 1 RPM。
 *
 * 这里使用保守估计：
 *   恒速理论时间 + 加速时间 + 减速时间
 * 然后再乘安全系数并加固定余量。
 * 这样动态时间只负责“最多允许多久”，不再拿它判断“已经到位”。
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
            ramp_ms = (ramp_ticks_50us + 19ULL) / 20ULL; /* 50us = 0.05ms */
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
                            chassis->target_z, chassis->speed_limit, &chassis->active_wheels);

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

static void fail_motion_dispatch(Chassis *chassis, uint32_t now_ms)
{
    if (chassis->active_mode == CHASSIS_MODE_POSITION) {
        chassis->position_failed = true;
        chassis->position_done = false;
    }

    chassis->tx_failed = true;
    chassis->dirty = false;
    chassis->velocity_watchdog_armed = false;
    chassis->emergency_stop = true;
    begin_stop(chassis, now_ms);
}

static void fail_position_motion(Chassis *chassis, uint32_t now_ms)
{
    chassis->position_failed = true;
    chassis->position_done = false;
    chassis->emergency_stop = true;
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

static bool velocity_input_valid(float vx, float vy, float wz)
{
    return isfinite(vx) && isfinite(vy) && isfinite(wz);
}

static bool position_input_valid(float dx, float dy, float dtheta, float speed)
{
    return isfinite(dx) && isfinite(dy) && isfinite(dtheta) &&
           isfinite(speed) && speed > 0.0f;
}

void chassis_init(Chassis *chassis, const MecanumConfig *config)
{
    *chassis = (Chassis){
        .config = *config,
        .acceleration = 100U,
        .watchdog_ms = 150U,
        .position_timeout_ms = 0U, /* 0 = 动态超时 */
        .mode = CHASSIS_MODE_VELOCITY,
        .active_mode = CHASSIS_MODE_VELOCITY,
        .tx_state = CHASSIS_TX_IDLE,
        .status_query_id = 1U
    };
}

void chassis_set_velocity(Chassis *chassis, float vx, float vy, float wz, uint32_t now_ms)
{
    /* NaN/Inf must never reach the kinematics or float-to-integer conversions. */
    if (!velocity_input_valid(vx, vy, wz)) {
        chassis_stop(chassis);
        return;
    }

    chassis->mode = CHASSIS_MODE_VELOCITY;
    chassis->target_x = vx;
    chassis->target_y = vy;
    chassis->target_z = wz;
    chassis->last_command_ms = now_ms;
    chassis->emergency_stop = false;
    chassis->velocity_watchdog_armed = true;
    chassis->tx_failed = false;
    chassis->dirty = true;
}

void chassis_set_position(Chassis *chassis, float dx, float dy, float dtheta,
                          float speed, uint32_t now_ms)
{
    /* Position speed is a positive magnitude; reject NaN/Inf/zero/negative inputs safely. */
    if (!position_input_valid(dx, dy, dtheta, speed)) {
        chassis_stop(chassis);
        chassis->position_done = false;
        chassis->position_failed = true;
        return;
    }

    chassis->mode = CHASSIS_MODE_POSITION;
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
}

void chassis_set_position_timeout(Chassis *chassis, uint32_t timeout_ms)
{
    chassis->position_timeout_ms = timeout_ms;
}

void chassis_stop(Chassis *chassis)
{
    /* 主动停车或速度超时后解除速度看门狗，避免重复触发 STOP。 */
    chassis->velocity_watchdog_armed = false;
    /* 显式 stop 允许从此前 TX fault 中重新尝试停车。 */
    chassis->tx_failed = false;
    /* Only abort a position command that is actually pending or still in progress.
     * A position command that already completed must keep position_done=true.
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
}

bool chassis_position_done(const Chassis *chassis)
{
    return chassis->position_done;
}

bool chassis_position_failed(const Chassis *chassis)
{
    return chassis->position_failed;
}

bool chassis_tx_failed(const Chassis *chassis)
{
    return chassis->tx_failed;
}

bool chassis_busy(const Chassis *chassis)
{
    /* A requested STOP is busy immediately, even before the next update starts STOP_1.
     * If the STOP sequence itself times out, tx_failed releases busy so upper layers do not hang.
     */
    return chassis->dirty ||
           chassis->tx_state != CHASSIS_TX_IDLE ||
           (chassis->emergency_stop && !chassis->tx_failed);
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
            chassis->position_feedback_not_before_ms = now_ms + CHASSIS_STATUS_QUERY_GUARD_MS;
            chassis->position_deadline_ms = now_ms + chassis->active_position_timeout_ms;
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

        /* E2/EE 或 0x3A 堵转/堵转保护：立即失败，不等到 timeout。 */
        if (emm_any_position_fault_after(chassis->motor_mask, chassis->position_start_ms)) {
            fail_position_motion(chassis, now_ms);
            return;
        }

        /*
         * 正常完成条件：四个参与运动的电机都在本次启动后的主动查询中
         * 返回 0x3A，并且 Prf_TF(bit1)=1。
         */
        if (emm_all_position_reached_after(chassis->motor_mask,
                                           chassis->position_feedback_not_before_ms)) {
            chassis->position_done = true;
            chassis->position_failed = false;
            chassis->tx_state = CHASSIS_TX_IDLE;
            return;
        }

        /* 动态时间只做故障兜底，不把“时间到了”当成“已经到位”。 */
        if ((int32_t)(now_ms - chassis->position_deadline_ms) >= 0) {
            fail_position_motion(chassis, now_ms);
            return;
        }

        if ((int32_t)(now_ms - chassis->next_status_query_ms) < 0) return;

        query_id = next_active_motor_id(chassis->motor_mask, chassis->status_query_id);
        if (query_id == 0U) return;

        status = emm_read_status(query_id);
        if (status == HAL_OK) {
            chassis->status_query_id = (query_id >= 4U) ? 1U : (uint8_t)(query_id + 1U);
            chassis->next_status_query_ms = now_ms + CHASSIS_STATUS_QUERY_INTERVAL_MS;
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
        } else {
            chassis->tx_state = (ChassisTxState)((uint32_t)chassis->tx_state + 1U);
            chassis->next_tx_ms = now_ms;
        }
        return;
    }

    default:
        fail_position_motion(chassis, now_ms);
        return;
    }
}
