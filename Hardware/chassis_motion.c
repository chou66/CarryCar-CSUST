#include "chassis_motion.h"
#include "chassis_config.h"
#include <math.h>

#define MOTION_PI_F 3.14159265358979323846f
#define MOTION_MIN_DISTANCE_M 0.000001f
#define MOTION_MIN_VECTOR_M   0.0001f

static float apply_signed_scale(float value, float positive_scale, float negative_scale)
{
    if (value > 0.0f) return value * positive_scale;
    if (value < 0.0f) return value * negative_scale;
    return 0.0f;
}

/*
 * Pure strafe on the real chassis has a repeatable +Y (forward) coupling.
 * Apply a small -Y command proportional to |dx| to cancel it.
 *
 * +X = right:
 *   0.85 m right produced about +0.035 m forward drift.
 *
 * -X = left:
 *   0.85 m left produced about +0.047 m forward drift.
 */
static float strafe_to_y_compensation(float dx_m)
{
    if (dx_m > 0.0f)
        return dx_m * MOTION_CAL_X_POS_TO_Y;

    if (dx_m < 0.0f)
        return (-dx_m) * MOTION_CAL_X_NEG_TO_Y;

    return 0.0f;
}

/*
 * Mirror of the strafe compensation above for the other axis: pure
 * forward/backward motion couples into X (measured as lateral creep,
 * e.g. backward drifting left). Apply a small signed X command
 * proportional to |dy| to cancel it.
 */
static float y_to_x_compensation(float dy_m)
{
    if (dy_m > 0.0f)
        return dy_m * MOTION_CAL_Y_POS_TO_X;

    if (dy_m < 0.0f)
        return (-dy_m) * MOTION_CAL_Y_NEG_TO_X;

    return 0.0f;
}

/*
 * Mirror of the cross-axis compensations above for heading: pure
 * forward/backward motion couples into theta (measured as heading
 * creep, e.g. forward 0.45 m yawing about +0.9 deg CCW on the
 * current field). Apply a small signed theta command proportional
 * to |dy| to cancel it; mecanum_inverse_pos() blends it into the
 * same position move, so the counter-rotation rides along with the
 * straight move instead of being a separate turn.
 */
static float y_to_theta_compensation(float dy_m)
{
    if (dy_m > 0.0f)
        return dy_m * MOTION_CAL_Y_POS_TO_THETA;

    if (dy_m < 0.0f)
        return (-dy_m) * MOTION_CAL_Y_NEG_TO_THETA;

    return 0.0f;
}

/*
 * Mirror of the heading compensation above for strafe moves: pure
 * lateral motion couples into theta (measured as right strafe 0.45 m
 * yawing about +0.9 deg CCW on the current field). Apply a small
 * signed theta command proportional to |dx| to cancel it.
 */
static float x_to_theta_compensation(float dx_m)
{
    if (dx_m > 0.0f)
        return dx_m * MOTION_CAL_X_POS_TO_THETA;

    if (dx_m < 0.0f)
        return (-dx_m) * MOTION_CAL_X_NEG_TO_THETA;

    return 0.0f;
}

static float motion_max_wheel_mps(const Chassis *chassis)
{
    if (!chassis ||
        chassis->config.wheel_radius_m <= 0.0f ||
        chassis->config.gear_ratio <= 0.0f)
        return 0.0f;

    return (float)chassis->config.max_motor_rpm * 2.0f * MOTION_PI_F *
           chassis->config.wheel_radius_m /
           (60.0f * chassis->config.gear_ratio);
}

/* 规划与速度矢量换算必须用同一个（已限幅）速度，否则实际行程会变短。 */
static float motion_limit_wheel_mps(const Chassis *chassis, float wheel_mps)
{
    float max_wheel_mps = motion_max_wheel_mps(chassis);

    if (wheel_mps > max_wheel_mps)
        return max_wheel_mps;
    return (wheel_mps > 0.0f) ? wheel_mps : 0.0f;
}

static float motion_peak_pulses(const float pulses[4])
{
    float peak = 0.0f;
    uint8_t i;

    for (i = 0U; i < 4U; ++i) {
        float a = fabsf(pulses[i]);
        if (a > peak) peak = a;
    }
    return peak;
}

/*
 * 由目标脉冲数反算正弦剖面的加速段/匀速段时长。
 *
 * 曲线对时间的积分是 峰值RPM * (ramp + cruise)，对应电机总脉冲：
 *   pulses = ∫rpm dt / 60 * pulses_per_rev
 * 因此先求 ∫rpm dt = pulses * 60 / pulses_per_rev，再按峰值拆分。
 * 行程太短时不再保留匀速段，直接把加速/减速段压缩到总时长的一半。
 */
static void motion_plan_profile(const Chassis *chassis,
                                float peak_pulses,
                                float wheel_mps,
                                uint32_t *ramp_ms,
                                uint32_t *cruise_ms)
{
    float rpm_peak;
    float rpm_seconds;
    float ramp_s;
    float cruise_s;

    *ramp_ms = 0U;
    *cruise_ms = 0U;

    if (!chassis || peak_pulses < 0.5f ||
        chassis->config.motor_pulses_per_rev == 0U)
        return;

    if (wheel_mps > motion_max_wheel_mps(chassis))
        wheel_mps = motion_max_wheel_mps(chassis);

    if (wheel_mps <= 0.0f)
        return;

    rpm_peak = wheel_mps * 60.0f * chassis->config.gear_ratio /
               (2.0f * MOTION_PI_F * chassis->config.wheel_radius_m);
    if (rpm_peak <= 0.0f)
        return;

    rpm_seconds = peak_pulses * 60.0f /
                  (float)chassis->config.motor_pulses_per_rev;

    ramp_s = (float)CHASSIS_PROFILE_RAMP_MS / 1000.0f;

    if (rpm_seconds >= rpm_peak * ramp_s) {
        cruise_s = rpm_seconds / rpm_peak - ramp_s;
    } else {
        /* 短行程：没有匀速段，峰值仍然只到调用方给的速度。 */
        ramp_s = rpm_seconds / rpm_peak;
        cruise_s = 0.0f;
    }

    if (ramp_s < 0.001f) ramp_s = 0.001f;

    *ramp_ms = (uint32_t)(ramp_s * 1000.0f + 0.5f);
    *cruise_ms = (uint32_t)(cruise_s * 1000.0f + 0.5f);
}

/*
 * 与 motion_plan_profile 相同，但允许首/尾不做加减速（拐角衔接段）。
 *
 * 曲线对时间的积分是 峰值RPM * (ramp_in*ramp + cruise + ramp_out*ramp)，
 * 因此 cruise 按实际参与加速的段数反算。
 */
static void motion_plan_profile_blend(const Chassis *chassis,
                                      float peak_pulses,
                                      float wheel_mps,
                                      uint8_t ramp_in,
                                      uint8_t ramp_out,
                                      uint32_t *ramp_ms,
                                      uint32_t *cruise_ms)
{
    float rpm_peak;
    float rpm_seconds;
    float ramp_s;
    float cruise_s;
    float ramp_count;

    *ramp_ms = 0U;
    *cruise_ms = 0U;

    if (!chassis || peak_pulses < 0.5f ||
        chassis->config.motor_pulses_per_rev == 0U)
        return;

    if (wheel_mps > motion_max_wheel_mps(chassis))
        wheel_mps = motion_max_wheel_mps(chassis);

    if (wheel_mps <= 0.0f)
        return;

    rpm_peak = wheel_mps * 60.0f * chassis->config.gear_ratio /
               (2.0f * MOTION_PI_F * chassis->config.wheel_radius_m);
    if (rpm_peak <= 0.0f)
        return;

    rpm_seconds = peak_pulses * 60.0f /
                  (float)chassis->config.motor_pulses_per_rev;

    ramp_s = (float)CHASSIS_PROFILE_RAMP_MS / 1000.0f;
    ramp_count = (float)((ramp_in ? 1U : 0U) + (ramp_out ? 1U : 0U));

    /*
     * 正弦斜坡对时间的积分只有满速的一半，所以每段斜坡贡献的“满速等效
     * 时长”是 ramp/2；ramp_count 段相加。
     */
    if (ramp_count <= 0.0f) {
        cruise_s = rpm_seconds / rpm_peak;
    } else if (rpm_seconds >= rpm_peak * 0.5f * ramp_s * ramp_count) {
        cruise_s = rpm_seconds / rpm_peak -
                   0.5f * ramp_s * ramp_count;
    } else {
        /* 距离不足以完成加速段：压缩斜坡，保证行程积分不变。 */
        ramp_s = 2.0f * rpm_seconds / (rpm_peak * ramp_count);
        cruise_s = 0.0f;
    }

    if (ramp_count > 0.0f && ramp_s < 0.001f) ramp_s = 0.001f;

    *ramp_ms = (uint32_t)(ramp_s * 1000.0f + 0.5f);
    *cruise_ms = (uint32_t)(cruise_s * 1000.0f + 0.5f);
}

/*
 * 把“各轮脉冲比例”还原成车体系峰值速度矢量。
 *
 * 速度模式的逆解为：
 *   fl = vy + vx - k*wz;  fr = vy - vx + k*wz;
 *   rl = vy - vx - k*wz;  rr = vy + vx + k*wz
 * 反解得 vx/vy/wz。由于各轮比例直接来自位置模式的标定脉冲分配，
 * 速度剖面对时间的积分就等于原来的位置脉冲，距离精度不随模式改变。
 */
static void motion_velocity_from_pulses(const Chassis *chassis,
                                        const float pulses[4],
                                        float wheel_mps,
                                        float *vx_mps,
                                        float *vy_mps,
                                        float *wz_radps)
{
    float peak = motion_peak_pulses(pulses);
    float k = chassis->config.half_length_m + chassis->config.half_width_m;
    float s_fl;
    float s_rl;
    float s_rr;
    float s_fr;

    *vx_mps = 0.0f;
    *vy_mps = 0.0f;
    *wz_radps = 0.0f;

    if (peak < 0.5f || k <= 0.0f)
        return;

    /* 顺序：FL, RL, RR, FR。 */
    s_fl = pulses[0] / peak;
    s_rl = pulses[1] / peak;
    s_rr = pulses[2] / peak;
    s_fr = pulses[3] / peak;

    *vx_mps = (s_fl - s_fr - s_rl + s_rr) * 0.25f * wheel_mps;
    *vy_mps = (s_fl + s_fr + s_rl + s_rr) * 0.25f * wheel_mps;
    *wz_radps = (s_fr + s_rr - s_fl - s_rl) * 0.25f / k * wheel_mps;
}

/*
 * 所有方向标定都放在 signed core 入口。节点寻路直接传入 dx/dy 时同样会
 * 经过这里，不会绕过标定宏。速度剖面和位置模式共用这一份换算，保证两种
 * 执行方式对同一目标给出相同的轮子转向与行程比例。
 */
static void apply_motion_calibration(float dx_m, float dy_m, float dtheta_rad,
                                     float *calibrated_x,
                                     float *calibrated_y,
                                     float *calibrated_theta)
{
    *calibrated_x = apply_signed_scale(dx_m,
                                       MOTION_CAL_X_POS_SCALE,
                                       MOTION_CAL_X_NEG_SCALE);
    *calibrated_y = apply_signed_scale(dy_m,
                                       MOTION_CAL_Y_POS_SCALE,
                                       MOTION_CAL_Y_NEG_SCALE);

    /*
     * Cross-axis compensation is added after the normal Y scale.
     * Most map edges are pure X or pure Y, where this only cancels the
     * measured strafe drift. Bay entry/exit and N7-N8 also
     * pass through here: the compensation stays proportional to |dx|,
     * which is how the physical coupling behaves, but its magnitude was
     * calibrated on pure strafe moves only.
     */
    *calibrated_y += strafe_to_y_compensation(dx_m);
    *calibrated_x += y_to_x_compensation(dy_m);
    *calibrated_theta = apply_signed_scale(dtheta_rad,
                                           MOTION_CAL_THETA_POS_SCALE,
                                           MOTION_CAL_THETA_NEG_SCALE);
    *calibrated_theta += y_to_theta_compensation(dy_m);
    *calibrated_theta += x_to_theta_compensation(dx_m);
}

void chassis_motion_move(Chassis *chassis,
                         float dx_m, float dy_m, float dtheta_rad,
                         float speed_mps, uint32_t now_ms)
{
    float calibrated_x;
    float calibrated_y;
    float calibrated_theta;
    float pulses[4];
    float peak_pulses;
    float wheel_mps;
    float vx_peak;
    float vy_peak;
    float wz_peak;
    uint32_t ramp_ms;
    uint32_t cruise_ms;

    apply_motion_calibration(dx_m, dy_m, dtheta_rad,
                             &calibrated_x, &calibrated_y,
                             &calibrated_theta);

    /*
     * 导航/大位移仍走速度模式正弦剖面：同一套脉冲分配得到各轮比例，
     * 再换算成峰值速度与加速/匀速时长。
     */
    mecanum_inverse_pulses(&chassis->config,
                           calibrated_x, calibrated_y, calibrated_theta,
                           pulses);

    peak_pulses = motion_peak_pulses(pulses);
    wheel_mps = motion_limit_wheel_mps(chassis, speed_mps);

    motion_plan_profile(chassis, peak_pulses, wheel_mps, &ramp_ms, &cruise_ms);
    motion_velocity_from_pulses(chassis, pulses, wheel_mps,
                                &vx_peak, &vy_peak, &wz_peak);

    chassis_set_profile(chassis, vx_peak, vy_peak, wz_peak,
                        ramp_ms, cruise_ms, now_ms);
}

/*
 * 精确小位移入口：走 EMM 位置模式（0xFD），由驱动器按目标脉冲闭环到位。
 *
 * 为什么需要单独一条路径：视觉对位每次只剩几毫米甚至 0.5 mm，速度剖面
 * 的 ramp 会被压缩到 4~8 ms，短于 CHASSIS_PROFILE_UPDATE_MS（10 ms），
 * 整段位移常常一帧非零速度都发不出去，或只发一帧就被 STOP 打断，表现为
 * 车轮原地一顿一挫。位置模式把脉冲一次交给驱动器，0.12 m/s 的加减速由
 * EMM 内部完成，不存在时间量化。
 *
 * 用于 K230 视觉微调，以及原地旋转（chassis_motion_rotate 也走这里）；
 * 导航的直线平动和圆弧仍用 chassis_motion_move / chassis_motion_arc_world_ex。
 */
void chassis_motion_move_precise(Chassis *chassis,
                                 float dx_m, float dy_m, float dtheta_rad,
                                 float speed_mps, uint32_t now_ms)
{
    float calibrated_x;
    float calibrated_y;
    float calibrated_theta;

    apply_motion_calibration(dx_m, dy_m, dtheta_rad,
                             &calibrated_x, &calibrated_y,
                             &calibrated_theta);

    chassis_set_position(chassis, calibrated_x, calibrated_y,
                         calibrated_theta, speed_mps, now_ms);
}

void chassis_motion_translate_precise(Chassis *chassis,
                                      float dx_m, float dy_m,
                                      float speed_mps, uint32_t now_ms)
{
    chassis_motion_move_precise(chassis, dx_m, dy_m, 0.0f, speed_mps, now_ms);
}

/*
 * 漂移路径：沿拓扑边的世界方向行驶，同时把车头锁在目标航向上。
 *
 * world_dx/world_dy 是边的世界坐标向量 from->to（只用来取方向）；
 * distance_m 是车身 Y 方向的有符号距离（负值=后退）；
 * 目标航向在后退时已经是“车头朝反方向”的角度，由 edge_turn 给出。
 *
 * 关键：世界位移方向永远是 from->to 的边方向，与前进/后退无关。
 * 后退时车头朝边方向的反方向，把同一个世界速度矢量旋到车体系后
 * 自然得到负的 vy（车身 -Y 行驶），所以这里不能再乘 distance_m 的符号。
 *
 * 位移脉冲仍按车身 Y 轴 + forward_pulses_per_m 标定分配，
 * 运行时由底盘层按当前 IMU 航向把速度矢量旋到车体系，
 * 因此车头有偏差时车体中心仍沿世界直线前进（边转边走）。
 * 因为航向由闭环维持，这里不再叠加 MOTION_CAL_*_TO_THETA 开环前馈。
 */
void chassis_motion_drift_translate(Chassis *chassis,
                                    float world_dx_m, float world_dy_m,
                                    float distance_m,
                                    float target_world_yaw_deg,
                                    float speed_mps, uint32_t now_ms)
{
    chassis_motion_drift_segment(chassis, world_dx_m, world_dy_m, distance_m,
                                 target_world_yaw_deg, speed_mps,
                                 1U, 1U, now_ms);
}

void chassis_motion_drift_segment(Chassis *chassis,
                                  float world_dx_m, float world_dy_m,
                                  float distance_m,
                                  float target_world_yaw_deg,
                                  float speed_mps,
                                  uint8_t ramp_in, uint8_t ramp_out,
                                  uint32_t now_ms)
{
    float pulses[4];
    float peak_pulses;
    float wheel_mps;
    float length;
    float unit_x;
    float unit_y;
    uint32_t ramp_ms;
    uint32_t cruise_ms;

    if (!chassis)
        return;

    length = sqrtf(world_dx_m * world_dx_m + world_dy_m * world_dy_m);

    if (length < MOTION_MIN_VECTOR_M ||
        fabsf(distance_m) < MOTION_MIN_DISTANCE_M) {
        chassis_set_profile(chassis, 0.0f, 0.0f, 0.0f, 0U, 0U, now_ms);
        return;
    }

    mecanum_inverse_pulses(&chassis->config, 0.0f, distance_m, 0.0f, pulses);
    peak_pulses = motion_peak_pulses(pulses);
    wheel_mps = motion_limit_wheel_mps(chassis, speed_mps);

    motion_plan_profile_blend(chassis, peak_pulses, wheel_mps,
                              ramp_in, ramp_out, &ramp_ms, &cruise_ms);

    /* 平移方向恒为 from->to；后退由车体航向反过来自然得到 -Y 速度。 */
    unit_x = world_dx_m / length;
    unit_y = world_dy_m / length;

    chassis_set_profile_world_ex(chassis,
                                 unit_x * wheel_mps,
                                 unit_y * wheel_mps,
                                 target_world_yaw_deg,
                                 ramp_in, ramp_out,
                                 ramp_ms,
                                 cruise_ms,
                                 now_ms);
}

void chassis_motion_arc_world_ex(Chassis *chassis,
                                 float start_tangent_world_yaw_deg,
                                 float turn_deg,
                                 float radius_m,
                                 float speed_mps,
                                 uint8_t reverse,
                                 uint32_t now_ms)
{
    float arc_len_m;
    float duration_s;
    float yaw_rate_dps;
    uint32_t total_ms;

    if (!chassis || radius_m <= 0.0f || speed_mps <= 0.0f)
        return;

    arc_len_m = fabsf(turn_deg) * MOTION_PI_F / 180.0f * radius_m;
    if (arc_len_m < MOTION_MIN_DISTANCE_M) {
        chassis_set_profile(chassis, 0.0f, 0.0f, 0.0f, 0U, 0U, now_ms);
        return;
    }

    duration_s = arc_len_m / speed_mps;
    total_ms = (uint32_t)(duration_s * 1000.0f + 0.5f);
    if (total_ms == 0U)
        total_ms = 1U;

    /* 转角 / 总时长 = 世界系角速度（顺时针为正）。 */
    yaw_rate_dps = turn_deg / ((float)total_ms * 0.001f);

    chassis_set_profile_world_arc_ex(chassis,
                                     start_tangent_world_yaw_deg,
                                     yaw_rate_dps,
                                     speed_mps,
                                     reverse,
                                     total_ms,
                                     now_ms);
}

void chassis_motion_arc_world(Chassis *chassis,
                              float start_world_yaw_deg,
                              float turn_deg,
                              float radius_m,
                              float speed_mps,
                              uint32_t now_ms)
{
    chassis_motion_arc_world_ex(chassis, start_world_yaw_deg, turn_deg,
                                radius_m, speed_mps, 0U, now_ms);
}

void chassis_motion_translate(Chassis *chassis,
                              float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms)
{
    chassis_motion_move(chassis, dx_m, dy_m, 0.0f, speed_mps, now_ms);
}

void chassis_motion_move_x(Chassis *chassis,
                           float dx_m, float speed_mps, uint32_t now_ms)
{
    chassis_motion_move(chassis, dx_m, 0.0f, 0.0f, speed_mps, now_ms);
}

void chassis_motion_move_y(Chassis *chassis,
                           float dy_m, float speed_mps, uint32_t now_ms)
{
    chassis_motion_move(chassis, 0.0f, dy_m, 0.0f, speed_mps, now_ms);
}

/*
 * 原地旋转回到 EMM 位置模式（0xFD），由驱动器按目标脉冲闭环到位。
 *
 * 纯旋转没有平动分量：目标角度直接乘以旋转半径换算成各轮脉冲，一次交给
 * 驱动器即可。速度剖面模式需要把同样的脉冲拆成 ramp/cruise，90° 这种中等
 * 角度下 ramp 只剩几十毫秒，与控制周期同量级，量化误差和首帧丢失都会直接
 * 变成角度误差/过冲，实车表现为“转不准”。位置模式不存在时间量化。
 *
 * 平动和圆弧仍走 chassis_motion_move / chassis_motion_arc_world_ex 的速度
 * 剖面。旋转入口保留 chassis_motion_rotate_deg 等原函数名，调用方无需改动。
 */
void chassis_motion_rotate(Chassis *chassis,
                           float dtheta_rad, float speed_mps, uint32_t now_ms)
{
    chassis_motion_move_precise(chassis, 0.0f, 0.0f, dtheta_rad,
                                speed_mps, now_ms);
}

void chassis_motion_rotate_deg(Chassis *chassis,
                               float angle_deg, float speed_mps, uint32_t now_ms)
{
    chassis_motion_rotate(chassis,
                          angle_deg * MOTION_PI_F / 180.0f,
                          speed_mps,
                          now_ms);
}

void chassis_motion_forward(Chassis *chassis, float distance_m,
                            float speed_mps, uint32_t now_ms)
{
    chassis_motion_move_y(chassis, (distance_m >= 0.0f) ? distance_m : -distance_m,
                          speed_mps, now_ms);
}

void chassis_motion_backward(Chassis *chassis, float distance_m,
                             float speed_mps, uint32_t now_ms)
{
    float d = (distance_m >= 0.0f) ? distance_m : -distance_m;
    chassis_motion_move_y(chassis, -d, speed_mps, now_ms);
}

void chassis_motion_strafe_right(Chassis *chassis, float distance_m,
                                 float speed_mps, uint32_t now_ms)
{
    chassis_motion_move_x(chassis, (distance_m >= 0.0f) ? distance_m : -distance_m,
                          speed_mps, now_ms);
}

void chassis_motion_strafe_left(Chassis *chassis, float distance_m,
                                float speed_mps, uint32_t now_ms)
{
    float d = (distance_m >= 0.0f) ? distance_m : -distance_m;
    chassis_motion_move_x(chassis, -d, speed_mps, now_ms);
}

void chassis_motion_rotate_ccw_deg(Chassis *chassis, float angle_deg,
                                   float speed_mps, uint32_t now_ms)
{
    float a = (angle_deg >= 0.0f) ? angle_deg : -angle_deg;
    chassis_motion_rotate_deg(chassis, a, speed_mps, now_ms);
}

void chassis_motion_rotate_cw_deg(Chassis *chassis, float angle_deg,
                                  float speed_mps, uint32_t now_ms)
{
    float a = (angle_deg >= 0.0f) ? angle_deg : -angle_deg;
    chassis_motion_rotate_deg(chassis, -a, speed_mps, now_ms);
}

bool chassis_motion_busy(const Chassis *chassis)
{
    return chassis_busy(chassis);
}

bool chassis_motion_done(const Chassis *chassis)
{
    return chassis_position_done(chassis);
}

bool chassis_motion_failed(const Chassis *chassis)
{
    return chassis_position_failed(chassis);
}

void chassis_motion_stop(Chassis *chassis)
{
    chassis_stop(chassis);
}
