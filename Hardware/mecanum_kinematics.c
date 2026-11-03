#include "mecanum_kinematics.h"
#include <math.h>

static WheelCommand make_command(const MecanumConfig *cfg, uint8_t id, float rpm_signed, float pulses_signed)
{
    WheelCommand out;
    /* 速度模式直接用 rpm 符号，位置模式用 pulses 符号 */
    float sign_val = pulses_signed != 0.0f ? pulses_signed : rpm_signed;
    
    out.dir = sign_val >= 0.0f ? cfg->forward_dir[id] : (uint8_t)!cfg->forward_dir[id];
    out.rpm = (uint16_t)(fabsf(rpm_signed) + 0.5f);
    out.pulses = (uint32_t)(fabsf(pulses_signed) + 0.5f);
    return out;
}

void mecanum_inverse(const MecanumConfig *cfg, float vx, float vy, float wz,
                     MecanumWheelCommand *out)
{
    const float k = cfg->half_length_m + cfg->half_width_m;
    const float factor = 60.0f * cfg->gear_ratio / (2.0f * 3.141592653589793f * cfg->wheel_radius_m);
    
    /* 坐标系：vx 向右为正(+X)，vy 向上/向前为正(+Y) */
    float fl = (vy + vx - k * wz) * factor;
    float fr = (vy - vx + k * wz) * factor;
    float rl = (vy - vx - k * wz) * factor;
    float rr = (vy + vx + k * wz) * factor;
    
    float peak = fmaxf(fmaxf(fabsf(fl), fabsf(fr)), fmaxf(fabsf(rl), fabsf(rr)));
    if (peak > cfg->max_motor_rpm) {
        const float scale = cfg->max_motor_rpm / peak;
        fl *= scale; fr *= scale; rl *= scale; rr *= scale;
    }
    
    out->fl = make_command(cfg, 1U, fl, 0.0f);
    out->rl = make_command(cfg, 2U, rl, 0.0f);
    out->rr = make_command(cfg, 3U, rr, 0.0f);
    out->fr = make_command(cfg, 4U, fr, 0.0f);
}

void mecanum_inverse_pulses(const MecanumConfig *cfg,
                            float dx, float dy, float dtheta,
                            float pulses[4])
{
    const float k = cfg->half_length_m + cfg->half_width_m;
    const float theoretical_pulses_per_m = cfg->motor_pulses_per_rev * cfg->gear_ratio /
                                           (2.0f * 3.141592653589793f * cfg->wheel_radius_m);
    const float forward_pulses_per_m = (cfg->forward_pulses_per_m > 0.0f) ?
                                       cfg->forward_pulses_per_m : theoretical_pulses_per_m;
    const float strafe_pulses_per_m = (cfg->strafe_pulses_per_m > 0.0f) ?
                                      cfg->strafe_pulses_per_m : theoretical_pulses_per_m;
    const float rotate_pulses_per_rad = (cfg->rotate_pulses_per_rad > 0.0f) ?
                                       cfg->rotate_pulses_per_rad :
                                       k * theoretical_pulses_per_m;

    if (!pulses) return;

    /* 坐标系：dx 向右为正(+X)，dy 向上/向前为正(+Y) */
    pulses[0] = dy * forward_pulses_per_m + dx * strafe_pulses_per_m - dtheta * rotate_pulses_per_rad;
    pulses[1] = dy * forward_pulses_per_m - dx * strafe_pulses_per_m - dtheta * rotate_pulses_per_rad;
    pulses[2] = dy * forward_pulses_per_m + dx * strafe_pulses_per_m + dtheta * rotate_pulses_per_rad;
    pulses[3] = dy * forward_pulses_per_m - dx * strafe_pulses_per_m + dtheta * rotate_pulses_per_rad;
}

void mecanum_inverse_pos(const MecanumConfig *cfg, float dx, float dy, float dtheta,
                         float speed_mps, MecanumWheelCommand *out)
{
    float pulses[4];

    mecanum_inverse_pulses(cfg, dx, dy, dtheta, pulses);

    /* 顺序与 mecanum_inverse_pulses 一致：FL, RL, RR, FR。 */
    float p_fl = pulses[0];
    float p_rl = pulses[1];
    float p_rr = pulses[2];
    float p_fr = pulses[3];

    float peak_pulse = fmaxf(fmaxf(fabsf(p_fl), fabsf(p_fr)), fmaxf(fabsf(p_rl), fabsf(p_rr)));
    float rpm_fl = 0.0f, rpm_fr = 0.0f, rpm_rl = 0.0f, rpm_rr = 0.0f;
    
    if (peak_pulse > 0.5f) {
        float max_rpm = speed_mps * 60.0f * cfg->gear_ratio / (2.0f * 3.141592653589793f * cfg->wheel_radius_m);
        if (max_rpm > cfg->max_motor_rpm) max_rpm = cfg->max_motor_rpm;
        
        float ratio = max_rpm / peak_pulse;
        rpm_fl = fabsf(p_fl) * ratio;
        rpm_fr = fabsf(p_fr) * ratio;
        rpm_rl = fabsf(p_rl) * ratio;
        rpm_rr = fabsf(p_rr) * ratio;
        
        /* 设置最小允许转速防止个别轮子停转卡死 */
        float min_rpm = (max_rpm < 25.0f) ? max_rpm : 25.0f;
        if (fabsf(p_fl) >= 1.0f && rpm_fl < min_rpm) rpm_fl = min_rpm;
        if (fabsf(p_fr) >= 1.0f && rpm_fr < min_rpm) rpm_fr = min_rpm;
        if (fabsf(p_rl) >= 1.0f && rpm_rl < min_rpm) rpm_rl = min_rpm;
        if (fabsf(p_rr) >= 1.0f && rpm_rr < min_rpm) rpm_rr = min_rpm;
    }

    out->fl = make_command(cfg, 1U, rpm_fl, p_fl);
    out->rl = make_command(cfg, 2U, rpm_rl, p_rl);
    out->rr = make_command(cfg, 3U, rpm_rr, p_rr);
    out->fr = make_command(cfg, 4U, rpm_fr, p_fr);
}
