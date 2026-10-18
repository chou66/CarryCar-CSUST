#include "heading_control.h"

static float wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float clamp_abs(float value, float max_abs)
{
    if (max_abs <= 0.0f) return 0.0f;
    if (value > max_abs) return max_abs;
    if (value < -max_abs) return -max_abs;
    return value;
}

void heading_init(HeadingController *ctrl, float kp, float kd, float max_w, float tolerance)
{
    *ctrl = (HeadingController){.kp=kp, .kd=kd, .max_w_radps=max_w, .tolerance_deg=tolerance};
}

void heading_set_min_output(HeadingController *ctrl, float min_w_radps)
{
    if (!ctrl) return;
    ctrl->min_w_radps = (min_w_radps >= 0.0f) ? min_w_radps : -min_w_radps;
    if (ctrl->min_w_radps > ctrl->max_w_radps)
        ctrl->min_w_radps = ctrl->max_w_radps;
}

void heading_set_target(HeadingController *ctrl, float target_deg)
{
    ctrl->target_deg = wrap_deg(target_deg);
    ctrl->last_error_deg = 0.0f;
    ctrl->enabled = true;
}

float heading_update(HeadingController *ctrl, float yaw_deg, float gyro_z_dps, float dt_s)
{
    float err, w;
    (void)dt_s; /* gyro already gives the derivative in deg/s. */
    if (!ctrl->enabled) return 0.0f;
    err = wrap_deg(ctrl->target_deg - yaw_deg);
    if (err < ctrl->tolerance_deg && err > -ctrl->tolerance_deg) return 0.0f;
    w = (ctrl->kp * err - ctrl->kd * gyro_z_dps) * 0.01745329252f;
    if (w > ctrl->max_w_radps) w = ctrl->max_w_radps;
    if (w < -ctrl->max_w_radps) w = -ctrl->max_w_radps;
    ctrl->last_error_deg = err;
    return w;
}

float heading_update_world_cw(HeadingController *ctrl,
                              float world_yaw_deg,
                              float world_gyro_dps,
                              float dt_s)
{
    float error_world_deg;
    float world_rate_cmd_dps;
    float chassis_w_radps;

    (void)dt_s; /* gyro already supplies the derivative in deg/s. */
    if (!ctrl || !ctrl->enabled) return 0.0f;

    error_world_deg = wrap_deg(ctrl->target_deg - world_yaw_deg);
    ctrl->last_error_deg = error_world_deg;

    if (absf_local(error_world_deg) <= ctrl->tolerance_deg)
        return 0.0f;

    /* PD is calculated in the clockwise-positive world frame first. */
    world_rate_cmd_dps = ctrl->kp * error_world_deg -
                         ctrl->kd * world_gyro_dps;

    /* Chassis +w is CCW, the opposite of world-yaw positive. */
    chassis_w_radps = -world_rate_cmd_dps * 0.01745329252f;
    chassis_w_radps = clamp_abs(chassis_w_radps, ctrl->max_w_radps);

    if (absf_local(chassis_w_radps) < ctrl->min_w_radps)
    {
        if (chassis_w_radps > 0.0f)
            chassis_w_radps = ctrl->min_w_radps;
        else if (chassis_w_radps < 0.0f)
            chassis_w_radps = -ctrl->min_w_radps;
        else
            chassis_w_radps = (error_world_deg > 0.0f) ?
                               -ctrl->min_w_radps : ctrl->min_w_radps;
    }

    return chassis_w_radps;
}

bool heading_arrived(const HeadingController *ctrl, float yaw_deg)
{
    float e = wrap_deg(ctrl->target_deg - yaw_deg);
    return e < ctrl->tolerance_deg && e > -ctrl->tolerance_deg;
}
