#include "chassis_motion.h"
#include "chassis_config.h"

#define MOTION_PI_F 3.14159265358979323846f

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

void chassis_motion_move(Chassis *chassis,
                         float dx_m, float dy_m, float dtheta_rad,
                         float speed_mps, uint32_t now_ms)
{
    float calibrated_x;
    float calibrated_y;
    float calibrated_theta;

    /*
     * 关键修复：所有方向标定都放在 signed core 入口。
     * 节点寻路直接传入 dx/dy 时同样会经过这里，不会绕过标定宏。
     */
    calibrated_x = apply_signed_scale(dx_m,
                                      MOTION_CAL_X_POS_SCALE,
                                      MOTION_CAL_X_NEG_SCALE);
    calibrated_y = apply_signed_scale(dy_m,
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
    calibrated_y += strafe_to_y_compensation(dx_m);
    calibrated_x += y_to_x_compensation(dy_m);
    calibrated_theta = apply_signed_scale(dtheta_rad,
                                          MOTION_CAL_THETA_POS_SCALE,
                                          MOTION_CAL_THETA_NEG_SCALE);
    calibrated_theta += y_to_theta_compensation(dy_m);
    calibrated_theta += x_to_theta_compensation(dx_m);

    chassis_set_position(chassis,
                         calibrated_x,
                         calibrated_y,
                         calibrated_theta,
                         speed_mps,
                         now_ms);
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

void chassis_motion_rotate(Chassis *chassis,
                           float dtheta_rad, float speed_mps, uint32_t now_ms)
{
    chassis_motion_move(chassis, 0.0f, 0.0f, dtheta_rad, speed_mps, now_ms);
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
