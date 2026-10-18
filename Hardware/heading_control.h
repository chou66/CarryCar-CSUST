#pragma once

#include <stdbool.h>

typedef struct {
    float kp;
    float kd;
    float max_w_radps;
    float min_w_radps;
    float tolerance_deg;
    float target_deg;
    float last_error_deg;
    bool enabled;
} HeadingController;

void heading_init(HeadingController *ctrl, float kp, float kd, float max_w_radps, float tolerance_deg);
void heading_set_min_output(HeadingController *ctrl, float min_w_radps);
void heading_set_target(HeadingController *ctrl, float target_deg);
/* dt_s must be the fixed scheduler period (recommended 0.01 s). */
float heading_update(HeadingController *ctrl, float yaw_deg, float gyro_z_dps, float dt_s);

/*
 * 世界航向专用闭环：
 *   world yaw / world gyro: 顺时针为正；
 *   返回的 chassis wz:     逆时针为正。
 *
 * 因此本函数内部完成符号变换，并用 gyro 项在接近目标时提前制动。
 * 输出同时受 min_w_radps/max_w_radps 限制：远处快速，近处慢速，
 * 避免把“提高纠偏速度”做成全程固定高速而造成反复超调。
 */
float heading_update_world_cw(HeadingController *ctrl,
                              float world_yaw_deg,
                              float world_gyro_dps,
                              float dt_s);
bool heading_arrived(const HeadingController *ctrl, float yaw_deg);
