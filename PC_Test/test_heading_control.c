#include <stdio.h>
#include "heading_control.h"

static int s_pass;
static int s_fail;

static float absf_local(float x) { return (x >= 0.0f) ? x : -x; }

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

int main(void)
{
    HeadingController ctrl;
    float w;

    heading_init(&ctrl, 1.80f, 0.35f, 1.00f, 0.50f);
    heading_set_min_output(&ctrl, 0.10f);
    heading_set_target(&ctrl, 0.0f);

    w = heading_update_world_cw(&ctrl, 90.0f, 0.0f, 0.04f);
    check(w > 0.99f && w <= 1.00f,
          "Large clockwise error saturates at fast CCW command");

    w = heading_update_world_cw(&ctrl, -90.0f, 0.0f, 0.04f);
    check(w < -0.99f && w >= -1.00f,
          "Large counter-clockwise error saturates at fast CW command");

    w = heading_update_world_cw(&ctrl, 1.0f, 0.0f, 0.04f);
    check(absf_local(w - 0.10f) < 0.001f,
          "Near target automatically drops to minimum correction speed");

    w = heading_update_world_cw(&ctrl, 0.2f, 0.0f, 0.04f);
    check(w == 0.0f, "Inside deadband stops rotation");

    /* Vehicle is still rotating CCW quickly while it approaches target.
     * The gyro damping term should command a brief CW brake before crossing. */
    w = heading_update_world_cw(&ctrl, 5.0f, -30.0f, 0.04f);
    check(w < 0.0f, "Gyro damping reverses command to brake overshoot");

    /* Task-yaw alignment uses the same controller with a wider final tolerance. */
    heading_init(&ctrl, 1.80f, 0.35f, 1.00f, 1.50f);
    heading_set_min_output(&ctrl, 0.10f);
    heading_set_target(&ctrl, -90.0f);

    w = heading_update_world_cw(&ctrl, 0.0f, 0.0f, 0.04f);
    check(w > 0.99f,
          "Task target -90 deg commands the shortest CCW chassis turn");

    w = heading_update_world_cw(&ctrl, -89.0f, 0.0f, 0.04f);
    check(w == 0.0f,
          "Task alignment releases only inside its 1.5 deg tolerance");

    heading_set_target(&ctrl, 180.0f);
    w = heading_update_world_cw(&ctrl, -179.0f, 0.0f, 0.04f);
    check(w == 0.0f,
          "Task alignment handles the plus/minus 180 deg wrap without a full turn");

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
