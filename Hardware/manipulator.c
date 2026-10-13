#include "manipulator.h"
#include "emm_adapter.h"
#include "robot_config.h"

static uint8_t sign_dir(float x)
{
    return (x >= 0.0f) ? 0U : 1U;
}

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

HAL_StatusTypeDef Manipulator_Gimbal_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc)
{
    return emm_vel(MOTOR_GIMBAL_ID, dir, rpm, acc, false);
}

HAL_StatusTypeDef Manipulator_Slide_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc)
{
    return emm_vel(MOTOR_SLIDE_ID, dir, rpm, acc, false);
}

HAL_StatusTypeDef Manipulator_Gimbal_MovePulses(uint8_t dir, uint16_t rpm,
                                                uint8_t acc, uint32_t pulses)
{
    return emm_pos(MOTOR_GIMBAL_ID, dir, rpm, acc, pulses, false, false);
}

HAL_StatusTypeDef Manipulator_Slide_MovePulses(uint8_t dir, uint16_t rpm,
                                               uint8_t acc, uint32_t pulses)
{
    return emm_pos(MOTOR_SLIDE_ID, dir, rpm, acc, pulses, false, false);
}

//电机5升降
bool Manipulator_Gimbal_MoveMm(float mm, uint16_t rpm, uint8_t acc)
{
    uint32_t pulses;
    uint8_t direction;

    if (GIMBAL_PULSES_PER_MM <= 0.0f)
    {
        return false;
    }

    if (mm >= 0.0f)
    {
        direction = GIMBAL_DIR_UP;
    }
    else
    {
        direction = GIMBAL_DIR_DOWN;
    }

    pulses = (uint32_t)(
        absf_local(mm) * GIMBAL_PULSES_PER_MM + 0.5f);

    return Manipulator_Gimbal_MovePulses(
        direction,
        rpm,
        acc,
        pulses) == HAL_OK;
}

//电机6前进后退
bool Manipulator_Slide_MoveMm(float mm, uint16_t rpm, uint8_t acc)
{
    uint32_t pulses;
    uint8_t direction;

    if (SLIDE_PULSES_PER_MM <= 0.0f)
    {
        return false;
    }

    if (mm >= 0.0f)
    {
        direction = SLIDE_DIR_EXTEND;
    }
    else
    {
        direction = SLIDE_DIR_RETRACT;
    }

    pulses = (uint32_t)(
        absf_local(mm) * SLIDE_PULSES_PER_MM + 0.5f);

    return Manipulator_Slide_MovePulses(
        direction,
        rpm,
        acc,
        pulses) == HAL_OK;
}

void Manipulator_StopAll(void)
{
    (void)emm_stop(MOTOR_GIMBAL_ID, false);
    (void)emm_stop(MOTOR_SLIDE_ID, false);
}
