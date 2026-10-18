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

/* 滑台位置基准取值；含义见 manipulator.h 的说明。 */
typedef enum
{
    SLIDE_REF_UNKNOWN = 0,
    SLIDE_REF_HOME,
    SLIDE_REF_ALIGN
} SlideRef_t;

/*
 * 上电默认「确知停在机械原点」——这依赖上电时滑台确实在原点。
 * 若实车上电时滑台可能停在别处，改这里或上电后人工归零再调 SetHome()。
 */
static SlideRef_t slide_ref = SLIDE_REF_HOME;

volatile uint8_t robot_slide_ref = (uint8_t)SLIDE_REF_HOME;

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

static void set_slide_ref(SlideRef_t ref)
{
    slide_ref = ref;
    robot_slide_ref = (uint8_t)ref;
}

bool Manipulator_Slide_AtHome(void)
{
    return slide_ref == SLIDE_REF_HOME;
}

bool Manipulator_Slide_AtAlign(void)
{
    return slide_ref == SLIDE_REF_ALIGN;
}

/* 人工把滑台推回机械原点之后调用，重新立起位置基准。 */
void Manipulator_Slide_SetHome(void)
{
    set_slide_ref(SLIDE_REF_HOME);
}

/* 只在滑台点动到位后由 manipulator_action 调用。 */
void Manipulator_Slide_SetAlign(void)
{
    set_slide_ref(SLIDE_REF_ALIGN);
}

void Manipulator_StopAll(void)
{
    (void)emm_stop(MOTOR_GIMBAL_ID, false);
    (void)emm_stop(MOTOR_SLIDE_ID, false);

    /*
     * 中途截停，滑台实际停在哪里不可知，位置基准就此作废：之后所有点动
     * 都会被拒绝，直到人工归零并调用 Manipulator_Slide_SetHome()。
     * 这里不能顺手发一条「回原点」指令——急停路径不允许再动电机。
     */
    set_slide_ref(SLIDE_REF_UNKNOWN);
}
