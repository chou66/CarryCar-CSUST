#include "servo.h"
#include "tim.h"
#include "robot_config.h"

static uint16_t clamp_us(uint16_t us)
{
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

#define CAROUSEL_STEP_PERIOD_MS     20U  //多久改变一次脉宽
#define CAROUSEL_STEP_US            10U  //每次改变多少脉宽
#define CAROUSEL_SETTLE_MS          150U //输出到达目标后的稳定时间

#define GRIPPER_STEP_PERIOD_MS      20U
#define GRIPPER_STEP_US             15U
#define GRIPPER_SETTLE_MS           150U

typedef enum
{
    CAROUSEL_STATE_IDLE = 0,
    CAROUSEL_STATE_MOVING,
    CAROUSEL_STATE_SETTLING,
    CAROUSEL_STATE_DONE
} CarouselServoState_t;

typedef enum
{
    GRIPPER_STATE_IDLE = 0,
    GRIPPER_STATE_MOVING,
    GRIPPER_STATE_SETTLING,
    GRIPPER_STATE_DONE
} GripperServoState_t;

static uint16_t carousel_current_us = SERVO_SAFE_US;
static uint16_t carousel_target_us = SERVO_SAFE_US;
static uint32_t carousel_last_step_ms = 0U;
static uint32_t carousel_settle_start_ms = 0U;
static CarouselServoState_t carousel_state =CAROUSEL_STATE_IDLE;

static uint16_t gripper_current_us =SERVO_SAFE_US;
static uint16_t gripper_target_us =SERVO_SAFE_US;
static uint32_t gripper_last_step_ms = 0U;
static uint32_t gripper_settle_start_ms = 0U;
static GripperServoState_t gripper_state =GRIPPER_STATE_IDLE;

void Servo_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_SAFE_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_SAFE_US);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_SAFE_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO_SAFE_US);
	
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	
		carousel_current_us = SERVO_SAFE_US;
		carousel_target_us = SERVO_SAFE_US;
		carousel_last_step_ms = HAL_GetTick();
		carousel_settle_start_ms = 0U;
		carousel_state = CAROUSEL_STATE_IDLE;
	
		gripper_current_us = SERVO_SAFE_US;
		gripper_target_us = SERVO_SAFE_US;
		gripper_last_step_ms = HAL_GetTick();
		gripper_settle_start_ms = 0U;
		gripper_state = GRIPPER_STATE_IDLE;

}

/*Pwm1Servo_SetUs()       → TIM3_CH1 → 板载PWM1
CarouselServo_SetUs()     → TIM3_CH2 → 板载PWM2
TraySelectorServo_SetUs() → TIM3_CH4 → 板载PWM3
GripperServo_SetUs()      → TIM3_CH3 → 板载PWM4*/

void Pwm1Servo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,clamp_us(us));
}

/*夹爪开合*/
void GripperServo_SetUs(uint16_t us)
{
    gripper_target_us = clamp_us(us);
    gripper_last_step_ms = HAL_GetTick();

    if (gripper_current_us ==
        gripper_target_us)
    {
        gripper_settle_start_ms =
            HAL_GetTick();

        gripper_state =
            GRIPPER_STATE_SETTLING;
    }
    else
    {
        gripper_state =
            GRIPPER_STATE_MOVING;
    }
}
void GripperServo_Open_Small(void)  { GripperServo_SetUs(GRIPPER_OPEN_US_Small); }
void GripperServo_Open_Large(void)  { GripperServo_SetUs(GRIPPER_OPEN_US_Large); }
void GripperServo_Close(void) { GripperServo_SetUs(GRIPPER_CLOSE_US); }
void GripperServo_Mid(void)   { GripperServo_SetUs(GRIPPER_MID_US); }

void GripperServo_Update(uint32_t now_ms)
{
    uint16_t difference_us;

    switch (gripper_state)
    {
        case GRIPPER_STATE_IDLE:
        case GRIPPER_STATE_DONE:
            break;

        case GRIPPER_STATE_MOVING:
            if ((now_ms - gripper_last_step_ms) <
                GRIPPER_STEP_PERIOD_MS)
            {
                break;
            }

            gripper_last_step_ms = now_ms;

            if (gripper_current_us <
                gripper_target_us)
            {
                difference_us =
                    (uint16_t)(
                        gripper_target_us -
                        gripper_current_us);

                if (difference_us <=
                    GRIPPER_STEP_US)
                {
                    gripper_current_us =
                        gripper_target_us;
                }
                else
                {
                    gripper_current_us =
                        (uint16_t)(
                            gripper_current_us +
                            GRIPPER_STEP_US);
                }
            }
            else
            {
                difference_us =
                    (uint16_t)(
                        gripper_current_us -
                        gripper_target_us);

                if (difference_us <=
                    GRIPPER_STEP_US)
                {
                    gripper_current_us =
                        gripper_target_us;
                }
                else
                {
                    gripper_current_us =
                        (uint16_t)(
                            gripper_current_us -
                            GRIPPER_STEP_US);
                }
            }

            __HAL_TIM_SET_COMPARE(
                &htim3,
                TIM_CHANNEL_3,
                gripper_current_us);

            if (gripper_current_us ==
                gripper_target_us)
            {
                gripper_settle_start_ms =
                    now_ms;

                gripper_state =
                    GRIPPER_STATE_SETTLING;
            }
            break;

        case GRIPPER_STATE_SETTLING:
            if ((now_ms -
                 gripper_settle_start_ms) >=
                GRIPPER_SETTLE_MS)
            {
                gripper_state =
                    GRIPPER_STATE_DONE;
            }
            break;

        default:
            gripper_state =
                GRIPPER_STATE_IDLE;
            break;
    }
}

bool GripperServo_IsBusy(void)
{
    return
        (gripper_state ==
         GRIPPER_STATE_MOVING) ||
        (gripper_state ==
         GRIPPER_STATE_SETTLING);
}

bool GripperServo_IsDone(void)
{
    return
        (gripper_state ==
         GRIPPER_STATE_IDLE) ||
        (gripper_state ==
         GRIPPER_STATE_DONE);
}

/*物料选择*/
void TraySelectorServo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, clamp_us(us));
}

void TraySelectorServo_GotoSlot(uint8_t slot)
{
    static const uint16_t slot_us[TRAY_SELECTOR_SLOT_COUNT] =
    {
        TRAY_SELECTOR_SLOT0_US,
        TRAY_SELECTOR_SLOT1_US,
        TRAY_SELECTOR_SLOT2_US
    };

    if (slot >= TRAY_SELECTOR_SLOT_COUNT)
    {
        return;
    }

    TraySelectorServo_SetUs(slot_us[slot]);
}

/*夹爪旋转*/
void CarouselServo_SetUs(uint16_t us)
{
    carousel_target_us = clamp_us(us);
    carousel_last_step_ms = HAL_GetTick();

    if (carousel_current_us == carousel_target_us)
    {
        carousel_settle_start_ms = HAL_GetTick();
        carousel_state = CAROUSEL_STATE_SETTLING;
    }
    else
    {
        carousel_state = CAROUSEL_STATE_MOVING;
    }
}

void CarouselServo_Update(uint32_t now_ms)
{
    uint16_t difference_us;

    switch (carousel_state)
    {
        case CAROUSEL_STATE_IDLE:
        case CAROUSEL_STATE_DONE:
            break;

        case CAROUSEL_STATE_MOVING:
            if ((now_ms - carousel_last_step_ms)< CAROUSEL_STEP_PERIOD_MS)
            {
                break;
            }

            carousel_last_step_ms = now_ms;

            if (carousel_current_us < carousel_target_us)
            {
                difference_us =
                    (uint16_t)(carousel_target_us
                    - carousel_current_us);

                if (difference_us <= CAROUSEL_STEP_US)
                {
                    carousel_current_us = carousel_target_us;
                }
                else
                {
                    carousel_current_us = 
									(uint16_t)(carousel_current_us+ CAROUSEL_STEP_US);
                }
            }
            else
            {
                difference_us =
                    (uint16_t)(carousel_current_us
                    - carousel_target_us);

                if (difference_us <= CAROUSEL_STEP_US)
                {
                    carousel_current_us =
                        carousel_target_us;
                }
                else
                {
                    carousel_current_us =
                        (uint16_t)(carousel_current_us
                        - CAROUSEL_STEP_US);
                }
            }

            __HAL_TIM_SET_COMPARE(
                &htim3,
                TIM_CHANNEL_2,
                carousel_current_us);

            if (carousel_current_us == carousel_target_us)
            {
                carousel_settle_start_ms = now_ms;
                carousel_state =
                    CAROUSEL_STATE_SETTLING;
            }
            break;

        case CAROUSEL_STATE_SETTLING:
            if ((now_ms - carousel_settle_start_ms)
                >= CAROUSEL_SETTLE_MS)
            {
                carousel_state = CAROUSEL_STATE_DONE;
            }
            break;

        default:
            carousel_state = CAROUSEL_STATE_IDLE;
            break;
    }
}

bool CarouselServo_IsBusy(void)
{
    return (carousel_state == CAROUSEL_STATE_MOVING) ||
           (carousel_state == CAROUSEL_STATE_SETTLING);
}

bool CarouselServo_IsDone(void)
{
    return (carousel_state == CAROUSEL_STATE_IDLE) ||
           (carousel_state == CAROUSEL_STATE_DONE);
}


//三个固定位置(夹取位、放置位、回中位）
void CarouselServo_ToPickupPos1(void)
{
    CarouselServo_SetUs(CAROUSEL_PICKUP_POS1_US);
}

void CarouselServo_ToPickup(void)
{
    CarouselServo_SetUs(CAROUSEL_PICKUP_POS2_US);
}

void CarouselServo_ToPickupPos3(void)
{
    CarouselServo_SetUs(CAROUSEL_PICKUP_POS3_US);
}

void CarouselServo_ToPlace(void)
{
    CarouselServo_SetUs(CAROUSEL_PLACE_US);
}

void CarouselServo_ToMid(void)
{
    CarouselServo_SetUs(CAROUSEL_MID_US);
}

