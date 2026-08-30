#include "servo.h"

#include "robot_config.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

static volatile uint16_t s_servo3_pulse_us = 1500U;
static uint16_t s_servo3_active_pulse_us = 1500U;
static volatile uint16_t s_servo4_pulse_us = 1500U;
static uint16_t s_servo4_active_pulse_us = 1500U;

static uint16_t angle_to_pulse(uint8_t servo, uint16_t angle_deg)
{
    static const uint8_t reversed[ARM_SERVO_COUNT] =
    {
        ARM_SERVO1_REVERSED, ARM_SERVO2_REVERSED, ARM_SERVO3_REVERSED,
        ARM_SERVO4_REVERSED
    };
    uint32_t span;

    if (angle_deg > 180U)
    {
        angle_deg = 180U;
    }
    if (reversed[servo] != 0U)
    {
        angle_deg = (uint16_t)(180U - angle_deg);
    }

    span = ARM_SERVO_MAX_PULSE_US - ARM_SERVO_MIN_PULSE_US;
    return (uint16_t)(ARM_SERVO_MIN_PULSE_US + ((uint32_t)angle_deg * span) / 180U);
}

void servo_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint16_t servo1_start_pulse;
    uint16_t servo2_start_pulse;
    uint16_t servo3_start_pulse;
    uint16_t servo4_start_pulse;

    servo1_start_pulse = angle_to_pulse(SERVO_JOINT_1, ARM_START_SERVO1_DEG);
    servo2_start_pulse = angle_to_pulse(SERVO_JOINT_2, ARM_START_SERVO2_DEG);
    servo3_start_pulse = angle_to_pulse(SERVO_JOINT_3, ARM_START_SERVO3_DEG);
    servo4_start_pulse = angle_to_pulse(SERVO_JOINT_4, ARM_START_SERVO4_DEG);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Start directly at the configured pose; do not pulse 90 degrees first. */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, servo1_start_pulse);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, servo2_start_pulse);
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* CH3 is internal timing only; PA2 remains available to USART2. */
    s_servo3_pulse_us = servo3_start_pulse;
    s_servo3_active_pulse_us = servo3_start_pulse;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, s_servo3_active_pulse_us);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    /*
     * Do not call HAL_TIM_OC_Start_IT(CH3): it sets CC3E and would also enable
     * TIM5_CH3 on PA2, which is USART2_TX on STM32F1. A compare interrupt does
     * not require the channel output gate.
     */
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC3);
    CLEAR_BIT(htim5.Instance->CCER, TIM_CCER_CC3E);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC3);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC3);

    /* CH4 is internal timing only; PA3 remains available to USART2. */
    s_servo4_pulse_us = servo4_start_pulse;
    s_servo4_active_pulse_us = servo4_start_pulse;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, s_servo4_active_pulse_us);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC4);
    CLEAR_BIT(htim5.Instance->CCER, TIM_CCER_CC4E);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC4);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC4);
}

void servo_set_angle(uint8_t servo, uint16_t angle_deg)
{
    uint16_t pulse;

    if (servo >= ARM_SERVO_COUNT)
    {
        return;
    }

    pulse = angle_to_pulse(servo, angle_deg);
    if (servo == SERVO_JOINT_1)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
    }
    else if (servo == SERVO_JOINT_2)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
    }
    else if (servo == SERVO_JOINT_3)
    {
        /* s_servo3_pulse_us is read by TIM5 period-elapsed ISR;
         * disable interrupts briefly so the ISR never sees a torn value. */
        __disable_irq();
        s_servo3_pulse_us = pulse;
        __enable_irq();
    }
    else
    {
        __disable_irq();
        s_servo4_pulse_us = pulse;
        __enable_irq();
    }
}

void servo_timer_period_elapsed(void)
{
    uint32_t counter;

    /* Latch the requested pulse at frame start so mid-frame updates are safe. */
    s_servo3_active_pulse_us = s_servo3_pulse_us;
    s_servo4_active_pulse_us = s_servo4_pulse_us;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, s_servo3_active_pulse_us);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, s_servo4_active_pulse_us);
    counter = __HAL_TIM_GET_COUNTER(&htim5);

    /*
     * HAL services CC3 before UPDATE. If both flags were delayed, never raise
     * PC5 after this frame's compare point or it could stay high for 20 ms.
     */
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_5,
        (counter < s_servo3_active_pulse_us) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        (counter < s_servo4_active_pulse_us) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void servo_timer_compare_elapsed(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM5) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
    {
        servo_timer_compare_elapsed();
    }
    else if ((htim->Instance == TIM5) &&
             (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
}
