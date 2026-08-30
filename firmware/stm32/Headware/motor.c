#include "motor.h"
#include "robot_config.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdlib.h>

typedef struct
{
    GPIO_TypeDef *dir_port_a;
    uint16_t      dir_pin_a;
    GPIO_TypeDef *dir_port_b;
    uint16_t      dir_pin_b;
    TIM_HandleTypeDef *htim;
    uint32_t      channel;
} MotorHW;

static const MotorHW s_motor_hw[4] =
{
    /* Motor 0 (FL, was RL): PB12/PB13, TIM2_CH4 */
    { GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &htim2, TIM_CHANNEL_4 },
    /* Motor 1 (RR, was FR): PC14/PC13, TIM2_CH1 */
    { GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_13, &htim2, TIM_CHANNEL_1 },
    /* Motor 2 (RL, was RR): PC8/PC9, TIM2_CH3 */
    { GPIOC, GPIO_PIN_8,  GPIOC, GPIO_PIN_9,  &htim2, TIM_CHANNEL_3 },
    /* Motor 3 (FR, was FL): PC1/PC0, TIM2_CH2 */
    { GPIOC, GPIO_PIN_1,  GPIOC, GPIO_PIN_0,  &htim2, TIM_CHANNEL_2 },
};

static const int8_t s_motor_output_dir[4] =
{
    MOTOR_OUTPUT_DIR_0, MOTOR_OUTPUT_DIR_1,
    MOTOR_OUTPUT_DIR_2, MOTOR_OUTPUT_DIR_3
};

static void motor_short_brake(uint8_t index)
{
    uint32_t full_pwm = __HAL_TIM_GET_AUTORELOAD(s_motor_hw[index].htim) + 1U;

    /* TB6612 short brake: STBY=H, IN1=H, IN2=H and PWM=H. */
    HAL_GPIO_WritePin(
        s_motor_hw[index].dir_port_a,
        s_motor_hw[index].dir_pin_a,
        GPIO_PIN_SET);
    HAL_GPIO_WritePin(
        s_motor_hw[index].dir_port_b,
        s_motor_hw[index].dir_pin_b,
        GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(
        s_motor_hw[index].htim,
        s_motor_hw[index].channel,
        full_pwm);
}

void motor_init(void)
{
    if ((HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK))
    {
        Error_Handler();
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

void motor_ctrol(Chassis_TypeDef *chassis)
{
    uint8_t i;
    uint8_t all_stopped = 1U;

    for (i = 0U; i < 4U; ++i)
    {
        if (abs(chassis->motor[i].speed) > MOTOR_STOP_THRESHOLD)
        {
            all_stopped = 0U;
            break;
        }
    }

    for (i = 0U; i < 4U; ++i)
    {
        int16_t speed   = (int16_t)(chassis->motor[i].speed * s_motor_output_dir[i]);
        int16_t mag     = abs(speed);
        uint8_t stopped = (all_stopped != 0U) || (mag == 0);

        if (stopped)
        {
            motor_short_brake(i);
        }
        else if (speed > 0)
        {
            HAL_GPIO_WritePin(s_motor_hw[i].dir_port_a, s_motor_hw[i].dir_pin_a, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(s_motor_hw[i].dir_port_b, s_motor_hw[i].dir_pin_b, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(s_motor_hw[i].htim, s_motor_hw[i].channel, (uint32_t)mag);
        }
        else /* speed < 0 */
        {
            HAL_GPIO_WritePin(s_motor_hw[i].dir_port_a, s_motor_hw[i].dir_pin_a, GPIO_PIN_SET);
            HAL_GPIO_WritePin(s_motor_hw[i].dir_port_b, s_motor_hw[i].dir_pin_b, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(s_motor_hw[i].htim, s_motor_hw[i].channel, (uint32_t)mag);
        }
    }
}
