#include "encoder.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "chassis_mode.h"
#include "motor.h"
#include "pid.h"
#include "robot_config.h"
#include "servo.h"

volatile int32_t s_raw_encoder_count[4] = {0, 0, 0, 0};
volatile uint8_t s_encoder_ready = 0U;

static const int8_t s_encoder_dir[4] =
{
    MOTOR_ENCODER_DIR_0, MOTOR_ENCODER_DIR_1,
    MOTOR_ENCODER_DIR_2, MOTOR_ENCODER_DIR_3
};

static float filter_encoder_speed(uint8_t index, float speed)
{
    static float filtered_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float alpha = 0.2f;

    filtered_speed[index] += alpha * (speed - filtered_speed[index]);
    return filtered_speed[index];
}

void encoder_start(void)
{
    if ((HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) ||
        (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) ||
        (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) ||
        (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) ||
        (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK))
    {
        Error_Handler();
    }
}

int read_encoder_count(TIM_HandleTypeDef *htim)
{
    int temp = (short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim, 0);
    return temp;
}

static float encoder_calc_mps(float pulse)
{
    return (pulse * PI * WHEEL_DIAMETER) / (ENC_LINE * ENC_MULTIPLE * GEAR_RATIO * CAPTURE_PERIOD);
}

static float clamp_float(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void encoder_process_and_control(void)
{
    int32_t raw[4];
    uint8_t i;

    if (s_encoder_ready == 0U)
    {
        return;
    }
    s_encoder_ready = 0U;

    /* Copy raw counts atomically (ISR no longer touches this copy). */
    for (i = 0U; i < 4U; ++i)
    {
        raw[i] = s_raw_encoder_count[i];
    }

    for (i = 0U; i < 4U; ++i)
    {
        float speed_mps;

        speed_mps = (float)raw[i] * (float)s_encoder_dir[i];
        speed_mps = encoder_calc_mps(speed_mps);
        speed_mps = clamp_float(speed_mps, -ENCODER_ACTUAL_SPEED_LIMIT_MPS, ENCODER_ACTUAL_SPEED_LIMIT_MPS);
        speed_mps = filter_encoder_speed(i, speed_mps);

        chassis.motor1_pid[i].actual = speed_mps;
        chassis.motor[i].speed = (int16_t)pid_calc_inc(&chassis.motor1_pid[i]);
    }

    /* Physical wheel order after rewiring: motor 0=FL, 1=RR, 2=RL, 3=FR. */
    {
        float s0 = chassis.motor1_pid[0].actual;
        float s1 = chassis.motor1_pid[1].actual;
        float s2 = chassis.motor1_pid[2].actual;
        float s3 = chassis.motor1_pid[3].actual;

        chassis.cur_vx = (s0 + s3 + s2 + s1) * 0.25f;
        chassis.cur_vy = (-s0 + s3 + s2 - s1) * 0.25f;
        chassis.cur_vz = (-s0 + s3 - s2 + s1) / (4.0f * CHASSIS_WHEEL_BASE_SUM_M);
    }

    motor_ctrol(&chassis);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        servo_timer_period_elapsed();
    }
    else if (htim->Instance == TIM6)
    {
        /* ISR only captures raw counts; processing is done in task context. */
        s_raw_encoder_count[0] = -read_encoder_count(&htim8);
        s_raw_encoder_count[1] =  read_encoder_count(&htim3);
        s_raw_encoder_count[2] =  read_encoder_count(&htim1);
        s_raw_encoder_count[3] = -read_encoder_count(&htim4);

        s_encoder_ready = 1U;
    }
}
