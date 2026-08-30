#include "chassis_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "arm_control.h"
#include "chassis_mode.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "ps2_usart.h"
#include "robot_config.h"

#include <string.h>

Chassis_TypeDef chassis;
volatile uint8_t ps2_active = 0U;
static uint8_t s_drive_rearm_required = 1U;

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float normalized_axis(int16_t value)
{
    int32_t magnitude = (value < 0) ? -(int32_t)value : (int32_t)value;
    float normalized;

    if (magnitude <= PS2_AXIS_DEADZONE)
    {
        return 0.0f;
    }

    if (magnitude > 128)
    {
        magnitude = 128;
    }

    normalized = (float)(magnitude - PS2_AXIS_DEADZONE) /
                 (float)(128 - PS2_AXIS_DEADZONE);
    return (value < 0) ? -normalized : normalized;
}

static void motor_pid_init(Chassis_TypeDef *state)
{
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        pid_init(
            &state->motor1_pid[i],
            MOTOR_PID_KP,
            MOTOR_PID_KI,
            MOTOR_PID_KD,
            MOTOR_PID_MAX_OUT);
    }
}

static void chassis_init(void)
{
    memset(&chassis, 0, sizeof(chassis));
    usart_rx_init();
    motor_init();
    motor_pid_init(&chassis);
    arm_control_init();
    /* Start the 20 ms encoder/PID interrupt only after every target is valid. */
    encoder_start();
}

void chassis_force_stop(void)
{
    uint8_t i;

    chassis.vx = 0.0f;
    chassis.vy = 0.0f;
    chassis.vz = 0.0f;
    for (i = 0U; i < 4U; ++i)
    {
        chassis.motor1_pid[i].target = 0.0f;
        chassis.motor1_pid[i].out = 0.0f;
        chassis.motor1_pid[i].err = 0.0f;
        chassis.motor1_pid[i].last_err = 0.0f;
        chassis.motor1_pid[i].prev_err = 0.0f;
        chassis.motor1_pid[i].integral = 0.0f;
        chassis.motor[i].speed = 0;
    }
    /* Write the zero-speed TB6612 short-brake state immediately. */
    motor_ctrol(&chassis);
}

static void apply_ps2_command(const PS2_HandleTypeDef *controller)
{
    if ((controller->frame_id != 0x01U) || (controller->select != 0U))
    {
        chassis_force_stop();
        return;
    }

    /*
     * Left stick is a continuous 2-D translation command:
     *   up/left     -> forward/left,
     *   down/right  -> reverse/right.
     * Keep yaw on the CH559 right-control left/right codes.
     */
    chassis.vx = normalized_axis(controller->ly) *
                 PS2_LEFT_Y_TO_FORWARD_SIGN * CHASSIS_MAX_VX_MPS;
    chassis.vy = normalized_axis(controller->lx) *
                 PS2_LEFT_X_TO_LEFT_SIGN * CHASSIS_MAX_VY_MPS;

    /* Exact discrete right-control mapping from the differential project. */
    if (controller->right_key == 3U)
    {
        chassis.vz = PS2_RIGHT_CONTROL_TO_CCW_SIGN * CHASSIS_MAX_WZ_RADPS;
    }
    else if (controller->right_key == 4U)
    {
        chassis.vz = -PS2_RIGHT_CONTROL_TO_CCW_SIGN * CHASSIS_MAX_WZ_RADPS;
    }
    else
    {
        /* Y/A/neutral must not retain an earlier yaw command. */
        chassis.vz = 0.0f;
    }
}

static uint8_t drive_inputs_are_neutral(const PS2_HandleTypeDef *controller)
{
    int32_t lx = (int32_t)controller->lx;
    int32_t ly = (int32_t)controller->ly;

    if (lx < 0)
    {
        lx = -lx;
    }
    if (ly < 0)
    {
        ly = -ly;
    }

    return ((lx <= PS2_AXIS_DEADZONE) &&
            (ly <= PS2_AXIS_DEADZONE) &&
            (controller->dpad_buttons == 0U) &&
            (controller->right_key == 0U) &&
            (controller->shoulder_buttons == 0U)) ? 1U : 0U;
}

static void set_wheel_targets(void)
{
    float wheel[4];
    float wz_term;
    float maximum;
    uint8_t i;

    if ((abs_float(chassis.vx) < 0.0001f) &&
        (abs_float(chassis.vy) < 0.0001f) &&
        (abs_float(chassis.vz) < 0.0001f))
    {
        chassis_force_stop();
        return;
    }

    wz_term = CHASSIS_WHEEL_BASE_SUM_M * chassis.vz;

    /*
     * X-roller mecanum inverse kinematics. Physical wheel remap:
     * motor 0=FL, 1=RR, 2=RL, 3=FR.
     */
    wheel[0] = chassis.vx - chassis.vy - wz_term; /* FL */
    wheel[3] = chassis.vx + chassis.vy + wz_term; /* FR */
    wheel[2] = chassis.vx + chassis.vy - wz_term; /* RL */
    wheel[1] = chassis.vx - chassis.vy + wz_term; /* RR */

    maximum = abs_float(wheel[0]);
    for (i = 1U; i < 4U; ++i)
    {
        float candidate = abs_float(wheel[i]);
        if (candidate > maximum)
        {
            maximum = candidate;
        }
    }

    if (maximum > CHASSIS_MAX_WHEEL_SPEED_MPS)
    {
        float scale = CHASSIS_MAX_WHEEL_SPEED_MPS / maximum;
        for (i = 0U; i < 4U; ++i)
        {
            wheel[i] *= scale;
        }
    }

    for (i = 0U; i < 4U; ++i)
    {
        float delta = wheel[i] - chassis.motor1_pid[i].target;
        if (delta > CHASSIS_TARGET_STEP_MPS)
        {
            delta = CHASSIS_TARGET_STEP_MPS;
        }
        else if (delta < -CHASSIS_TARGET_STEP_MPS)
        {
            delta = -CHASSIS_TARGET_STEP_MPS;
        }
        chassis.motor1_pid[i].target += delta;
    }
}

void chassis_task(void const *argument)
{
    TickType_t last_wake;
    (void)argument;

    chassis_init();
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint8_t new_ps2_frame = 0U;
        uint8_t inputs_neutral;
        uint32_t now_ms;

        if (uart5_rx_finish != 0U)
        {
            data_flag = 0U;
            ps2_parse_data(uart5_rx_buf);
            uart5_rx_finish = 0U;

            if ((data_flag != 0U) &&
                ((uint32_t)(HAL_GetTick() - ps2_last_frame_tick) <=
                 PS2_LINK_TIMEOUT_MS))
            {
                new_ps2_frame = 1U;
                data_flag = 0U;
            }
        }

        /* Read the tick after parsing; the parser records its own last-good tick. */
        now_ms = HAL_GetTick();
        ps2_update_timeout(now_ms, PS2_LINK_TIMEOUT_MS);
        if (remote_stopped_flag != 0U)
        {
            ps2_active = 0U;
            chassis_force_stop();
            s_drive_rearm_required = 1U;
            arm_control_reset_input();
            arm_control_hold();
        }
        else
        {
            ps2_active = 1U;
            inputs_neutral = drive_inputs_are_neutral(&ps2);
            arm_control_handle_input(
                ps2.select,
                ps2.shoulder_buttons,
                ps2.dpad_buttons,
                ps2.right_key,
                inputs_neutral,
                new_ps2_frame);

            if (ps2.select != 0U)
            {
                chassis_force_stop();
                s_drive_rearm_required = 1U;
            }
            else if (s_drive_rearm_required != 0U)
            {
                chassis_force_stop();
                if ((new_ps2_frame != 0U) && (inputs_neutral != 0U))
                {
                    /* Stay stopped for this centered frame; move on the next. */
                    s_drive_rearm_required = 0U;
                }
            }
            else
            {
                apply_ps2_command(&ps2);
            }
        }

        set_wheel_targets();
        encoder_process_and_control();
        arm_control_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONTROL_PERIOD_MS));
    }
}
