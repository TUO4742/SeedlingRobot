#include "arm_control.h"

#include "ps2_usart.h"
#include "robot_config.h"
#include "servo.h"

typedef struct
{
    uint16_t angle[ARM_SERVO_COUNT];
} ArmPose;

static const ArmPose s_presets[ARM_PRESET_COUNT] =
{
    {{ARM_POSE_L1_SERVO1_DEG, ARM_POSE_L1_SERVO2_DEG, ARM_POSE_L1_SERVO3_DEG, ARM_POSE_L1_SERVO4_DEG}},
    {{ARM_POSE_L2_SERVO1_DEG, ARM_POSE_L2_SERVO2_DEG, ARM_POSE_L2_SERVO3_DEG, ARM_POSE_L2_SERVO4_DEG}},
    {{ARM_POSE_R1_SERVO1_DEG, ARM_POSE_R1_SERVO2_DEG, ARM_POSE_R1_SERVO3_DEG, ARM_POSE_R1_SERVO4_DEG}},
    {{ARM_POSE_R2_SERVO1_DEG, ARM_POSE_R2_SERVO2_DEG, ARM_POSE_R2_SERVO3_DEG, ARM_POSE_R2_SERVO4_DEG}}
};

static uint16_t s_current_angle[ARM_SERVO_COUNT];
static uint16_t s_target_angle[ARM_SERVO_COUNT];
static uint8_t s_select_was_pressed;
static uint8_t s_select_ready;
static uint8_t s_previous_dpad_buttons;

static uint16_t clamp_joint_angle(uint8_t servo, uint16_t angle)
{
    static const uint16_t min_angle[ARM_SERVO_COUNT] =
    {
        ARM_SERVO1_MIN_DEG, ARM_SERVO2_MIN_DEG, ARM_SERVO3_MIN_DEG,
        ARM_SERVO4_MIN_DEG
    };
    static const uint16_t max_angle[ARM_SERVO_COUNT] =
    {
        ARM_SERVO1_MAX_DEG, ARM_SERVO2_MAX_DEG, ARM_SERVO3_MAX_DEG,
        ARM_SERVO4_MAX_DEG
    };

    if (angle < min_angle[servo])
    {
        return min_angle[servo];
    }
    if (angle > max_angle[servo])
    {
        return max_angle[servo];
    }
    return angle;
}

void arm_control_init(void)
{
    static const uint16_t startup[ARM_SERVO_COUNT] =
    {
        ARM_START_SERVO1_DEG, ARM_START_SERVO2_DEG, ARM_START_SERVO3_DEG,
        ARM_START_SERVO4_DEG
    };
    uint8_t i;

    servo_init();
    arm_control_reset_input();

    for (i = 0U; i < ARM_SERVO_COUNT; ++i)
    {
        s_current_angle[i] = clamp_joint_angle(i, startup[i]);
        s_target_angle[i] = s_current_angle[i];
        servo_set_angle(i, s_current_angle[i]);
    }
}

void arm_control_select_preset(ArmPreset preset)
{
    uint8_t i;

    if ((uint32_t)preset >= (uint32_t)ARM_PRESET_COUNT)
    {
        return;
    }

    for (i = 0U; i < ARM_SERVO_COUNT; ++i)
    {
        s_target_angle[i] = clamp_joint_angle(i, s_presets[preset].angle[i]);
    }
}

void arm_control_hold(void)
{
    uint8_t i;

    for (i = 0U; i < ARM_SERVO_COUNT; ++i)
    {
        s_target_angle[i] = s_current_angle[i];
    }
}

static void adjust_target_angle(uint8_t servo, int16_t delta)
{
    int32_t candidate;

    if (servo >= ARM_SERVO_COUNT)
    {
        return;
    }

    /* Manual jogging cancels any distant preset and advances from here. */
    candidate = (int32_t)s_current_angle[servo] + (int32_t)delta;
    if (candidate < 0)
    {
        candidate = 0;
    }
    else if (candidate > 180)
    {
        candidate = 180;
    }

    s_target_angle[servo] = clamp_joint_angle(servo, (uint16_t)candidate);
}

static int16_t clockwise_delta(int8_t direction, int8_t clockwise_sign)
{
    int16_t delta = (int16_t)ARM_MANUAL_JOG_STEP_DEG;

    if (clockwise_sign < 0)
    {
        delta = (int16_t)-delta;
    }
    if (direction < 0)
    {
        delta = (int16_t)-delta;
    }
    return delta;
}

void arm_control_jog_big_arm(int8_t direction)
{
    if (direction != 0)
    {
        adjust_target_angle(
            0U,
            clockwise_delta(direction, (int8_t)ARM_BIG_ARM_CW_SIGN));
    }
}

void arm_control_jog_small_arm(int8_t direction)
{
    if (direction != 0)
    {
        adjust_target_angle(
            1U,
            clockwise_delta(direction, (int8_t)ARM_SMALL_ARM_CW_SIGN));
    }
}

void arm_control_set_gripper(uint8_t open)
{
    s_target_angle[2U] = clamp_joint_angle(
        2U,
        (open != 0U) ? ARM_GRIPPER_OPEN_DEG : ARM_GRIPPER_CLOSE_DEG);
}

static void arm_control_jog_gripper(uint8_t servo, int8_t direction)
{
    if (direction != 0)
    {
        adjust_target_angle(
            servo,
            (direction > 0) ? (int16_t)ARM_GRIPPER_JOG_STEP_DEG :
                              (int16_t)-ARM_GRIPPER_JOG_STEP_DEG);
    }
}

void arm_control_reset_input(void)
{
    s_select_was_pressed = 0U;
    s_select_ready = 0U;
    s_previous_dpad_buttons = 0U;
}

static void handle_dpad_preset(uint8_t dpad_buttons)
{
    switch (dpad_buttons)
    {
        case PS2_DPAD_UP:    arm_control_select_preset(ARM_PRESET_L1); break;
        case PS2_DPAD_DOWN:  arm_control_select_preset(ARM_PRESET_L2); break;
        case PS2_DPAD_LEFT:  arm_control_select_preset(ARM_PRESET_R1); break;
        case PS2_DPAD_RIGHT: arm_control_select_preset(ARM_PRESET_R2); break;
        default: break;
    }
}

void arm_control_handle_input(
    uint8_t select,
    uint8_t shoulder_buttons,
    uint8_t dpad_buttons,
    uint8_t right_key,
    uint8_t controls_neutral,
    uint8_t new_frame)
{
    uint8_t big_cw;
    uint8_t big_ccw;
    uint8_t small_cw;
    uint8_t small_ccw;
    uint8_t active_categories;

    if (select == 0U)
    {
        if (new_frame != 0U)
        {
            arm_control_reset_input();
        }
        return;
    }

    if (s_select_was_pressed == 0U)
    {
        if (new_frame == 0U)
        {
            return;
        }
        s_select_ready = 0U;
        s_previous_dpad_buttons = 0U;
    }
    s_select_was_pressed = 1U;

    if ((controls_neutral != 0U) && (new_frame != 0U))
    {
        /* Require one fresh neutral SELECT frame before accepting arm chords. */
        s_select_ready = 1U;
    }

    if (s_select_ready == 0U)
    {
        if (new_frame != 0U)
        {
            s_previous_dpad_buttons = dpad_buttons;
        }
        return;
    }

    active_categories = 0U;
    if (shoulder_buttons != 0U)
    {
        ++active_categories;
    }
    if (dpad_buttons != 0U)
    {
        ++active_categories;
    }
    if (right_key != 0U)
    {
        ++active_categories;
    }

    if (active_categories != 1U)
    {
        if (new_frame != 0U)
        {
            s_previous_dpad_buttons = dpad_buttons;
        }
        return;
    }

    big_cw = ((shoulder_buttons & KEY_L1) != 0U) ? 1U : 0U;
    big_ccw = ((shoulder_buttons & KEY_R1) != 0U) ? 1U : 0U;
    small_cw = ((shoulder_buttons & KEY_L2) != 0U) ? 1U : 0U;
    small_ccw = ((shoulder_buttons & KEY_R2) != 0U) ? 1U : 0U;

    if ((new_frame != 0U) &&
        (shoulder_buttons != 0U) &&
        (big_cw != 0U) && (big_ccw == 0U))
    {
        arm_control_jog_big_arm(1);
    }
    else if ((new_frame != 0U) &&
             (big_ccw != 0U) && (big_cw == 0U))
    {
        arm_control_jog_big_arm(-1);
    }
    if ((new_frame != 0U) &&
        (shoulder_buttons != 0U) &&
        (small_cw != 0U) && (small_ccw == 0U))
    {
        arm_control_jog_small_arm(1);
    }
    else if ((new_frame != 0U) &&
             (small_ccw != 0U) && (small_cw == 0U))
    {
        arm_control_jog_small_arm(-1);
    }

    if ((dpad_buttons != 0U) && (new_frame != 0U))
    {
        uint8_t exactly_one =
            ((dpad_buttons & (uint8_t)(dpad_buttons - 1U)) == 0U) ? 1U : 0U;
        if ((exactly_one != 0U) &&
            (dpad_buttons != s_previous_dpad_buttons))
        {
            handle_dpad_preset(dpad_buttons);
        }
    }

    /* Hold SELECT+X/B to continuously open/close Servo 3.
     * Hold SELECT+Y/A to continuously open/close Servo 4. */
    if ((right_key != 0U) && (new_frame != 0U))
    {
        if (right_key == 3U)
        {
            arm_control_jog_gripper(2U, -1);
        }
        else if (right_key == 4U)
        {
            arm_control_jog_gripper(2U, 1);
        }
        else if (right_key == 1U)
        {
            arm_control_jog_gripper(3U, -1);
        }
        else if (right_key == 2U)
        {
            arm_control_jog_gripper(3U, 1);
        }
    }

    if (new_frame != 0U)
    {
        s_previous_dpad_buttons = dpad_buttons;
    }
}

void arm_control_update(void)
{
    uint8_t i;

    for (i = 0U; i < ARM_SERVO_COUNT; ++i)
    {
        if (s_current_angle[i] < s_target_angle[i])
        {
            uint16_t remaining = (uint16_t)(s_target_angle[i] - s_current_angle[i]);
            s_current_angle[i] += (remaining > ARM_MOVE_STEP_DEG) ? ARM_MOVE_STEP_DEG : remaining;
        }
        else if (s_current_angle[i] > s_target_angle[i])
        {
            uint16_t remaining = (uint16_t)(s_current_angle[i] - s_target_angle[i]);
            s_current_angle[i] -= (remaining > ARM_MOVE_STEP_DEG) ? ARM_MOVE_STEP_DEG : remaining;
        }

        servo_set_angle(i, s_current_angle[i]);
    }
}
