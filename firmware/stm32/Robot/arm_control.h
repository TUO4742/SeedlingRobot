#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include <stdint.h>

typedef enum
{
    ARM_PRESET_L1 = 0,
    ARM_PRESET_L2,
    ARM_PRESET_R1,
    ARM_PRESET_R2,
    ARM_PRESET_COUNT
} ArmPreset;

void arm_control_init(void);
void arm_control_update(void);
void arm_control_select_preset(ArmPreset preset);
/* direction: +1 clockwise, -1 counter-clockwise. */
void arm_control_jog_big_arm(int8_t direction);
void arm_control_jog_small_arm(int8_t direction);
void arm_control_set_gripper(uint8_t open);
void arm_control_handle_input(
    uint8_t select,
    uint8_t shoulder_buttons,
    uint8_t dpad_buttons,
    uint8_t right_key,
    uint8_t controls_neutral,
    uint8_t new_frame);
void arm_control_reset_input(void);
void arm_control_hold(void);

#endif
