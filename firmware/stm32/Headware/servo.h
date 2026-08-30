#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

enum
{
    SERVO_JOINT_1 = 0,
    SERVO_JOINT_2,
    SERVO_JOINT_3,
    SERVO_JOINT_4
};

void servo_init(void);
void servo_set_angle(uint8_t servo, uint16_t angle_deg);
void servo_timer_period_elapsed(void);
void servo_timer_compare_elapsed(void);

#endif
