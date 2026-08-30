#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include <stdint.h>

extern volatile uint8_t ps2_active;

void chassis_task(void const *argument);
void chassis_force_stop(void);

#endif
