#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"

/* Encoder-to-speed feedback clamp (m/s), same unit as PID. */
#define ENCODER_ACTUAL_SPEED_LIMIT_MPS  (1.28f)

extern volatile int32_t s_raw_encoder_count[4];
extern volatile uint8_t s_encoder_ready;

int read_encoder_count(TIM_HandleTypeDef *htim);
void encoder_start(void);
void encoder_process_and_control(void);

#endif
