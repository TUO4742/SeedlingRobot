#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h"
#include "chassis_mode.h"
// ==================== 电机 / 轮子 / 测速 参数 ====================
#define ENC_LINE            11.0f     // 电机转子编码器线数
#define ENC_MULTIPLE        4.0f      // 4倍频
#define GEAR_RATIO          30.0f     // 520 电机 1:30 减速比（你是330转，就是1:30）
#define WHEEL_DIAMETER      0.065f     // 轮子直径 0.08m 
#define CAPTURE_PERIOD      0.02f     // 20ms
#define PI                  3.1415926f

void motor_init(void);
void motor_ctrol(Chassis_TypeDef* chassis);
#endif
