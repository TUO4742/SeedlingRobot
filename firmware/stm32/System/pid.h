#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* Velocity deadband: when |target| is below this threshold (m/s),
 * the incremental PID resets its output, integral and error history. */
#ifndef PID_VEL_DEADBAND_MPS
#define PID_VEL_DEADBAND_MPS  (0.04f)
#endif

/* Maximum output increment per control cycle (20 ms).
 * Tune with the motor response; too high causes overshoot. */
#ifndef PID_INC_MAX_STEP
#define PID_INC_MAX_STEP       (3.5f)
#endif

// PID �ṹ��
typedef struct {
    float target;   // Ŀ��ֵ
    float actual;   // ��ǰֵ
    float err;      // ��ǰ���
    float last_err; // ��һ�����
    float prev_err; // ���ϴ�������ʽר�ã�

    float Kp;
    float Ki;
    float Kd;

    float out;      // ���
    float max_out;  // ������
    float integral; // �����ۼ�
} pid_typedef;

// ��������
void pid_init(pid_typedef *pid, float kp, float ki, float kd, float max_out);
float pid_calc(pid_typedef *pid);
float pid_calc_inc(pid_typedef *pid);

#endif

