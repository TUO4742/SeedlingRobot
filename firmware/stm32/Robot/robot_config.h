#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/*
 * RuiKang v1.4 user configuration.
 *
 * Coordinate convention:
 *   +vx: robot forward
 *   +vy: robot left
 *   +wz: counter-clockwise viewed from above
 *
 * The values in this file are intentionally centralized so that joystick
 * direction, speed and arm poses can be calibrated without editing drivers.
 */

/* ---------------- PS2 / chassis ---------------- */
#define ROBOT_CONTROL_PERIOD_MS          (20U)
#define PS2_LINK_TIMEOUT_MS              (300U)
#define PS2_AXIS_DEADZONE                (12)

/* Raw USB-HID axes are centered at 0x80. Up/left are normally negative. */
#define PS2_LEFT_Y_TO_FORWARD_SIGN       (-1.0f)
#define PS2_LEFT_X_TO_LEFT_SIGN          (-1.0f)
/* Set to -1 only if the CH559 X/B yaw direction is reversed on the robot. */
#define PS2_RIGHT_CONTROL_TO_CCW_SIGN    (1.0f)

#define CHASSIS_MAX_VX_MPS               (0.6f)
#define CHASSIS_MAX_VY_MPS               (0.5f)
#define CHASSIS_MAX_WZ_RADPS             (3.50f)

/* L + W for mecanum inverse kinematics, in metres. */
#define CHASSIS_WHEEL_BASE_SUM_M         (0.168f)
#define CHASSIS_MAX_WHEEL_SPEED_MPS      (0.72f)
#define CHASSIS_TARGET_STEP_MPS          (0.020f)

/* Encoder sign for each motor: +1 or -1, depending on physical wiring.
 * Physical wheel index after rewiring: 0=FL, 1=RR, 2=RL, 3=FR.
 * The lifted-wheel test showed RL (2) and FR (3) require reversed signs. */
#define MOTOR_ENCODER_DIR_0             (1)
#define MOTOR_ENCODER_DIR_1             (1)
#define MOTOR_ENCODER_DIR_2             (-1)
#define MOTOR_ENCODER_DIR_3             (-1)

/* Motor output sign for physical wheel index 0=FL, 1=RR, 2=RL, 3=FR. */
#define MOTOR_OUTPUT_DIR_0              (1)
#define MOTOR_OUTPUT_DIR_1              (1)
#define MOTOR_OUTPUT_DIR_2              (-1)
#define MOTOR_OUTPUT_DIR_3              (-1)

/* Minimum motor PWM output to avoid exact-zero comparison noise. */
#define MOTOR_STOP_THRESHOLD            (3)

/* Verified differential-chassis gains; revalidate with the mecanum load. */
#define MOTOR_PID_KP                     (70.0f)
#define MOTOR_PID_KI                     (20.0f)
#define MOTOR_PID_KD                     (0.0f)
#define MOTOR_PID_MAX_OUT                (100.0f)

/* ---------------- Four-servo arm ----------------
 * Board connectors:
 *   Servo 1: J9  / PA0 / TIM5_CH1 hardware PWM
 *   Servo 2: J11 / PA1 / TIM5_CH2 hardware PWM
 *   Servo 3: J12 / PC5 (TIM5_CH3 compare-timed GPIO pulse)
 *   Servo 4: J14 / PB0 (TIM5_CH4 compare-timed GPIO pulse)
 *   J15/PB1 and J16/PB2 are intentionally left unused.
 */
#define ARM_SERVO_COUNT                  (4U)
#define ARM_SERVO_MIN_PULSE_US           (500U)
#define ARM_SERVO_MAX_PULSE_US           (2500U)

/* Set to 1 if a joint rotates opposite to the requested logical angle. */
#define ARM_SERVO1_REVERSED              (0U)
#define ARM_SERVO2_REVERSED              (0U)
#define ARM_SERVO3_REVERSED              (0U)
#define ARM_SERVO4_REVERSED              (0U)

/* Narrow these limits before the first powered mechanical test if needed. */
#define ARM_SERVO1_MIN_DEG               (0U)
#define ARM_SERVO1_MAX_DEG               (180U)
#define ARM_SERVO2_MIN_DEG               (0U)
#define ARM_SERVO2_MAX_DEG               (180U)
#define ARM_SERVO3_MIN_DEG               (0U)
#define ARM_SERVO3_MAX_DEG               (180U)
#define ARM_SERVO4_MIN_DEG               (0U)
#define ARM_SERVO4_MAX_DEG               (180U)

#define ARM_START_SERVO1_DEG             (175U)      //初始化机械臂姿态，180最下面
#define ARM_START_SERVO2_DEG             (35U)     //30最上面
#define ARM_START_SERVO3_DEG             (120U)     //30合住
#define ARM_START_SERVO4_DEG             (120U)     /* Same initial state as Servo 3. */

/* Maximum movement per control period. 2 deg / 20 ms = 100 deg/s. */
#define ARM_MOVE_STEP_DEG                (2U)

/* SELECT+shoulder manual jogging. Flip a sign if physical CW is reversed. */
#define ARM_MANUAL_JOG_STEP_DEG          (2U)
#define ARM_GRIPPER_JOG_STEP_DEG         (2U)
#define ARM_BIG_ARM_CW_SIGN              (1)
#define ARM_SMALL_ARM_CW_SIGN            (1)

/* Legacy fixed Servo 3 targets retained for calibration/reference. */
#define ARM_GRIPPER_OPEN_DEG             (105U)
#define ARM_GRIPPER_CLOSE_DEG            (140U)

/*
 * Four editable three-servo poses. These starting values come from the useful
 * approach/carry/plant/barn postures in the first-generation project. Adjust
 * only this table after confirming each joint's safe direction and limits.
 */
#define ARM_POSE_L1_SERVO1_DEG           (115U)    //上下左右
#define ARM_POSE_L1_SERVO2_DEG           (95U)    //放置幼苗动作
#define ARM_POSE_L1_SERVO3_DEG           (0U)                   
#define ARM_POSE_L1_SERVO4_DEG           (0U)
                                                                 
#define ARM_POSE_L2_SERVO1_DEG           (175U)     //采集幼苗动作
#define ARM_POSE_L2_SERVO2_DEG           (35U)
#define ARM_POSE_L2_SERVO3_DEG           (120U)
#define ARM_POSE_L2_SERVO4_DEG           (120U)

#define ARM_POSE_R1_SERVO1_DEG           (175U)
#define ARM_POSE_R1_SERVO2_DEG           (35U)
#define ARM_POSE_R1_SERVO3_DEG           (0U)
#define ARM_POSE_R1_SERVO4_DEG           (0U)

#define ARM_POSE_R2_SERVO1_DEG           (130U)
#define ARM_POSE_R2_SERVO2_DEG           (90U)
#define ARM_POSE_R2_SERVO3_DEG           (120U)
#define ARM_POSE_R2_SERVO4_DEG           (120U)

#if (ARM_SERVO_MIN_PULSE_US >= ARM_SERVO_MAX_PULSE_US)
#error "Servo minimum pulse must be below the maximum pulse"
#endif

#if (ARM_SERVO_MAX_PULSE_US >= 20000U)
#error "Servo pulse must be shorter than the fixed 20 ms TIM5 frame"
#endif

#if ((ARM_SERVO1_MIN_DEG > ARM_SERVO1_MAX_DEG) || \
     (ARM_SERVO2_MIN_DEG > ARM_SERVO2_MAX_DEG) || \
     (ARM_SERVO3_MIN_DEG > ARM_SERVO3_MAX_DEG) || \
     (ARM_SERVO4_MIN_DEG > ARM_SERVO4_MAX_DEG))
#error "Each servo minimum angle must be below its maximum angle"
#endif

#if ((ARM_SERVO1_MAX_DEG > 180U) || \
     (ARM_SERVO2_MAX_DEG > 180U) || \
     (ARM_SERVO3_MAX_DEG > 180U) || \
     (ARM_SERVO4_MAX_DEG > 180U))
#error "Servo logical angles must stay within 0..180 degrees"
#endif

#if (ARM_MOVE_STEP_DEG == 0U)
#error "ARM_MOVE_STEP_DEG must be non-zero"
#endif

#if (ARM_MANUAL_JOG_STEP_DEG == 0U)
#error "ARM_MANUAL_JOG_STEP_DEG must be non-zero"
#endif

#if (ARM_GRIPPER_JOG_STEP_DEG == 0U)
#error "ARM_GRIPPER_JOG_STEP_DEG must be non-zero"
#endif

#if (((ARM_BIG_ARM_CW_SIGN) != 1) && ((ARM_BIG_ARM_CW_SIGN) != -1))
#error "ARM_BIG_ARM_CW_SIGN must be +1 or -1"
#endif

#if (((ARM_SMALL_ARM_CW_SIGN) != 1) && ((ARM_SMALL_ARM_CW_SIGN) != -1))
#error "ARM_SMALL_ARM_CW_SIGN must be +1 or -1"
#endif

#if ((ARM_GRIPPER_OPEN_DEG < ARM_SERVO3_MIN_DEG) || \
     (ARM_GRIPPER_OPEN_DEG > ARM_SERVO3_MAX_DEG) || \
     (ARM_GRIPPER_CLOSE_DEG < ARM_SERVO3_MIN_DEG) || \
     (ARM_GRIPPER_CLOSE_DEG > ARM_SERVO3_MAX_DEG))
#error "Gripper open/close targets must stay inside Servo 3 limits"
#endif

#endif
