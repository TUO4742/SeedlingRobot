# 睿抗机器人 v1.4

这是面向“分苗必争”任务赛的 STM32F103RCT6 固件。底层硬件、CH559/PS2
通信、四电机编码器和速度 PID 以“差速底盘_5_11”为依据；应用代码按第一版
“摘苗”的 Robot / Headware / System 思路整理。两个参考工程均只读，不会被本工程覆盖。

## 已实现

- 左摇杆上下：前进 / 后退；左摇杆左右：连续左移 / 右移。两轴可同时生效，因此
  左上为左前斜行，右下为右后斜退。
- 右侧控制左右：原地逆时针 / 顺时针旋转。这里严格使用差速参考工程的
  CH559 `x6` 离散编码：`0x8F` 为左转、`0x2F` 为右转；没有猜测未使用字段为模拟轴。
- 按住 SELECT 后进入机械臂组合控制：L1/R1 控制大臂顺/逆时针，L2/R2
  控制小臂逆/顺时针，X 开爪，B 收爪。
- SELECT+上/下/左/右依次选择原 L1/L2/R1/R2 四组三舵机目标姿态。
- 舵机以每 20 ms 最大 2° 的方式平滑移动，不在控制循环中阻塞延时。
- PS2 有效帧超过 300 ms 未到达时，四轮目标立即归零，机械臂停止在当前角度。
- 四轮闭环 PID、麦克纳姆逆解、整体归一化和目标轮速斜坡。
- SELECT 按住时软件停车；松开后必须等待所有行走输入回中，底盘才重新解锁。
  它不是比赛要求的物理急停替代品。

## SELECT 组合操作

先将两个摇杆和按键全部回中，再按住 SELECT，随后按功能键：

| 组合键 | 动作 |
|---|---|
| SELECT + L1 / R1 | 大臂顺时针 / 逆时针点动 |
| SELECT + L2 / R2 | 小臂顺时针 / 逆时针点动 |
| 按住 SELECT + X / B | 旧夹爪持续开 / 合，松开保持 |
| 按住 SELECT + Y / A | 新夹爪持续开 / 合，松开保持 |
| SELECT + 上 / 下 / 左 / 右 | L1 / L2 / R1 / R2 预设姿态 |

CH559 的十字键与左摇杆拉满共用 `x4/x5`，X 与右侧左控制共用 `x6`，因此
SELECT 是必须的上下文键。按住 SELECT 时底盘保持停止；松开 SELECT 后也要先松开
方向键并让摇杆回中，才能继续行驶。SELECT+肩键组合依赖 CH559 将 `x7` 按位上报，
首次烧录后必须架空轮胎实测确认。

## 最常改的参数

打开 `Robot/robot_config.h`：

- `ARM_POSE_L1_*`、`ARM_POSE_L2_*`、`ARM_POSE_R1_*`、`ARM_POSE_R2_*`：四组三舵机角度。
- `ARM_SERVO*_MIN_DEG/MAX_DEG`：各关节安全角度范围。
- `ARM_SERVO*_REVERSED`：舵机逻辑方向。
- `ARM_MANUAL_JOG_STEP_DEG`：肩键每个新手柄帧的点动角度。
- `ARM_BIG_ARM_CW_SIGN`、`ARM_SMALL_ARM_CW_SIGN`：顺时针点动方向；方向错误时在
  `1` 与 `-1` 之间切换。
- `ARM_GRIPPER_OPEN_DEG`、`ARM_GRIPPER_CLOSE_DEG`：开爪、收爪角度。
- `ARM_SERVO_MIN_PULSE_US/MAX_PULSE_US`：舵机脉宽范围。
- `CHASSIS_MAX_VX_MPS/VY_MPS/WZ_RADPS`：三个方向的最大速度。
- 手柄回中、SELECT 停车或通信超时时，四路目标和 PID 历史立即清零，
  TB6612 进入短刹车状态；正常运动最高速度为上一版的约三分之二。
- `PS2_LEFT_Y_TO_FORWARD_SIGN`、`PS2_LEFT_X_TO_LEFT_SIGN`、
  `PS2_RIGHT_CONTROL_TO_CCW_SIGN`：三个控制方向的校准。

默认的四组姿态只是从第一版动作序列提取的起始值，必须在机械限位确认后再调成
实车姿态。

## 舵机接口

PCB 上有 6 个三线舵机接口，但只有 J9、J11 是原生定时器 PWM。当前三自由度方案：

| 逻辑舵机 | PCB 接口 | 信号 | 实现 |
|---|---|---|---|
| Servo 1 | J9 | PA0 / TIM5_CH1 | 硬件 PWM |
| Servo 2 | J11 | PA1 / TIM5_CH2 | 硬件 PWM |
| Servo 3 | J12 | PC5 | TIM5_CH3 内部比较中断驱动 GPIO |

J14/PB0 用于第四舵机，J15/PB1、J16/PB2 保留不用。六个接口的电源脚在 PCB 网络中均为
7.4 V；接舵机前必须确认其额定电压和堵转电流，不要把“舵机口”理解为 5 V 电源。

完整接线、轮序和上电检查见 `Doc/接线与调试说明.md`。

## 编译

用 Keil MDK-ARM 打开：

`MDK-ARM/1_template_led.uvprojx`

工具链为 ARM Compiler 5.06 update 5。最终交付前已执行完整重编译，日志位于
`MDK-ARM/v1_4_build.log`；HEX/AXF 在 `MDK-ARM/1_template_led/`。

## 重要边界

- 当前只完成静态检查和工具链编译，未替你烧录，也不能代替架空轮胎、示波器和
  实际负载测试。
- 首次带机械结构上电前，先把三个角度范围收窄到不会碰撞的区间，并拆下舵盘或
  断开连杆逐路确认方向。
- 先架空验证四轮的 FL/FR/RL/RR 插口、物理正方向与编码器正号，再落地测试前进、
  左移和逆时针旋转。
- 必须安装可直接切断执行器动力的物理急停；SELECT 只是一层软件停车。
- CubeMX 中 TIM5_CH3 不映射到 PA2，PA2 仍是 USART2_TX。第三舵机的 CH3 内部比较
  配置位于 `Src/tim.c` 的 USER CODE 区；`Headware/servo.c` 只开启 CC3 中断并明确关闭
  CC3E，不能改成 `HAL_TIM_OC_Start_IT()`。重新生成代码后仍要检查这两段和 TIM5 IRQ。
