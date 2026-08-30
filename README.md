# ROS2 Automatic Seedling Clamping Robot

一套面向自动夹苗、运输与放苗作业的 ROS2 移动机器人。项目由本人完成结构、硬件、嵌入式软件与 ROS2 接口的全栈开发。

![Robot assembly](docs/media/robot-assembly.jpg)

## 项目视频

演示视频：[`docs/media/automatic-seedling-clamping-demo.mp4`](docs/media/automatic-seedling-clamping-demo.mp4)

## 项目职责

- SolidWorks 结构设计、零部件选型和整机装配
- STM32 主控、电机驱动、舵机与电源接线
- FreeRTOS 实时任务设计
- 底盘运动学、编码器测速与四轮 PID
- 四舵机执行机构与动作序列
- ROS2 速度/动作指令和状态反馈接口
- 整机联调、安全机制与连续作业验证

## 系统架构

```text
ROS2 Control / Automatic Sequence
               │ UART
               ▼
       STM32F103RC + FreeRTOS
          │               │
          ▼               ▼
  Chassis Control     Arm/Servo Control
  Kinematics + PID    Pick/Clamp/Place
          │               │
          └───────┬───────┘
                  ▼
          Safety State Machine
```

## 实时控制

基于 FreeRTOS 划分：

- 底盘控制任务
- 执行机构动作任务
- 通信解析任务
- 安全监控任务

系统控制周期约 20 ms。底盘采用四路编码器反馈与增量式 PID，并加入轮速归一化和速度斜坡。

## 执行机构

在定时器资源有限的条件下，使用两路硬件 PWM 与两路比较中断 GPIO 实现四路舵机控制：

- 取苗姿态
- 夹苗姿态
- 运输姿态
- 放苗姿态
- 连续点动
- 约 2°/20 ms 缓动

## 工作模式与安全机制

- 遥控调试模式
- 自动动作序列模式
- ROS2 速度与动作指令
- 底盘状态反馈
- 300 ms 失联停车
- 短刹、模式互锁和回中解锁

## 仓库结构

```text
firmware/stm32/   STM32F103RC + FreeRTOS 固件
hardware/         控制系统和硬件资料
mechanical/       STEP、STL、DXF 与 SolidWorks 结构文件
docs/media/       项目图片和演示视频
```

## 固件构建

1. 使用 STM32CubeMX 打开 `.ioc` 配置文件。
2. 检查时钟、定时器、编码器、UART 与 GPIO 配置。
3. 使用 Keil MDK 打开工程并编译。
4. 烧录前确认电机方向、舵机零位、急停和通信参数。

## English Summary

A full-stack ROS2 automatic seedling clamping robot covering mechanical design, hardware integration, STM32/FreeRTOS firmware, chassis kinematics and PID, servo motion sequencing, ROS2 communication, and safety control.

## 说明

本仓库用于个人技术归档与私有展示。第三方 HAL、CMSIS 与 FreeRTOS 文件遵循其原始许可条款。
