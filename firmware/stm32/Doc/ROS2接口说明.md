# ROS2 底盘接口（固件侧改动说明）

本文件说明为了让 `ros2_ws/src/seedling_chassis` 能驱动本机，
STM32 固件需要补什么。**不改动现有 PS2 通路**，只在旁边加一条并行通道。

## 1. 为什么不能直接用现有串口

现状（`Headware/ps2_usart5.c`）：

- UART5 @ 57600，ASCII 文本行 `HUB0_Joystick data: xNN ×8`
- 只有手柄输入，**没有速度指令入口**，也**没有编码器回传出口**
- 校验只有首字节 `x1 == 0x01`，无长度、无 CRC

导航需要的是：

1. 上层给**速度**（vx / vy / wz），而不是摇杆量
2. 底层回**编码器累计计数**，让 ROS 积分里程计

## 2. 建议实现

新增 `Robot/ros_bridge.c` 与 `Robot/ros_bridge.h`，用另一路串口
（UART4 更合适，UART5 留给手柄）。

### 2.1 下行（14 字节，50 Hz）

| 字节 | 内容 |
| ---: | --- |
| 0 | `0x7B` |
| 1 | 标志位：bit0 使能，bit1 急停 |
| 2–5 | `vx` float32 大端 |
| 6–9 | `vy` float32 大端 |
| 10–11 | `wz` int16 大端（0.001 rad/s） |
| 12 | byte0..byte11 异或 |
| 13 | `0x7D` |

### 2.2 上行（22 字节，50 Hz）

| 字节 | 内容 |
| ---: | --- |
| 0 | `0x7B` |
| 1 | 状态：bit0 急停，bit1 PS2 在线 |
| 2–5 | 编码器 0（FL）int32 大端 |
| 6–9 | 编码器 1（RR）int32 大端 |
| 10–13 | 编码器 2（RL）int32 大端 |
| 14–17 | 编码器 3（FR）int32 大端 |
| 18–19 | 保留（电压 / 电流） |
| 20 | byte0..byte19 异或 |
| 21 | `0x7D` |

## 3. 在 `chassis_task` 里怎么接

`Robot/chassis_task.c:208-285` 的主循环中，在
`apply_ps2_command(&ps2)` 之后、`set_wheel_targets()` 之前插入：

```c
/* 导航速度优先，但 PS2 一动作就抢回控制权 */
uint8_t have_ros = ros_bridge_poll();          /* 解析下行帧，返回是否收到有效帧 */
if (have_ros && (ps2.lx == 0) && (ps2.ly == 0) && (chassis.vz == 0.0f)) {
    chassis.vx = ros_bridge_vx();
    chassis.vy = ros_bridge_vy();
    chassis.vz = ros_bridge_wz();
} else if (have_ros) {
    /* 手柄有输入，保持手柄优先，避免调试时被上层覆盖 */
}

ros_bridge_tick(now_ms);                        /* 组上行帧：4 路 s_raw_encoder_count */
```

> 关键：`chassis.vx/vy` 已经是**车体速度（m/s）**，
> 现有 `decompose_and_normalize()` 会自己做麦轮逆解，
> 所以 ROS 侧下发的就是车体速度，**不要在下发前再算一遍轮速**。

## 4. 编码器计数从哪来

`Headware/encoder.c` 里已有：

```c
volatile int32_t s_raw_encoder_count[4];   /* 0=FL, 1=RR, 2=RL, 3=FR */
```

直接把这四个值按大端写入上行帧的 2–17 字节即可，**不要做单位换算**，
换算放在 ROS 侧（`mecanum.cpp`），这样 encoder 分辨率变化时只改一处。

## 5. 解析写法

沿用项目里已有的 `ros_comm_process_rx_bytes()` 风格：**滑动窗口定长匹配**。

```c
while (offset + 14 <= len) {
    if (buf[offset] != 0x7B || buf[offset + 13] != 0x7D) { offset++; continue; }
    if (xor8(&buf[offset], 12) != buf[offset + 12])       { offset++; continue; }
    handle_frame(&buf[offset]);
    offset += 14;
}
```

不要写成多层 `switch(state)` 状态机，本项目的帧是定长的，滑动窗口更简单可靠。

## 6. 故障与超时

| 场景 | 处理 |
| --- | --- |
| 200 ms 未收到有效下行帧 | `vx=vy=wz=0`，底盘短刹（复用 `chassis_force_stop()`） |
| 上行发送失败 | 计数并在 `ps2` 全零时同样停车 |
| PS2 与 ROS 同时有输入 | 手柄优先（`chassis_task.c` 现有 `ps2_active` 逻辑），并在状态位回传给上层 |

## 7. 需要同步修改的地方

1. `Robot/robot_config.h`：新增 `ROS_LINK_TIMEOUT_MS (200U)`
2. `Src/usart.c`：新增一路 UART4 初始化（115200 8N1）+ DMA
3. `Src/stm32f1xx_it.c`：挂 UART4 IDLE 回调
4. `Robot/chassis_task.c`：按第 3 节插入 5 行调用
5. `tests/verify_project.py`：注意它目前把 `ros_comm` / `remote_task` 列为**禁止符号**，
   新增文件后需要更新这份断言，否则自检脚本会误报
