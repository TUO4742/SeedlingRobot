# 拔苗机器人 ROS2 导航栈

本目录把 `RosCar-Algorithm` 的导航链路迁移到拔苗机器人（麦克纳姆全向底盘），
按本机实际参数重新标定，不是照搬。

## 1. 与 RosCar 的对应关系

| 环节 | RosCar | 本项目 |
| --- | --- | --- |
| 底盘桥接 | `turn_on_wit_robot` → `base_node` | `seedling_chassis` → `chassis_node` |
| 通信 | 14 B 二进制帧，左右轮 float32 | 14 B 二进制帧，vx/vy/wz |
| 运动学 | 差速 / 部分车型麦轮 | 麦轮四轮，IK 在 MCU，FK 在 ROS |
| 里程计 | MCU 回传 → `/odom` | MCU 回传编码器计数 → `/odom` |
| 融合 | `robot_localization` EKF | 同（`ekf.yaml`） |
| 定位 | AMCL `OmniMotionModel` | 同（麦轮必须用 Omni） |
| 控制 | RotationShim + MPPI(`Omni`) | 同，限速按本机下调 |
| 坐标 | `odom_combined → base_footprint` | 同 |

## 2. 串口协议

> ⚠️ 现行固件 UART5 只解析 PS2 手柄 ASCII 文本。接入导航需要在固件里
> **并行增加**这套二进制协议（见第 6 节），PS2 保留为手动调试通道。

### 2.1 下行 ROS2 → MCU（14 字节，50 Hz）

| 字节 | 内容 | 格式 |
| ---: | --- | --- |
| 0 | 帧头 | `0x7B` |
| 1 | 标志位 | bit0=使能, bit1=急停 |
| 2–5 | `vx` 前进速度 | float32 大端，m/s |
| 6–9 | `vy` 横移速度 | float32 大端，m/s（正=左） |
| 10–11 | `wz` 角速度 | int16 大端，单位 0.001 rad/s（正=逆时针） |
| 12 | 校验 | byte0..byte11 逐字节异或 |
| 13 | 帧尾 | `0x7D` |

### 2.2 上行 MCU → ROS2（22 字节，50 Hz）

| 字节 | 内容 | 格式 |
| ---: | --- | --- |
| 0 | 帧头 | `0x7B` |
| 1 | 状态 | bit0=急停, bit1=PS2 在线 |
| 2–5 | 编码器 0（FL） | int32 大端，累计计数 |
| 6–9 | 编码器 1（RR） | int32 大端 |
| 10–13 | 编码器 2（RL） | int32 大端 |
| 14–17 | 编码器 3（FR） | int32 大端 |
| 18–19 | 保留 | 电压 / 电流，先填 0 |
| 20 | 校验 | byte0..byte19 逐字节异或 |
| 21 | 帧尾 | `0x7D` |

## 3. 运动学

轮序 `0=FL, 1=RR, 2=RL, 3=FR`，`K = L+W = 0.168 m`。

**逆解（MCU 内已实现，ROS 侧只做限幅）**

```text
FL = vx - vy - K*wz
RR = vx - vy + K*wz
RL = vx + vy - K*wz
FR = vx + vy + K*wz
```

**正解（ROS 侧 `mecanum.cpp`，与固件 `encoder.c` 一致）**

```text
vx = ( FL + RR + RL + FR) / 4
vy = (-FL - RR + RL + FR) / 4
wz = (-FL + RR - RL + FR) / (4K)
```

**编码器 → 轮缘线速度**

```text
ENC_LINE=11, ENC_MULTIPLE=4, GEAR_RATIO=30  → 1320 counts / 轮转
r = WHEEL_DIAMETER/2 = 0.0325 m
v = Δcount / 1320 * 2πr / dt
```

## 4. 里程计与 TF

`chassis_node` 只发布 **`/odom` 话题**，不广播 TF（与 RosCar 一致，
避免和 EKF 抢同一对父子的发布权）。

```text
chassis_node  →  /odom
robot_localization EKF  →  TF odom_combined → base_footprint
AMCL  →  TF map → odom_combined
```

EKF 输入：

- `odom0: /odom`（差分模式关闭，直接融合连续位姿）
- `imu0: /imu/data_raw`（若扳机机装了 WT9011G4K 才启用，否则注释掉）

## 5. 编译与运行

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

# 1) 底盘 + EKF
ros2 launch seedling_bringup chassis.launch.py

# 2) 建图（可选，首次需先建图）
ros2 launch seedling_bringup slam.launch.py

# 3) 导航（需已有地图）
ros2 launch seedling_bringup nav2.launch.py map:=/path/to/map.yaml
```

## 6. 固件需要补的代码

`firmware/stm32/Robot/` 下新增一组接口即可，**不要动现有 PS2 通路**：

1. 新增 `ros_bridge.c/.h`：UART4（建议单独一路，115200 8N1）收发 14/22 字节帧。
2. `chassis_task` 主循环里，每周期：
   - 若收到有效下行帧 → 用帧里的 `vx/vy/wz` 覆盖 `apply_ps2_command()` 的结果；
   - 否则沿用 PS2 结果（手动优先，方便调试）。
3. 每 20 ms 组装一帧上行，填入 4 路 `s_raw_encoder_count[]`。
4. 复用现有 `decompose_and_normalize()` 做逆解，`encoder.c` 的正解不用改。

解析建议沿用现有 `ros_comm_process_rx_bytes()` 的**滑动窗口定长匹配**写法，
不要写成嵌套状态机。

## 7. 与 RosCar 的差异（必须知道）

| 项 | RosCar | 本项目 | 原因 |
| --- | --- | --- | --- |
| 下行内容 | 左右轮角速度 | vx/vy/wz | IK 已在 MCU，避免两处重复 |
| 上行内容 | IMU 原始量 | 编码器计数 | 本机无 IMU，靠编码器积分 |
| 最大速度 | 0.60 / 0.50 / 0.8 | 0.35 / 0.30 / 0.8 | 小车轴距更短、更轻 |
| 轮距参数 | 未用 | 0.168 | 麦轮必须 |

## 8. 已知限制

- 无 IMU 时，`wz` 完全依赖四轮正解，打滑会导致航向漂移；
- 编码器计数 1320/转，20 ms 采样，低速分辨率约 0.0077 m/s，接近停止时量化明显；
- 麦轮横移效率低，`vy_max` 已按 0.30 收敛，不要盲目调高。

## 9. 自检工具（无需硬件）

```bash
python3 tools/kinematics_check.py     # 正解/逆解闭环、轮速符号、编码器分辨率
python3 tools/protocol_selftest.py    # 14B/22B 帧编解码与异或校验
```

均为纯 Python、无第三方依赖，退出码 0 表示通过。

`kinematics_check.py` 会打印一个硬件事实：**1320 counts/转、20 ms 采样下，
1 个计数对应 7.7 mm/s**，因此低于约 8 mm/s 时速度量化明显，低速对准会「走走停停」。
这是分辨率下限，不是调参问题。

## 10. 编译注意

- 本工作区 C++ **尚未在 ROS2 环境编译过**，首次 `colcon build` 可能需要补依赖。
- `seedling_chassis` 依赖 POSIX 串口（`termios`），仅支持 Linux。
- 建议为底盘串口建 udev 规则，例如：

```text
# /etc/udev/rules.d/99-seedling-chassis.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="seedling_chassis"
```

然后把 `chassis.yaml` 里的 `serial_port` 改成 `/dev/seedling_chassis`。
