# ROS2 导航设计与验证

本文记录 `ros2_ws/` 的设计取舍、参数来源、已完成验证和实机验收计划。
目的是让接手的人能判断「哪些是验证过的、哪些还没」。

---

## 1. 为什么是重写而不是复制

参考项目 `RosCar-Algorithm` 是**差速 / 部分麦轮**方案，与本机差异太大，直接复制会留下隐性错误：

| 维度 | RosCar | 本机 | 直接复制的后果 |
| --- | --- | --- | --- |
| 底盘 | 差速为主 | 麦克纳姆四轮 | 横移功能直接丢失 |
| 下行 | 左右轮角速度 | vx / vy / wz | 两个轮子喂四个轮子，解算错 |
| 上行 | IMU + 轮速 | 仅编码器计数 | 里程计拿不到数据 |
| 轮距参数 | 差速轮距 | `K = L+W = 0.168` | yaw 尺度错 |
| 限速 | 0.60 / 0.50 / 0.8 | 0.35 / 0.30 / 0.8 | 小车打滑、横移失控 |
| 通信 | 二进制帧 | 原本只有手柄 ASCII | 协议完全对不上 |

所以保留**架构与参数思想**，重写**实现与数值**。

---

## 2. 协议设计取舍

### 2.1 下行为什么发车体速度，不发四轮轮速

本机 MCU 里 `decompose_and_normalize()` 已经实现了麦轮逆解、归一化和速度斜坡。
如果 ROS 侧再算一遍轮速下发，就等于**两处实现同一套运动学**，一旦改轮径或改轮距必然不同步。

→ 下行只发 `vx / vy / wz`，**IK 只存在于 MCU 一处**。

### 2.2 上行为什么发原始计数，不发速度

如果 MCU 换算成 m/s 再发，那么换算常数（轮径、编码器线数、减速比）就被固化在固件里。
以后换码盘或改减速比，要同时改固件和 ROS。

→ 上行发 **int32 原始计数**，换算集中在 `mecanum.cpp`，**改一处即可**。

### 2.3 为什么是 14 / 22 字节

| 帧 | 长度推导 |
| --- | --- |
| 下行 | 1 头 + 1 标志 + 4 `vx` + 4 `vy` + 2 `wz` + 1 校验 + 1 尾 = **14** |
| 上行 | 1 头 + 1 状态 + 4×4 计数 + 2 保留 + 1 校验 + 1 尾 = **22** |

下行沿用参考项目已有的 14 字节长度，固件侧缓冲区和解析窗口可以直接复用同一套滑动窗口写法。

### 2.4 为什么用异或而不是 CRC

- 帧长固定 14/22 字节，且运行在**板内短距离 UART**（<20 cm 排线），误码率很低；
- 异或实现是一行代码，MCU 侧零成本；
- 实测可拦截**单字节翻转**（见 `tools/protocol_selftest.py`）。

**已知不足**：异或无法定位于双字节错误。若后续改用长线或高波特率，建议换 CRC-8/CRC-16。

---

## 3. 坐标系与 TF 分工

```text
map  ──(AMCL)──▶  odom_combined  ──(EKF)──▶  base_footprint  ──(URDF 固定关节)──▶  base_link
```

| 变换 | 发布者 |
| --- | --- |
| `map → odom_combined` | `nav2_amcl` |
| `odom_combined → base_footprint` | `robot_localization`（`publish_tf: true`） |
| `base_footprint → base_link` | `robot_state_publisher`（URDF） |
| `base_link → laser_link` | `robot_state_publisher`（URDF） |

**`chassis_node` 只发布 `/odom` 话题，不广播任何 TF。**

这是刻意的：参考项目里出现过「桥接节点和 EKF 同时发同一对父子 TF」导致位姿跳变的问题。
一条 link 只能有一个发布者，职责必须唯一。

---

## 4. 参数来源

每个数值都能追到固件里的出处，不是拍脑袋：

| 参数 | 值 | 来源 |
| --- | --- | --- |
| `wheel_base_sum_m` | 0.168 | `robot_config.h` `CHASSIS_WHEEL_BASE_SUM_M` |
| `wheel_diameter_m` | 0.065 | `motor.h` `WHEEL_DIAMETER` |
| `encoder_line` | 11 | `motor.h` `ENC_LINE` |
| `encoder_multiple` | 4 | `motor.h` `ENC_MULTIPLE` |
| `gear_ratio` | 30 | `motor.h` `GEAR_RATIO` |
| `counts_per_rev` | 1320 | 11 × 4 × 30 |
| `rate_hz` | 50 | `robot_config.h` `ROBOT_CONTROL_PERIOD_MS 20` |
| `cmd_timeout_ms` | 300 | `robot_config.h` `PS2_LINK_TIMEOUT_MS` |
| `max_vx` | 0.35 | 低于 RosCar 0.60；车小、无 IMU、纯轮速航向 |
| `max_vy` | 0.30 | 麦轮横移效率约 70%，保守取值 |
| `max_wz` | 0.80 | 与 RosCar 同量级，实测可达 |
| `controller_frequency` | 10 | 与 MPPI `model_dt 0.1` 同量级，避免控制周期内跑多次预测 |

---

## 5. 已完成的验证

以下结果由仓库内脚本产生，可复现：

```bash
python3 ros2_ws/tools/kinematics_check.py
python3 ros2_ws/tools/protocol_selftest.py
```

### 5.1 运动学（`kinematics_check.py`）

```text
最大闭环误差: 2.220e-16      → 正解与逆解严格互逆
```

| 用例 | FL | RR | RL | FR |
| --- | ---: | ---: | ---: | ---: |
| 直行 `vx=0.3` | +0.300 | +0.300 | +0.300 | +0.300 |
| 左移 `vy=0.3` | −0.300 | −0.300 | +0.300 | +0.300 |
| 原地逆时针 `wz=0.6` | −0.101 | +0.101 | −0.101 | +0.101 |

符号语义与固件一致。

**重要发现（编码器分辨率）**：

| 项 | 值 |
| --- | ---: |
| 每转计数 | 1320 |
| 每计数位移 | 0.1547 mm |
| 20 ms 内 1 计数对应速度 | **7.735 mm/s** |
| 0.1 m/s 时每周期计数 | 12.9 |
| 0.01 m/s 时每周期计数 | 1.29 |

→ **低于约 8 mm/s 时，单周期只走 0~1 个计数，速度量化非常明显**。
这是硬件条件决定的下限，不是调参能解决的；低速原地对准时会出现「走走停停」。

### 5.2 协议（`protocol_selftest.py`）

- 下行 8 组用例往返：`vx/vy` 误差 < 1e-6，`wz` 误差 ≤ 5e-4（1 mrad/s 量化）
- 上行 int32 边界值（`2^31-1`、`-2^31`）无损往返
- 单字节翻转被异或校验拦截（上下行均验证）

### 5.3 静态校验

| 项 | 结果 |
| --- | --- |
| `launch/*.py` 语法 | 通过 |
| `config/*.yaml` 解析 | 通过 |
| `urdf/*.urdf` XML 结构 | 通过 |
| C++ 编译 | **未执行**（本机无 ROS2 头文件） |

---

## 6. 实机验收计划

按顺序推进，每步不过不进入下一步。

### 阶段 0：固件通信

- [ ] `ros_bridge.c` 上电后能收到下行帧（串口助手抓包确认头部 `7B` 和尾部 `7D`）
- [ ] 上位机能收到上行 22 字节帧，校验位正确
- [ ] 拔掉串口 200 ms 后底盘自动停车

### 阶段 1：里程计尺度

- [ ] 架空车架，手动转轮一圈，`/odom` 位移 ≈ `π × 0.065 = 0.204 m`
- [ ] 地面直线推 1.0 m，`/odom` 报 1.0 ± 0.02 m
- [ ] 地面原地转 360°，`/odom` yaw ≈ 6.28 ± 0.05 rad

> 若尺度偏差稳定，优先怀疑 `wheel_diameter_m`，而不是改代码。

### 阶段 2：闭环控制

- [ ] `ros2 topic pub /cmd_vel` 发 0.5 rad/s，实测 0.5 ± 0.05 rad/s
- [ ] 发 `linear.y=0.2`，实测横移方向正确（`+y` 向左）
- [ ] 发 `linear.x=0.2`，实测前进方向正确

### 阶段 3：建图与定位

- [ ] `slam.launch.py` 建图，闭环无明显错层
- [ ] 保存地图后 `nav2.launch.py` 启动，AMCL 收敛
- [ ] RViz 中激光与地图重合

### 阶段 4：导航

- [ ] 单点导航到 1 m 内目标，`xy_goal_tolerance 0.10` 达标
- [ ] 含横移的路径能实际横移（若只走前进，先查 `velocity_smoother` 第二项）
- [ ] 障碍物前能绕行或停车

---

## 7. 调参顺序

**不要一上来调 MPPI。** 按依赖顺序：

1. **里程计尺度**（阶段 1）——尺度错，后面全是白调
2. **AMCL 收敛**（`update_min_d` / `update_min_a`）——定位飘，MPPI 再好也没用
3. **`velocity_smoother` 限幅**——确认横移没被截断
4. **MPPI 权重**（最后才动）
   - 走不到目标 → 加 `PathFollowCritic`
   - 贴不住路径 → 加 `PathAlignCritic`
   - 撞障碍 → 加 `ObstaclesCritic`
   - 原地抖动 → 降 `wz_std`、加 `PathAngleCritic`

---

## 8. 故障排查

| 现象 | 优先检查 |
| --- | --- |
| `/odom` 不动 | 上行帧是否收到；编码器 `s_raw_encoder_count` 是否变化；串口是否被手柄占用 |
| `/odom` 尺度差 2 倍 | `wheel_diameter_m` 是否写成半径；`gear_ratio` 是否反了 |
| 一直原地旋转 | AMCL `robot_model_type` 是否 `OmniMotionModel`；`min_particle` 是否过小 |
| 只走前进不横移 | `velocity_smoother.max_velocity` 第二项是否为 0；MPPI `motion_model` 是否 `Omni` |
| 位姿周期性跳变 | 是否有多余节点在发 `odom→base_footprint`（本设计中只有 EKF 该发） |
| 车抖 | `model_dt` 与 `controller_frequency` 是否匹配；`vx_std/vy_std` 是否过大 |
| 低速对准反复启停 | 编码器分辨率下限（7.7 mm/s），属硬件限制 |

---

## 9. 变更记录

| 日期 | 变更 |
| --- | --- |
| 2026-10 | 建立 `ros2_ws/`：底盘桥接 + Nav2 参数 + URDF + 自检脚本 |
