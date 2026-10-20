#!/usr/bin/env python3
"""运动学自检：正解/逆解闭环 + 编码器分辨率。

公式与固件保持一致：
  chassis_task.c:169-172  逆解
  encoder.c:96-98         正解
单位：m、m/s、rad/s。

用法：
    python3 tools/kinematics_check.py
退出码 0 表示全部通过。
"""

import math

# ---- 与 robot_config.h / motor.h 一致 ----
WHEEL_BASE_SUM_M = 0.168      # L + W
WHEEL_DIAMETER_M = 0.065
ENC_LINE = 11.0
ENC_MULTIPLE = 4.0
GEAR_RATIO = 30.0
CONTROL_PERIOD_S = 0.020
MAX_WHEEL_MPS = 0.72

# 轮序：0=FL, 1=RR, 2=RL, 3=FR
FL, RR, RL, FR = 0, 1, 2, 3


def counts_per_rev() -> float:
    return ENC_LINE * ENC_MULTIPLE * GEAR_RATIO


def wheel_circumference() -> float:
    return math.pi * WHEEL_DIAMETER_M


def body_to_wheels(vx: float, vy: float, wz: float):
    """逆解，与 chassis_task.c:169-172 一致。"""
    k = WHEEL_BASE_SUM_M * wz
    w = [0.0, 0.0, 0.0, 0.0]
    w[FL] = vx - vy - k
    w[RR] = vx - vy + k
    w[RL] = vx + vy - k
    w[FR] = vx + vy + k
    return w


def wheels_to_body(w):
    """正解，与 encoder.c:96-98 一致。"""
    vx = (w[FL] + w[RR] + w[RL] + w[FR]) * 0.25
    vy = (-w[FL] - w[RR] + w[RL] + w[FR]) * 0.25
    wz = (-w[FL] + w[RR] - w[RL] + w[FR]) / (4.0 * WHEEL_BASE_SUM_M)
    return vx, vy, wz


def counts_to_wheels(delta_counts, dt: float):
    m_per_count = wheel_circumference() / counts_per_rev()
    return [c * m_per_count / dt for c in delta_counts]


def normalize(wheels):
    """与固件一致：任一轮超限则整体等比缩放，保持方向不变。"""
    peak = max(abs(w) for w in wheels)
    if peak <= MAX_WHEEL_MPS:
        return wheels, 1.0
    scale = MAX_WHEEL_MPS / peak
    return [w * scale for w in wheels], scale


def check_roundtrip():
    cases = [
        (0.30, 0.00, 0.00),    # 直行
        (-0.20, 0.00, 0.00),   # 倒车
        (0.00, 0.25, 0.00),    # 左横移
        (0.00, -0.25, 0.00),   # 右横移
        (0.00, 0.00, 0.60),    # 原地逆时针
        (0.00, 0.00, -0.60),   # 原地顺时针
        (0.20, 0.15, 0.40),    # 边走边转
        (0.35, 0.30, 0.80),    # 限幅边界
    ]
    print("=" * 68)
    print("正解/逆解闭环校验  K=L+W=%.3f m  r=%.4f m" %
          (WHEEL_BASE_SUM_M, WHEEL_DIAMETER_M / 2))
    print("=" * 68)
    print(f"{'vx':>7} {'vy':>7} {'wz':>7} | {'vx_out':>8} {'vy_out':>8} {'wz_out':>8} | {'max_err':>9}")
    worst = 0.0
    for vx, vy, wz in cases:
        w = body_to_wheels(vx, vy, wz)
        vx2, vy2, wz2 = wheels_to_body(w)
        err = max(abs(vx2 - vx), abs(vy2 - vy), abs(wz2 - wz))
        worst = max(worst, err)
        print(f"{vx:7.3f} {vy:7.3f} {wz:7.3f} | "
              f"{vx2:8.6f} {vy2:8.6f} {wz2:8.6f} | {err:9.2e}")
    print("-" * 68)
    print(f"最大闭环误差: {worst:.3e}")
    assert worst < 1e-9, "正解/逆解不互逆"
    print("PASS: 正解与逆解严格互逆")
    return worst


def check_wheel_signs():
    """方向语义校验：前进时四轮同向；左移时左前/左后反向。"""
    print()
    print("=" * 68)
    print("轮速符号语义")
    print("=" * 68)
    expectations = [
        ("直行 vx>0", body_to_wheels(0.3, 0, 0), lambda w: w[FL] > 0 and w[FR] > 0 and w[RL] > 0 and w[RR] > 0),
        ("左移 vy>0", body_to_wheels(0, 0.3, 0), lambda w: w[FL] < 0 and w[RL] > 0),
        ("原地逆时针 wz>0", body_to_wheels(0, 0, 0.6), lambda w: w[FL] < 0 and w[RR] > 0 and w[RL] < 0 and w[FR] > 0),
    ]
    for name, w, pred in expectations:
        ok = pred(w)
        print(f"  {'OK ' if ok else 'FAIL'} {name:<18} "
              f"FL={w[FL]:+.3f} RR={w[RR]:+.3f} RL={w[RL]:+.3f} FR={w[FR]:+.3f}")
        assert ok, f"符号语义不符: {name}"
    print("PASS: 轮速符号与固件语义一致")


def check_resolution():
    """低速分辨率：20 ms 采样下 1 个计数对应多快。"""
    print()
    print("=" * 68)
    print("编码器分辨率")
    print("=" * 68)
    cpr = counts_per_rev()
    m_per_count = wheel_circumference() / cpr
    min_speed = m_per_count / CONTROL_PERIOD_S
    print(f"  每转计数        : {cpr:.0f}")
    print(f"  每计数位移      : {m_per_count * 1000:.4f} mm")
    print(f"  20 ms 下 1 计数 : {min_speed * 1000:.3f} mm/s")
    print(f"  0.1 m/s 时 20 ms: {0.1 * CONTROL_PERIOD_S / m_per_count:.1f} counts")
    print(f"  0.01 m/s 时 20 ms: {0.01 * CONTROL_PERIOD_S / m_per_count:.2f} counts")
    print("  -> 低于约 8 mm/s 时，单周期可能只走 0~1 个计数，速度量化明显")


def check_normalize():
    print()
    print("=" * 68)
    print("轮速归一化")
    print("=" * 68)
    w = body_to_wheels(0.35, 0.30, 0.80)
    nw, scale = normalize(w)
    print(f"  原始: FL={w[FL]:+.3f} RR={w[RR]:+.3f} RL={w[RL]:+.3f} FR={w[FR]:+.3f}")
    print(f"  缩放系数: {scale:.4f}")
    print(f"  归一: FL={nw[FL]:+.3f} RR={nw[RR]:+.3f} RL={nw[RL]:+.3f} FR={nw[FR]:+.3f}")
    assert max(abs(x) for x in nw) <= MAX_WHEEL_MPS + 1e-9
    print("  PASS: 任一轮超限时整体等比缩放，方向不变，峰值不超上限")


if __name__ == "__main__":
    check_roundtrip()
    check_wheel_signs()
    check_resolution()
    check_normalize()
    print()
    print("ALL CHECKS PASSED")
