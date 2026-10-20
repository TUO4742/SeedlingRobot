#!/usr/bin/env python3
"""串口协议自检：14 字节下行 / 22 字节上行的编解码与校验。

不依赖任何第三方库，可在 PC 上直接跑：
    python3 ros2_ws/tools/protocol_selftest.py
退出码 0 表示全部通过。

协议定义见 ros2_ws/README.md 第 2 节。
"""

import struct

FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
DOWN_LEN = 14
UP_LEN = 22


def xor8(data) -> int:
    s = 0
    for b in data:
        s ^= b
    return s


# ---------------- 下行：ROS -> MCU ----------------

def encode_down(vx: float, vy: float, wz: float, enable: bool = True, estop: bool = False) -> bytes:
    flags = (0x01 if enable else 0x00) | (0x02 if estop else 0x00)
    wz_milli = int(round(wz * 1000.0))
    wz_milli = max(-32768, min(32767, wz_milli))
    body = bytearray()
    body.append(FRAME_HEAD)
    body.append(flags)
    body += struct.pack(">f", vx)
    body += struct.pack(">f", vy)
    body += struct.pack(">h", wz_milli)
    assert len(body) == 12, len(body)
    body.append(xor8(body))
    body.append(FRAME_TAIL)
    return bytes(body)


def decode_down(frame: bytes):
    assert len(frame) == DOWN_LEN, len(frame)
    assert frame[0] == FRAME_HEAD and frame[13] == FRAME_TAIL
    assert xor8(frame[:12]) == frame[12], "校验失败"
    flags = frame[1]
    vx = struct.unpack(">f", frame[2:6])[0]
    vy = struct.unpack(">f", frame[6:10])[0]
    wz = struct.unpack(">h", frame[10:12])[0] / 1000.0
    return vx, vy, wz, bool(flags & 0x01), bool(flags & 0x02)


# ---------------- 上行：MCU -> ROS ----------------

def encode_up(enc, status: int = 0x00) -> bytes:
    assert len(enc) == 4
    body = bytearray()
    body.append(FRAME_HEAD)
    body.append(status)
    for c in enc:
        body += struct.pack(">i", int(c))
    body += struct.pack(">H", 0)          # 保留：电压/电流
    assert len(body) == 20, len(body)
    body.append(xor8(body))
    body.append(FRAME_TAIL)
    return bytes(body)


def decode_up(frame: bytes):
    assert len(frame) == UP_LEN, len(frame)
    assert frame[0] == FRAME_HEAD and frame[21] == FRAME_TAIL
    assert xor8(frame[:20]) == frame[20], "校验失败"
    status = frame[1]
    enc = [struct.unpack(">i", frame[2 + i * 4: 6 + i * 4])[0] for i in range(4)]
    return enc, status


# ---------------- 测试 ----------------

def test_down_roundtrip():
    print("=" * 62)
    print("下行帧（ROS -> MCU）14 字节")
    print("=" * 62)
    cases = [
        (0.0, 0.0, 0.0),
        (0.30, 0.0, 0.0),
        (-0.20, 0.0, 0.0),
        (0.0, 0.25, 0.0),
        (0.0, 0.0, 0.60),
        (0.0, 0.0, -0.60),
        (0.35, 0.30, 0.80),
        (0.123456, -0.234567, -0.789),
    ]
    for vx, vy, wz in cases:
        f = encode_down(vx, vy, wz)
        vx2, vy2, wz2, en, es = decode_down(f)
        wz_err = abs(wz2 - wz)
        assert abs(vx2 - vx) < 1e-6, (vx, vx2)
        assert abs(vy2 - vy) < 1e-6, (vy, vy2)
        assert wz_err <= 5e-4, (wz, wz2)
        print(f"  OK  vx={vx:+.4f} vy={vy:+.4f} wz={wz:+.4f}  "
              f"-> {f.hex(' ')}")
    print(f"  帧长 {DOWN_LEN} 字节，逐字节异或校验")
    print("PASS: 下行往返误差 vx/vy < 1e-6, wz < 5e-4（1 mrad/s 量化）")


def test_up_roundtrip():
    print()
    print("=" * 62)
    print("上行帧（MCU -> ROS）22 字节")
    print("=" * 62)
    cases = [
        [0, 0, 0, 0],
        [1320, -1320, 660, -660],
        [2147483647, -2147483648, 1, -1],
        [123456, 234567, -345678, -456789],
    ]
    for enc in cases:
        f = encode_up(enc, status=0x01)
        enc2, st = decode_up(f)
        assert enc2 == enc, (enc, enc2)
        print(f"  OK  enc={enc}  status=0x{st:02X}")
    print(f"  帧长 {UP_LEN} 字节，逐字节异或校验，编码器为 int32 大端")
    print("PASS: 上行 int32 边界值无损往返")


def test_corruption():
    print()
    print("=" * 62)
    print("校验能力")
    print("=" * 62)
    f = bytearray(encode_down(0.3, 0.1, 0.5))
    f[5] ^= 0xFF  # 破坏一个载荷字节
    try:
        decode_down(bytes(f))
        raise AssertionError("损坏帧竟然通过了校验")
    except AssertionError as e:
        if "损坏帧" in str(e):
            raise
        print("  OK  单字节翻转被异或校验拦截")

    g = bytearray(encode_up([100, 200, 300, 400]))
    g[10] ^= 0x01
    try:
        decode_up(bytes(g))
        raise AssertionError("损坏帧竟然通过了校验")
    except AssertionError as e:
        if "损坏帧" in str(e):
            raise
        print("  OK  上行单字节翻转同样被拦截")
    print("PASS: 异或校验可拦截单字节错误（定位能力有限，见文档说明）")


if __name__ == "__main__":
    test_down_roundtrip()
    test_up_roundtrip()
    test_corruption()
    print()
    print("ALL CHECKS PASSED")
