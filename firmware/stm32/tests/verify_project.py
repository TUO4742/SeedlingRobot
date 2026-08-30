"""Static consistency checks for the RuiKang v1.4 firmware project."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8-sig")


def macro(source: str, name: str) -> str:
    match = re.search(rf"^#define\s+{re.escape(name)}\s+([^\r\n]+)", source, re.M)
    require(match is not None, f"missing macro {name}")
    value = match.group(1).split("/*", 1)[0].split("//", 1)[0].strip()
    if value.startswith("(") and value.endswith(")"):
        value = value[1:-1].strip()
    return value


def uint_macro(source: str, name: str) -> int:
    return int(macro(source, name).rstrip("Uu"), 0)


def check_keil_project() -> None:
    project = ROOT / "MDK-ARM" / "1_template_led.uvprojx"
    tree = ET.parse(project)
    paths = {
        node.text.replace("\\", "/")
        for node in tree.findall(".//FilePath")
        if node.text
    }
    required = {
        "../Headware/ps2_usart5.c",
        "../Headware/motor.c",
        "../Headware/encoder.c",
        "../Headware/servo.c",
        "../Robot/chassis_task.c",
        "../Robot/arm_control.c",
        "../System/pid.c",
    }
    require(required <= paths, "Keil project is missing a business source")
    forbidden = ("/app/", "/bsp/", "/com/", "remote_task", "ros_comm", "app_key")
    require(not any(any(item in path.lower() for item in forbidden) for path in paths),
            "Keil project still links a legacy business source")


def check_ps2() -> None:
    header = text("Headware/ps2_usart.h")
    source = text("Headware/ps2_usart5.c")
    chassis = text("Robot/chassis_task.c")
    arm = text("Robot/arm_control.c")
    config = text("Robot/robot_config.h")

    expected_codes = {
        "KEY_X": 0x8F,
        "KEY_B": 0x2F,
        "KEY_L1": 0x01,
        "KEY_L2": 0x04,
        "KEY_R1": 0x02,
        "KEY_R2": 0x08,
        "KEY_SELECTION": 0x10,
    }
    for name, expected in expected_codes.items():
        require(uint_macro(header, name) == expected, f"wrong CH559 code for {name}")

    shoulder_mask = uint_macro(header, "PS2_SHOULDER_MASK")
    shoulder_codes = (uint_macro(header, "KEY_L1") |
                      uint_macro(header, "KEY_R1") |
                      uint_macro(header, "KEY_L2") |
                      uint_macro(header, "KEY_R2"))
    require(shoulder_mask == shoulder_codes == 0x0F,
            "x7 shoulder mask does not preserve all four buttons")
    require((shoulder_mask & uint_macro(header, "KEY_SELECTION")) == 0,
            "SELECT overlaps a shoulder bit")
    require("shoulder_buttons" in header and "dpad_buttons" in header,
            "PS2 button bit masks are missing from the decoded report")
    require("top_key" not in header, "legacy single top_key decoder remains")

    require("HUB0_Joystick data: x%02X" in source, "CH559 frame format changed")
    copy_pos = source.index("memcpy(uart5_rx_buf")
    tick_pos = source.index("uart5_rx_frame_tick = HAL_GetTick()", copy_pos)
    ready_pos = source.index("uart5_rx_finish = 1U", tick_pos)
    require(copy_pos < tick_pos < ready_pos, "UART line/timestamp publication order is unsafe")
    require("ps2_last_frame_tick = uart5_rx_frame_tick" in source,
            "parser renews a buffered frame instead of using receive time")
    require("ps2.shoulder_buttons = (uint8_t)(x7 & PS2_SHOULDER_MASK)" in source,
            "SELECT+shoulder combinations are not preserved")
    require("ps2.select = ((x7 & KEY_SELECTION) != 0U)" in source,
            "SELECT is not decoded as an x7 bit")
    require("switch (x7)" not in source, "legacy single-value x7 switch remains")
    require("ps2.dpad_buttons |= PS2_DPAD_UP" in source and
            "ps2.dpad_buttons |= PS2_DPAD_RIGHT" in source,
            "D-pad directions are not retained as a bit mask")
    require("PS2_LINK_TIMEOUT_MS" in chassis and uint_macro(config, "PS2_LINK_TIMEOUT_MS") == 300,
            "PS2 300 ms failsafe is missing")
    require("controller->right_key == 3U" in chassis and "controller->right_key == 4U" in chassis,
            "reference x6 left/right mapping is missing")
    require(re.search(
        r"chassis\.vy\s*=\s*normalized_axis\(controller->lx\)\s*\*\s*"
        r"PS2_LEFT_X_TO_LEFT_SIGN\s*\*\s*CHASSIS_MAX_VY_MPS",
        chassis, re.S) is not None,
            "left-stick X is not mapped to continuous lateral translation")
    require(re.search(
        r"controller->right_key == 3U.*?chassis\.vz\s*=\s*"
        r"PS2_RIGHT_CONTROL_TO_CCW_SIGN\s*\*\s*CHASSIS_MAX_WZ_RADPS",
        chassis, re.S) is not None and
            re.search(
                r"controller->right_key == 4U.*?chassis\.vz\s*=\s*-"
                r"PS2_RIGHT_CONTROL_TO_CCW_SIGN\s*\*\s*CHASSIS_MAX_WZ_RADPS",
                chassis, re.S) is not None,
            "right-control left/right is not mapped to yaw")
    require("PS2_LEFT_X_TO_CCW_SIGN" not in chassis and
            "PS2_RIGHT_CONTROL_TO_LEFT_SIGN" not in chassis,
            "legacy turn/strafe axis mapping remains")
    require(re.search(
        r"arm_control_handle_input\s*\(\s*ps2\.select\s*,\s*"
        r"ps2\.shoulder_buttons\s*,\s*ps2\.dpad_buttons\s*,\s*"
        r"ps2\.right_key\s*,\s*inputs_neutral\s*,\s*"
        r"new_ps2_frame\s*\)", chassis, re.S) is not None,
            "SELECT arm input is not connected to the control task")
    require("s_drive_rearm_required" in chassis and "drive_inputs_are_neutral" in chassis,
            "post-SELECT neutral interlock is missing")
    require(re.search(r"if\s*\(ps2\.select != 0U\)\s*\{\s*"
                      r"chassis_force_stop\(\)", chassis, re.S) is not None,
            "SELECT does not force an immediate chassis stop")
    require("(new_ps2_frame != 0U) && (inputs_neutral != 0U)" in chassis,
            "drive can re-arm without a fresh neutral frame")
    timeout_pos = chassis.index("if (remote_stopped_flag != 0U)")
    reset_pos = chassis.index("arm_control_reset_input();", timeout_pos)
    hold_pos = chassis.index("arm_control_hold();", reset_pos)
    require(timeout_pos < reset_pos < hold_pos,
            "timeout must cancel SELECT gestures before holding the arm")

    require("big_cw = ((shoulder_buttons & KEY_L1)" in arm and
            "big_ccw = ((shoulder_buttons & KEY_R1)" in arm and
            "small_cw = ((shoulder_buttons & KEY_L2)" in arm and
            "small_ccw = ((shoulder_buttons & KEY_R2)" in arm,
            "manual shoulder mapping is incomplete")
    require(re.search(r"new_frame != 0U.*?big_cw != 0U.*?"
                      r"arm_control_jog_big_arm\(1\)", arm, re.S) is not None and
            re.search(r"big_ccw != 0U.*?arm_control_jog_big_arm\(-1\)",
                      arm, re.S) is not None and
            re.search(r"new_frame != 0U.*?small_cw != 0U.*?"
                      r"arm_control_jog_small_arm\(1\)", arm, re.S) is not None and
            re.search(r"small_ccw != 0U.*?arm_control_jog_small_arm\(-1\)",
                      arm, re.S) is not None,
            "manual clockwise/counter-clockwise mapping is incomplete")
    preset_cases = {
        "PS2_DPAD_UP": "ARM_PRESET_L1",
        "PS2_DPAD_DOWN": "ARM_PRESET_L2",
        "PS2_DPAD_LEFT": "ARM_PRESET_R1",
        "PS2_DPAD_RIGHT": "ARM_PRESET_R2",
    }
    for dpad, preset in preset_cases.items():
        require(re.search(
            rf"case\s+{dpad}\s*:\s*arm_control_select_preset\({preset}\)",
            arm) is not None,
                f"wrong preset mapping for {dpad}")
    require(re.search(r"right_key == 3U.*?arm_control_jog_gripper\(2U, -1\)",
                      arm, re.S) is not None and
            re.search(r"right_key == 4U.*?arm_control_jog_gripper\(2U, 1\)",
                      arm, re.S) is not None,
            "SELECT+X/B continuous Servo 3 mapping is missing")
    require(re.search(r"right_key == 1U.*?arm_control_jog_gripper\(3U, -1\)",
                      arm, re.S) is not None and
            re.search(r"right_key == 2U.*?arm_control_jog_gripper\(3U, 1\)",
                      arm, re.S) is not None,
            "SELECT+Y/A continuous Servo 4 mapping is missing")
    require("ARM_SELECT_TAP_MAX_MS" not in config and
            "s_select_chord_used" not in arm,
            "obsolete SELECT-only close gesture remains")
    require("active_categories != 1U" in arm and "exactly_one" in arm,
            "conflicting arm/D-pad combinations are not rejected")


def check_servo_and_timers() -> None:
    servo = text("Headware/servo.c")
    gpio = text("Src/gpio.c")
    tim = text("Src/tim.c")
    ioc = text("1_template_led.ioc")

    require("CLEAR_BIT(htim5.Instance->CCER, TIM_CCER_CC3E)" in servo,
            "TIM5_CH3 output gate is not forced off")
    require("__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC3)" in servo,
            "TIM5 CC3 internal interrupt is missing")
    require("CLEAR_BIT(htim5.Instance->CCER, TIM_CCER_CC4E)" in servo and
            "__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC4)" in servo,
            "TIM5 CC4 internal interrupt is missing or its PA3 gate is enabled")
    require(re.search(r"^\s*HAL_TIM_OC_Start_IT\s*\(", servo, re.M) is None,
            "HAL_TIM_OC_Start_IT would conflict with USART2_TX on PA2")
    require("counter < s_servo3_active_pulse_us" in servo,
            "late TIM5 frame protection is missing")
    require("GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_8" in gpio and
            "GPIO_PIN_9|GPIO_PIN_0" in gpio,
            "PC5/PB0 servo outputs are not driven low during early initialization")
    require("HAL_NVIC_SetPriority(TIM5_IRQn, 4, 0)" in tim,
            "TIM5 must pre-empt the floating-point TIM6 PID")
    require("HAL_NVIC_SetPriority(TIM6_IRQn, 5, 0)" in tim,
            "unexpected TIM6 priority")
    require("PA2.Signal=USART2_TX" in ioc and "PA0-WKUP.Signal=S_TIM5_CH1" in ioc,
            "CubeMX timer/UART pin allocation is inconsistent")
    require("PC5.Signal=GPIO_Output" in ioc and
            "PB0.Signal=GPIO_Output" in ioc and "UART4" not in ioc,
            "CubeMX still contains an obsolete or missing board resource")


def check_arm_config() -> None:
    config = text("Robot/robot_config.h")
    require(uint_macro(config, "ARM_SERVO_COUNT") == 4,
            "v1.4 must configure four arm servos")
    for servo in range(1, 5):
        minimum = uint_macro(config, f"ARM_SERVO{servo}_MIN_DEG")
        maximum = uint_macro(config, f"ARM_SERVO{servo}_MAX_DEG")
        require(0 <= minimum <= maximum <= 180, f"unsafe Servo {servo} limits")
        startup = uint_macro(config, f"ARM_START_SERVO{servo}_DEG")
        require(minimum <= startup <= maximum,
                f"Servo {servo} startup angle is outside limits")
        for key in ("L1", "L2", "R1", "R2"):
            angle = uint_macro(config, f"ARM_POSE_{key}_SERVO{servo}_DEG")
            require(minimum <= angle <= maximum, f"{key} Servo {servo} is outside limits")

    jog_step = uint_macro(config, "ARM_MANUAL_JOG_STEP_DEG")
    require(1 <= jog_step <= 10, "manual arm jog step is unreasonable")
    require(uint_macro(config, "ARM_BIG_ARM_CW_SIGN") in (-1, 1),
            "big-arm CW sign must be +/-1")
    require(uint_macro(config, "ARM_SMALL_ARM_CW_SIGN") in (-1, 1),
            "small-arm CW sign must be +/-1")
    require(uint_macro(config, "ARM_SMALL_ARM_CW_SIGN") == 1,
            "L2/R2 small-arm direction swap is not configured")
    servo3_min = uint_macro(config, "ARM_SERVO3_MIN_DEG")
    servo3_max = uint_macro(config, "ARM_SERVO3_MAX_DEG")
    gripper_open = uint_macro(config, "ARM_GRIPPER_OPEN_DEG")
    gripper_close = uint_macro(config, "ARM_GRIPPER_CLOSE_DEG")
    require(servo3_min <= gripper_open <= servo3_max and
            servo3_min <= gripper_close <= servo3_max and
            gripper_open != gripper_close,
            "gripper targets must be distinct and inside Servo 3 limits")


def check_mecanum_signs() -> None:
    config = text("Robot/robot_config.h")
    motor = text("Headware/motor.c")
    chassis = text("Robot/chassis_task.c")
    pid = text("System/pid.c")
    output_dirs = tuple(uint_macro(config, f"MOTOR_OUTPUT_DIR_{i}") for i in range(4))
    encoder_dirs = tuple(uint_macro(config, f"MOTOR_ENCODER_DIR_{i}") for i in range(4))
    require(output_dirs == (1, 1, -1, -1),
            "RL/FR motor output reversal is not configured")
    require(encoder_dirs == (1, 1, -1, -1),
            "front-wheel encoder feedback signs are incorrect")
    require("void motor_short_brake(uint8_t index)" in motor and
            "__HAL_TIM_GET_AUTORELOAD" in motor and
            "GPIO_PIN_SET" in motor,
            "TB6612 short-brake implementation is missing")
    require("chassis.motor1_pid[i].out = 0.0f" in chassis and
            "chassis.motor1_pid[i].integral = 0.0f" in chassis and
            "motor_ctrol(&chassis);" in chassis,
            "immediate stop does not clear PID state and command hardware brake")
    require("pid->out > pid->max_out" in pid and
            "pid->out < -pid->max_out" in pid and
            "PID_INC_MAX_OUT" not in pid,
            "MOTOR_PID_MAX_OUT is not the active incremental-PID output limit")

    mixer_lines = (
        "wheel[0] = chassis.vx - chassis.vy - wz_term;",
        "wheel[3] = chassis.vx + chassis.vy + wz_term;",
        "wheel[2] = chassis.vx + chassis.vy - wz_term;",
        "wheel[1] = chassis.vx - chassis.vy + wz_term;",
    )
    require(all(line in chassis for line in mixer_lines),
            "firmware mecanum mixer no longer matches the verified X-roller equations")

    # Motor array order after physical rewiring: 0=FL, 1=RR, 2=RL, 3=FR.
    def wheels(vx: float, vy: float, wz_term: float) -> tuple[int, int, int, int]:
        motor = [0.0] * 4
        motor[0] = vx - vy - wz_term
        motor[3] = vx + vy + wz_term
        motor[2] = vx + vy - wz_term
        motor[1] = vx - vy + wz_term
        # Return physical order FL, FR, RL, RR as signs.
        return tuple(1 if value > 0 else -1 if value < 0 else 0
                     for value in (motor[0], motor[3], motor[2], motor[1]))

    require(wheels(1, 0, 0) == (1, 1, 1, 1), "forward wheel signs changed")
    require(wheels(-1, 0, 0) == (-1, -1, -1, -1), "reverse targets are not equal")
    require(wheels(0, 1, 0) == (-1, 1, 1, -1), "left-strafe wheel signs changed")
    require(wheels(0, 0, 1) == (-1, 1, -1, 1), "CCW wheel signs changed")
    require(wheels(1, 1, 0) == (0, 1, 1, 0), "upper-left diagonal signs changed")
    require(wheels(-1, -1, 0) == (0, -1, -1, 0), "lower-right diagonal signs changed")


def main() -> int:
    check_keil_project()
    check_ps2()
    check_servo_and_timers()
    check_arm_config()
    check_mecanum_signs()
    print("PASS: v1.4 PS2 chords, 2-D left-stick translation, arm limits/poses and mecanum signs")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, ET.ParseError, OSError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
