#!/usr/bin/env python3
"""
Jetson Orin NX 舵机 PWM 控制脚本

功能:
- 使用 Jetson.GPIO 在 40-pin 物理引脚 15 输出 50Hz 软件 PWM
- 通过角度 (0~180°) 控制舵机位置
- 支持扫动模式与脉宽/频率校准

注意:
- 舵机电源请使用独立 5V，地线与 Orin NX 共地；不要从板载 5V 给大舵机供电
- 软件 PWM 抖动较小，一般足以驱动常见舵机；如需更稳，可改用硬件 PWM 引脚 (如 32/33) 并开启硬件 PWM
- 运行脚本可能需要 sudo 权限，或将当前用户加入 gpio 组
"""

import argparse
import sys
import time

try:
    import Jetson.GPIO as GPIO
except Exception as e:  # pragma: no cover - 硬件环境外仅提示
    print("未找到 Jetson.GPIO，请先安装: sudo apt-get install python3-jetson-gpio 或 pip install Jetson.GPIO", file=sys.stderr)
    raise


def angle_to_duty(angle: float, min_pulse_us: float, max_pulse_us: float, freq_hz: float) -> float:
    """将角度映射为 PWM 占空比(%)。

    参数:
    - angle: 目标角度，0~180 之间(可稍超出以便校准测试)
    - min_pulse_us: 对应 0° 的脉宽(微秒)
    - max_pulse_us: 对应 180° 的脉宽(微秒)
    - freq_hz: PWM 频率(Hz)
    返回:
    - duty_cycle: 0~100(%)
    """
    period_us = 1_000_000.0 / freq_hz
    # 线性插值 (可接收轻微越界以便校准，后续 clamp)
    pulse_us = min_pulse_us + (max_pulse_us - min_pulse_us) * (angle / 180.0)
    # 限幅，避免无效占空比
    pulse_us = max(min_pulse_us, min(max_pulse_us, pulse_us))
    duty = (pulse_us / period_us) * 100.0
    return duty


def setup_gpio(pin: int):
    GPIO.setwarnings(False)
    # 使用物理引脚编号(BOARD)，pin=15 即 40-pin 头的第 15 号脚
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def main():
    parser = argparse.ArgumentParser(description="在 Jetson Orin NX 的物理引脚输出舵机 PWM")
    parser.add_argument("--pin", type=int, default=15, help="物理引脚号(BOARD 编号)，默认 15")
    parser.add_argument("--freq", type=float, default=50.0, help="PWM 频率，默认 50Hz")
    parser.add_argument("--min-pulse", type=float, default=500.0, help="0° 脉宽(微秒)，默认 500us")
    parser.add_argument("--max-pulse", type=float, default=2500.0, help="180° 脉宽(微秒)，默认 2500us")

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--angle", type=float, help="设置目标角度(0~180)，不随时间变化")
    group.add_argument("--sweep", action="store_true", help="在最小/最大角度间往返扫动")

    parser.add_argument("--min-angle", type=float, default=0.0, help="扫动/限制的最小角度，默认 0°")
    parser.add_argument("--max-angle", type=float, default=180.0, help="扫动/限制的最大角度，默认 180°")
    parser.add_argument("--step", type=float, default=2.0, help="扫动时每步角度增量，默认 2°")
    parser.add_argument("--dwell", type=float, default=0.02, help="每步停留时间(秒)，默认 0.02s")
    parser.add_argument("--hold", type=float, default=1.0, help="--angle 模式下保持时间(秒)，默认 1s；<=0 表示持续保持直到 Ctrl+C")

    args = parser.parse_args()

    pin = args.pin
    freq = float(args.freq)
    min_pulse = float(args.min_pulse)
    max_pulse = float(args.max_pulse)

    if not (0 < freq <= 400):
        print("频率建议 50Hz，且不超过 400Hz", file=sys.stderr)
        sys.exit(2)

    if max_pulse <= min_pulse:
        print("参数错误: --max-pulse 必须大于 --min-pulse", file=sys.stderr)
        sys.exit(2)

    if args.angle is None and not args.sweep:
        # 默认将舵机置中
        args.angle = 90.0

    setup_gpio(pin)

    pwm = None
    try:
        pwm = GPIO.PWM(pin, freq)
        pwm.start(0.0)

        if args.sweep:
            # 往返扫动
            angle = args.min_angle
            direction = 1.0
            while True:
                duty = angle_to_duty(angle, min_pulse, max_pulse, freq)
                pwm.ChangeDutyCycle(duty)
                time.sleep(args.dwell)

                angle += direction * args.step
                if angle >= args.max_angle:
                    angle = args.max_angle
                    direction = -1.0
                elif angle <= args.min_angle:
                    angle = args.min_angle
                    direction = 1.0
        else:
            # 固定角度保持
            target = float(args.angle)
            duty = angle_to_duty(target, min_pulse, max_pulse, freq)
            pwm.ChangeDutyCycle(duty)
            if args.hold and args.hold > 0:
                time.sleep(args.hold)
            else:
                # 持续保持，直至 Ctrl+C
                while True:
                    time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        # 将占空比归零以避免抖动，然后清理
        try:
            if pwm is not None:
                pwm.ChangeDutyCycle(0.0)
                time.sleep(0.05)
                pwm.stop()
        except Exception:
            pass
        GPIO.cleanup()


if __name__ == "__main__":
    main()
