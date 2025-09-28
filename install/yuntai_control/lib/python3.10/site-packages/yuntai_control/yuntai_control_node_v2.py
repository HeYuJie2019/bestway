"""
云台水平舵机控制（仅 pigpio 版）

此版本去掉 Jetson.GPIO 软件 PWM，只保留 pigpio 后端，使用 DMA 精准脉宽，适合较长信号线与降低抖动场景。

前置要求：已安装 pigpio 并启动 pigpiod 守护进程。
    安装 (apt)：
            sudo apt update && sudo apt install -y pigpio python3-pigpio
            sudo systemctl enable --now pigpiod
    或源码安装后自行创建 systemd 服务，临时测试也可直接：
            sudo pigpiod -l &

硬件建议（依旧重要）：
 1. 舵机与主控共地，舵机单独稳定 5V 供电，电源端加 470~1000µF 电解 + 0.1µF 去耦。
 2. 信号线长时（>30cm）串 100~220Ω，必要时加 74HCT14 缓冲。
 3. 尽量采用绞合或屏蔽（信号 + GND）。

话题 '/servo_position' (std_msgs/String) 支持：
    - "<pos>" 例如 "40"   (软件范围 soft_min~soft_max, 默认 -90~90)
    - "0:<pos>" 历史兼容格式

ROS2 参数：
    pin              (int)    BCM 引脚号 (默认 18, 常见硬件 PWM 引脚 18/19)
    min_pulse_us     (double) 默认 500
    max_pulse_us     (double) 默认 2500
    soft_min         (double) 默认 -90
    soft_max         (double) 默认 90
    resend_period_ms (int)    周期重发 (默认 1000, 0 关闭)
    ramp_deg_per_sec (double) 限制最大角速度 (0=不限制)
    ramp_timer_hz    (double) 平滑/重发定时频率 默认 50

运行示例：
    ros2 run yuntai_control yuntai_horizontal_servo_node \
         --ros-args -p pin:=18 -p ramp_deg_per_sec:=180 -p resend_period_ms:=500
"""

import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import pigpio  # type: ignore
except Exception as e:  # 直接在导入阶段给出明确提示
    print("未找到 pigpio 库，请先: sudo apt install -y pigpio python3-pigpio", file=sys.stderr)
    raise


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PigpioServo:
    """pigpio 舵机后端 (仅保留这一种)。"""

    def __init__(self, bcm_pin: int, min_pulse_us: float, max_pulse_us: float):
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpiod 未运行，请先: sudo systemctl start pigpiod 或 sudo pigpiod -l &")
        self.pin = bcm_pin
        self.min_pulse = float(min_pulse_us)
        self.max_pulse = float(max_pulse_us)
        if self.max_pulse <= self.min_pulse:
            raise ValueError("max_pulse_us 必须大于 min_pulse_us")
        self._last_pw = 0.0

    def _angle_to_pulse(self, angle_deg: float) -> float:
        angle = _clamp(angle_deg, 0.0, 180.0)
        return self.min_pulse + (self.max_pulse - self.min_pulse) * (angle / 180.0)

    def set_angle(self, angle_deg: float):
        pw = self._angle_to_pulse(angle_deg)
        if abs(pw - self._last_pw) > 0.5:  # 0.5us deadband
            self._pi.set_servo_pulsewidth(self.pin, pw)
            self._last_pw = pw

    def resend(self):
        if self._last_pw > 0:
            self._pi.set_servo_pulsewidth(self.pin, self._last_pw)

    def cleanup(self):
        try:
            self._pi.set_servo_pulsewidth(self.pin, 0)
        except Exception:
            pass
        try:
            self._pi.stop()
        except Exception:
            pass


class YuntaiHorizontalNode(Node):
    """云台水平舵机控制节点 (仅 pigpio)"""

    def __init__(self):
        super().__init__('yuntai_horizontal_servo_node')
        # 参数声明（精简）
        self.declare_parameter('pin', 18)
        self.declare_parameter('min_pulse_us', 500.0)
        self.declare_parameter('max_pulse_us', 2500.0)
        self.declare_parameter('soft_min', -90.0)
        self.declare_parameter('soft_max', 90.0)
        self.declare_parameter('resend_period_ms', 1000)
        self.declare_parameter('ramp_deg_per_sec', 0.0)
        self.declare_parameter('ramp_timer_hz', 50.0)

        pin = self.get_parameter('pin').get_parameter_value().integer_value
        min_pulse = self.get_parameter('min_pulse_us').get_parameter_value().double_value
        max_pulse = self.get_parameter('max_pulse_us').get_parameter_value().double_value
        self._soft_min = self.get_parameter('soft_min').get_parameter_value().double_value
        self._soft_max = self.get_parameter('soft_max').get_parameter_value().double_value
        self._resend_period_ms = int(self.get_parameter('resend_period_ms').get_parameter_value().integer_value)
        self._ramp_deg_per_sec = self.get_parameter('ramp_deg_per_sec').get_parameter_value().double_value
        self._ramp_timer_hz = self.get_parameter('ramp_timer_hz').get_parameter_value().double_value
        if self._ramp_timer_hz <= 0:
            self._ramp_timer_hz = 50.0

        self._angle_min = 0.0
        self._angle_max = 180.0
        self._center_deg = 90.0

        # 初始化 pigpio 后端
        try:
            self._servo = PigpioServo(bcm_pin=pin, min_pulse_us=min_pulse, max_pulse_us=max_pulse)
        except Exception as e:
            self.get_logger().error(f"初始化 pigpio 失败: {e}")
            raise

        self._current_angle = self._center_deg
        self._target_angle = self._center_deg
        self._servo.set_angle(self._center_deg)
        self._last_send_time = time.time()
        self._last_cmd_time = time.time()

        self._sub = self.create_subscription(String, '/servo_position', self._on_msg, 10)
        self._dt = 1.0 / self._ramp_timer_hz
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f"Yuntai horizontal servo (pigpio) started BCM pin={pin} soft=[{self._soft_min},{self._soft_max}] ramp={self._ramp_deg_per_sec}deg/s resend={self._resend_period_ms}ms"
        )

    def _soft_to_angle(self, position: float) -> float:
        # 线性映射: [soft_min, soft_max] -> [0, 180]
        p = _clamp(position, self._soft_min, self._soft_max)
        span_soft = (self._soft_max - self._soft_min)
        if span_soft <= 0:
            return self._center_deg
        ratio = (p - self._soft_min) / span_soft
        angle = self._angle_min + ratio * (self._angle_max - self._angle_min)
        return _clamp(angle, 0.0, 180.0)

    def _on_msg(self, msg: String) -> None:
        s = msg.data.strip()
        try:
            if ':' in s:
                idx_str, pos_str = s.split(':', 1)
                idx = int(idx_str)
                if idx != 0:
                    self.get_logger().warn(f"忽略非水平轴索引 idx={idx}，仅支持 0")
                    return
                position = float(pos_str)
            else:
                position = float(s)
            target_angle = self._soft_to_angle(position)
            self._target_angle = target_angle
            self._last_cmd_time = time.time()
            # 若未启用 ramp 直接设定
            if self._ramp_deg_per_sec <= 0:
                self._current_angle = target_angle
                self._servo.set_angle(self._current_angle)
                self._last_send_time = time.time()
            self.get_logger().debug(f"cmd position={position:.2f} -> target_angle={target_angle:.2f}")
        except Exception as e:
            self.get_logger().error(f"消息解析失败: '{s}', 错误: {e}")

    def _on_timer(self):
        # ramp + 周期重发
        now = time.time()
        need_send = False
        if self._ramp_deg_per_sec > 0:
            max_step = self._ramp_deg_per_sec * self._dt
            delta = self._target_angle - self._current_angle
            if abs(delta) > 0.01:
                if abs(delta) <= max_step:
                    self._current_angle = self._target_angle
                else:
                    self._current_angle += max_step if delta > 0 else -max_step
                need_send = True
        # 重发
        if self._resend_period_ms > 0 and (now - self._last_send_time) * 1000.0 >= self._resend_period_ms:
            need_send = True
        if need_send:
            self._servo.set_angle(self._current_angle)
            self._last_send_time = now

    def destroy_node(self) -> bool:
        try:
            self._servo.cleanup()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = YuntaiHorizontalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Yuntai horizontal servo node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()