"""
Jetson 硬件 PWM 版云台水平舵机节点
====================================

适用场景：在 Jetson Orin / Xavier / Nano 等平台上，已经通过 Jetson-IO 或设备树 overlay
启用了某个 40pin 引脚的 PWM 功能（例如 40pin: 32=PWM0, 33=PWM2 视型号而定），并希望用
硬件 PWM (sysfs) 而不是软件 PWM (Jetson.GPIO) 或外部驱动板。

原理：使用 /sys/class/pwm/pwmchipX/pwmY 提供的 period + duty_cycle 控制。
目标：输出 50Hz (period=20_000_000ns)；舵机脉宽 500~2500µs -> duty_cycle 500_000~2_500_000ns。

注意：
1. 首先启用 PWM：
   sudo /opt/nvidia/jetson-io/jetson-io.py  (选择启用需要的 PWM，引脚复用)
   重启后检查： ls /sys/class/pwm
2. 找到 pwmchip：
   ls -1 /sys/class/pwm | grep pwmchip
   进入其中： cd /sys/class/pwm/pwmchip0  (示例)
   cat npwm  # 查看包含多少通道
3. 导出通道 (假设使用 channel 0)：
   echo 0 | sudo tee /sys/class/pwm/pwmchip0/export
   之后会出现 /sys/class/pwm/pwmchip0/pwm0 目录
4. 设置频率：
   echo 0 | sudo tee /sys/class/pwm/pwmchip0/pwm0/enable
   echo 20000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/period
   echo 1500000  | sudo tee /sys/class/pwm/pwmchip0/pwm0/duty_cycle  # 中位
   echo 1 | sudo tee /sys/class/pwm/pwmchip0/pwm0/enable

本节点自动执行 export / 设置 period / duty / enable，可通过参数控制，支持：
  pwmchip_index        (int)   默认 0
  channel              (int)   默认 0
  period_ns            (int)   默认 20000000 (50Hz)
  min_pulse_us         (double) 500.0
  max_pulse_us         (double) 2500.0
  soft_min             (double) -90.0 输入软件范围下限
  soft_max             (double) 90.0  输入软件范围上限
  resend_period_ms     (int)    1000 周期重发 (写一次 duty)，0 关闭
  ramp_deg_per_sec     (double) 0    角速度限制，0=无
  ramp_timer_hz        (double) 50.0 定时器 (平滑 + 重发)
  auto_export          (bool)   True  若通道未导出则自动 echo export
  auto_unexport_on_exit(bool)  True  退出时 unexport

订阅话题： /servo_position (std_msgs/String)
  支持 "<pos>" 与 "0:<pos>" 形式

权限问题：默认 /sys/class/pwm 需要 root 写入。解决方案：
  1) 直接使用 sudo 运行节点 (简单但不推荐长期)。
  2) 添加 udev 规则：
     sudo tee /etc/udev/rules.d/99-pwm.rules >/dev/null <<'EOF'\nSUBSYSTEM=="pwm", MODE="0664", GROUP="gpio"\nEOF\n
     sudo groupadd -f gpio
     sudo usermod -aG gpio $USER
     重启或重新登录；确认 /sys/class/pwm/* 权限组为 gpio。

失败诊断：
  - 如果找不到 pwmchipX：说明 PWM 功能未启用或设备树缺失；重新用 jetson-io 配置。
  - EBUSY：其它程序占用（或未 disable 就修改 period）。先写 enable=0 再改 period。
  - Permission denied：权限问题，见上面解决方案。

"""

from __future__ import annotations
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SysFSPWMServo:
    """封装 sysfs PWM 接口 (单通道)"""

    def __init__(self, chip_index: int, channel: int, period_ns: int, min_pulse_us: float, max_pulse_us: float,
                 auto_export: bool = True, auto_unexport: bool = True, logger=None):
        self.chip_index = chip_index
        self.channel = channel
        self.period_ns = int(period_ns)
        self.min_pulse_us = float(min_pulse_us)
        self.max_pulse_us = float(max_pulse_us)
        self.auto_export = auto_export
        self.auto_unexport = auto_unexport
        self.logger = logger
        if self.max_pulse_us <= self.min_pulse_us:
            raise ValueError("max_pulse_us 必须大于 min_pulse_us")
        if self.period_ns <= 0:
            raise ValueError("period_ns 必须 > 0")

        self._chip_path = f"/sys/class/pwm/pwmchip{self.chip_index}"
        self._pwm_path = f"{self._chip_path}/pwm{self.channel}"
        self._export_if_needed()
        self._configure_period()
        self.enable(True)
        self._last_duty_ns = 0

    # ---- 文件写入帮助 ----
    def _write(self, path: str, value: str):
        try:
            with open(path, 'w') as f:
                f.write(value)
        except Exception as e:
            raise RuntimeError(f"写入 {path} 失败: {e}")

    def _read(self, path: str) -> str:
        with open(path, 'r') as f:
            return f.read().strip()

    def _export_if_needed(self):
        if not os.path.isdir(self._chip_path):
            raise RuntimeError(f"未找到 {self._chip_path}，PWM 功能可能未启用")
        if not os.path.isdir(self._pwm_path):
            if not self.auto_export:
                raise RuntimeError(f"通道未导出: {self._pwm_path} (auto_export=False)")
            self._write(f"{self._chip_path}/export", str(self.channel))
            # 等待 sysfs 创建
            for _ in range(50):
                if os.path.isdir(self._pwm_path):
                    break
                time.sleep(0.01)
            if not os.path.isdir(self._pwm_path):
                raise RuntimeError("export 后仍未出现 pwm 目录")

    def _configure_period(self):
        enable_path = f"{self._pwm_path}/enable"
        # 如果已启用，需要先 disable 才能改 period
        try:
            if os.path.exists(enable_path):
                self._write(enable_path, '0')
        except Exception:
            pass
        self._write(f"{self._pwm_path}/period", str(self.period_ns))
        # 给一个默认 duty (0)
        duty_path = f"{self._pwm_path}/duty_cycle"
        if os.path.exists(duty_path):
            self._write(duty_path, '0')

    def enable(self, en: bool):
        self._write(f"{self._pwm_path}/enable", '1' if en else '0')

    def set_angle(self, angle_deg: float):
        angle = _clamp(angle_deg, 0.0, 180.0)
        pulse_us = self.min_pulse_us + (self.max_pulse_us - self.min_pulse_us) * (angle / 180.0)
        duty_ns = int(pulse_us * 1000.0)
        # 保护：duty <= period
        duty_ns = min(duty_ns, self.period_ns - 1000)  # 留一点安全余量
        if abs(duty_ns - self._last_duty_ns) < 200:  # <200ns 变化忽略
            return
        self._write(f"{self._pwm_path}/duty_cycle", str(duty_ns))
        self._last_duty_ns = duty_ns

    def resend(self):
        if self._last_duty_ns > 0:
            self._write(f"{self._pwm_path}/duty_cycle", str(self._last_duty_ns))

    def cleanup(self):
        try:
            self.enable(False)
        except Exception:
            pass
        if self.auto_unexport:
            try:
                self._write(f"{self._chip_path}/unexport", str(self.channel))
            except Exception:
                pass


class YuntaiHardwarePWMNode(Node):
    """硬件 PWM 控制 (单水平轴)"""

    def __init__(self):
        super().__init__('yuntai_hardware_pwm_servo_node')
        # 参数
        self.declare_parameter('pwmchip_index', 0)
        self.declare_parameter('channel', 0)
        self.declare_parameter('period_ns', 20_000_000)  # 50Hz
        self.declare_parameter('min_pulse_us', 500.0)
        self.declare_parameter('max_pulse_us', 2500.0)
        self.declare_parameter('soft_min', -90.0)
        self.declare_parameter('soft_max', 90.0)
        self.declare_parameter('resend_period_ms', 1000)
        self.declare_parameter('ramp_deg_per_sec', 0.0)
        self.declare_parameter('ramp_timer_hz', 50.0)
        self.declare_parameter('auto_export', True)
        self.declare_parameter('auto_unexport_on_exit', True)

        chip = self.get_parameter('pwmchip_index').get_parameter_value().integer_value
        ch = self.get_parameter('channel').get_parameter_value().integer_value
        period_ns = self.get_parameter('period_ns').get_parameter_value().integer_value
        min_pulse = self.get_parameter('min_pulse_us').get_parameter_value().double_value
        max_pulse = self.get_parameter('max_pulse_us').get_parameter_value().double_value
        self._soft_min = self.get_parameter('soft_min').get_parameter_value().double_value
        self._soft_max = self.get_parameter('soft_max').get_parameter_value().double_value
        self._resend_period_ms = int(self.get_parameter('resend_period_ms').get_parameter_value().integer_value)
        self._ramp_deg_per_sec = self.get_parameter('ramp_deg_per_sec').get_parameter_value().double_value
        self._ramp_timer_hz = self.get_parameter('ramp_timer_hz').get_parameter_value().double_value
        self._auto_export = self.get_parameter('auto_export').get_parameter_value().bool_value
        self._auto_unexport = self.get_parameter('auto_unexport_on_exit').get_parameter_value().bool_value

        if self._ramp_timer_hz <= 0:
            self._ramp_timer_hz = 50.0
        self._dt = 1.0 / self._ramp_timer_hz

        self._angle_min = 0.0
        self._angle_max = 180.0
        self._center = 90.0
        # 初始化硬件 PWM
        try:
            self._pwm = SysFSPWMServo(chip_index=chip, channel=ch, period_ns=period_ns,
                                      min_pulse_us=min_pulse, max_pulse_us=max_pulse,
                                      auto_export=self._auto_export, auto_unexport=self._auto_unexport, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"初始化硬件 PWM 失败: {e}")
            raise

        self._current_angle = self._center
        self._target_angle = self._center
        self._pwm.set_angle(self._center)
        self._last_send_time = time.time()
        self._sub = self.create_subscription(String, '/servo_position', self._on_msg, 10)
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f"Hardware PWM started chip={chip} ch={ch} period={period_ns} soft=[{self._soft_min},{self._soft_max}] ramp={self._ramp_deg_per_sec} resend={self._resend_period_ms}ms"
        )

    def _soft_to_angle(self, position: float) -> float:
        p = _clamp(position, self._soft_min, self._soft_max)
        span = self._soft_max - self._soft_min
        if span <= 0:
            return self._center
        ratio = (p - self._soft_min) / span
        ang = self._angle_min + ratio * (self._angle_max - self._angle_min)
        return _clamp(ang, 0.0, 180.0)

    def _on_msg(self, msg: String):
        s = msg.data.strip()
        try:
            if ':' in s:
                idx_str, pos_str = s.split(':', 1)
                if int(idx_str) != 0:
                    self.get_logger().warn(f"忽略非水平轴 idx={idx_str}")
                    return
                position = float(pos_str)
            else:
                position = float(s)
            target = self._soft_to_angle(position)
            self._target_angle = target
            if self._ramp_deg_per_sec <= 0:
                self._current_angle = target
                self._pwm.set_angle(self._current_angle)
                self._last_send_time = time.time()
            self.get_logger().debug(f"cmd position={position:.2f} -> target={target:.2f}")
        except Exception as e:
            self.get_logger().error(f"解析失败 '{s}': {e}")

    def _on_timer(self):
        now = time.time()
        need = False
        if self._ramp_deg_per_sec > 0:
            max_step = self._ramp_deg_per_sec * self._dt
            delta = self._target_angle - self._current_angle
            if abs(delta) > 0.01:
                if abs(delta) <= max_step:
                    self._current_angle = self._target_angle
                else:
                    self._current_angle += max_step if delta > 0 else -max_step
                need = True
        if self._resend_period_ms > 0 and (now - self._last_send_time) * 1000.0 >= self._resend_period_ms:
            need = True
        if need:
            self._pwm.set_angle(self._current_angle)
            self._last_send_time = now

    def destroy_node(self) -> bool:
        try:
            self._pwm.cleanup()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = YuntaiHardwarePWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down hardware PWM servo node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
