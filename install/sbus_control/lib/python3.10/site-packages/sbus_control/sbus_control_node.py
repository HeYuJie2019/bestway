#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import threading
from typing import List, Sequence, Tuple, Optional

# ======================= WS2812 SPI (内联版) 开始 =======================
# 该段为 src/ws2812_blink_spi.py 的核心逻辑，做了少量改造以适应在节点中后台运行：
# - 去掉 argparse/命令行主程序，改为类 LedBlinker 可启动/停止
# - 发生异常时仅记录日志，不退出进程
# - 保持与原脚本一致的编码与时序参数

try:
    import spidev  # type: ignore
    _HAS_SPI = True
except Exception as _e:  # noqa: N816
    spidev = None  # type: ignore
    _HAS_SPI = False

# GPIO 支持（优先 Jetson.GPIO，其次 RPi.GPIO）
try:
    import Jetson.GPIO as GPIO  # type: ignore
    _HAS_GPIO = True
except Exception:
    try:
        import RPi.GPIO as GPIO  # type: ignore
        _HAS_GPIO = True
    except Exception:
        GPIO = None  # type: ignore
        _HAS_GPIO = False

Pixel = Tuple[int, int, int]  # (R, G, B)


def _build_lut() -> List[bytes]:
    lut: List[bytes] = [b"\x00\x00\x00"] * 256
    for val in range(256):
        bits = []
        for i in range(7, -1, -1):
            bit = (val >> i) & 1
            if bit:
                bits.extend((1, 1, 0))  # 1 -> 110
            else:
                bits.extend((1, 0, 0))  # 0 -> 100
        b0 = (
            (bits[0] << 7)
            | (bits[1] << 6)
            | (bits[2] << 5)
            | (bits[3] << 4)
            | (bits[4] << 3)
            | (bits[5] << 2)
            | (bits[6] << 1)
            | bits[7]
        )
        b1 = (
            (bits[8] << 7)
            | (bits[9] << 6)
            | (bits[10] << 5)
            | (bits[11] << 4)
            | (bits[12] << 3)
            | (bits[13] << 2)
            | (bits[14] << 1)
            | bits[15]
        )
        b2 = (
            (bits[16] << 7)
            | (bits[17] << 6)
            | (bits[18] << 5)
            | (bits[19] << 4)
            | (bits[20] << 3)
            | (bits[21] << 2)
            | (bits[22] << 1)
            | bits[23]
        )
        lut[val] = bytes((b0, b1, b2))
    return lut


_LUT = _build_lut()


def _apply_brightness(color: Pixel, brightness: int) -> Pixel:
    b = max(0, min(255, int(brightness)))
    if b >= 255:
        return color
    r, g, bl = color
    return (r * b // 255, g * b // 255, bl * b // 255)


def _order_channels(color: Pixel, order: str) -> Tuple[int, int, int]:
    r, g, b = color
    mapping = {
        "GRB": (g, r, b),
        "RGB": (r, g, b),
        "BGR": (b, g, r),
        "RBG": (r, b, g),
        "GBR": (g, b, r),
        "BRG": (b, r, g),
    }
    return mapping.get(order, (g, r, b))


def _encode_pixels(pixels: Sequence[Pixel], order: str, brightness: int) -> bytes:
    out = bytearray()
    for c in pixels:
        c2 = _apply_brightness(c, brightness)
        ch = _order_channels(c2, order)
        for byte in ch:  # type: ignore[assignment]
            out += _LUT[byte]
    out += b"\x00" * 32  # reset > 50us
    return bytes(out)


def _set_all(count: int, color: Pixel) -> List[Pixel]:
    return [color] * count


def _build_block_pattern(count: int, color_a: Pixel, color_b: Pixel, block_size: int = 8, phase: int = 0) -> List[Pixel]:
    bs = max(1, int(block_size))
    out: List[Pixel] = []
    toggle = phase & 1
    i = 0
    while i < count:
        color = color_b if toggle else color_a
        run = min(bs, count - i)
        out.extend([color] * run)
        i += run
        toggle ^= 1
    return out


class LedBlinker:
    """
    后台线程驱动 WS2812 闪烁（红/蓝按块交替）。
    与原脚本一致的默认参数，可通过构造时覆盖。
    """

    def __init__(
        self,
        logger,
        count: int = 128,
        bus: int = 0,
        device: int = 0,
        speed: int = 2_400_000,
        brightness: int = 64,
        interval: float = 0.1,
        order: str = "GRB",
        block_size: int = 8,
        color_a: Pixel = (255, 0, 0),
        color_b: Pixel = (0, 0, 255),
    ) -> None:
        self._logger = logger
        self.count = int(count)
        self.bus = int(bus)
        self.device = int(device)
        self.speed = int(speed)
        self.brightness = int(brightness)
        self.interval = float(interval)
        self.order = str(order)
        self.block_size = int(block_size)
        self.color_a = color_a
        self.color_b = color_b

        # 后台线程句柄
        self._thread = None  # type: Optional[threading.Thread]
        self._stop_evt = threading.Event()
        self._running = False
        self._spi = None

        if not _HAS_SPI:
            self._logger.warning("spidev 未安装，WS2812 功能将被禁用。请 pip install spidev 并确保有权限访问 /dev/spidev*")

    def _open_spi(self):
        if not _HAS_SPI:
            return None
        try:
            spi = spidev.SpiDev()  # type: ignore[attr-defined]
            spi.open(self.bus, self.device)
            spi.mode = 0
            spi.max_speed_hz = self.speed
            return spi
        except PermissionError:
            self._logger.error(
                "打开 SPI 失败（权限不足）。可尝试 sudo 运行或将用户加入相应组后重登。"
            )
        except Exception as e:
            self._logger.error(f"打开 SPI 失败: {e}")
        return None

    def _show_frame(self, frame: bytes) -> None:
        if self._spi is not None:
            try:
                self._spi.writebytes2(frame)
            except Exception as e:
                self._logger.error(f"SPI 发送失败: {e}")

    def _clear(self):
        off: Pixel = (0, 0, 0)
        frame = _encode_pixels(_set_all(self.count, off), self.order, 0)
        self._show_frame(frame)

    def _run(self):
        self._spi = self._open_spi()
        if self._spi is None:
            self._running = False
            return

        try:
            # 预构建两个相位的帧，提高刷新效率
            pixels_p0 = _build_block_pattern(self.count, self.color_a, self.color_b, self.block_size, phase=0)
            pixels_p1 = _build_block_pattern(self.count, self.color_a, self.color_b, self.block_size, phase=1)
            frame_p0 = _encode_pixels(pixels_p0, self.order, self.brightness)
            frame_p1 = _encode_pixels(pixels_p1, self.order, self.brightness)

            while not self._stop_evt.is_set():
                self._show_frame(frame_p0)
                self._stop_evt.wait(max(0.01, self.interval))
                if self._stop_evt.is_set():
                    break
                self._show_frame(frame_p1)
                self._stop_evt.wait(max(0.01, self.interval))
        finally:
            try:
                # 清空关闭
                self._clear()
            except Exception:
                pass
            try:
                if self._spi is not None:
                    self._spi.close()
            finally:
                self._spi = None
                self._running = False

    def start(self):
        if not _HAS_SPI:
            return
        if self._running:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, name="WS2812Blink", daemon=True)
        self._running = True
        self._thread.start()
        self._logger.info("WS2812 闪烁已启动（BOSS 第8通道>500）")

    def stop(self):
        if not self._running:
            return
        self._stop_evt.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None
        self._running = False
        self._logger.info("WS2812 闪烁已停止（BOSS 第8通道<=500 或退出 BOSS）")

    @property
    def running(self) -> bool:
        return self._running

# ======================= WS2812 SPI (内联版) 结束 =======================

def calculate_xor(data):
    xor = 0
    for byte in data:
        xor ^= byte
    return xor

def create_sbus_frame(channels, flag=0x00):
    if len(channels) != 16:
        raise ValueError("需要 16 个通道值")
    channels = [min(max(ch, 0), 2047) for ch in channels]
    frame = [0x0F]
    for ch in channels:
        frame.append((ch >> 8) & 0xFF)
        frame.append(ch & 0xFF)
    frame.append(flag)
    xor = calculate_xor(frame[1:])
    frame.append(xor)
    return bytes(frame)

def parse_sbus_frame(data):
    """
    解析35字节SBUS数据帧，返回16通道、flag、校验
    """
    if len(data) != 35 or data[0] != 0x0F:
        return None, None, None

    # 16通道，每通道2字节，高字节在前
    channels = []
    for i in range(16):
        high = data[1 + i*2]
        low = data[2 + i*2]
        value = (high << 8) | low
        channels.append(value)

    flag = data[33]
    xor = data[34]
    # 校验
    calc_xor = 0
    for b in data[1:34]:
        calc_xor ^= b
    if xor != calc_xor:
        return None, None, None

    return channels, flag, xor

def map_channel(val, old_mid=1020, new_mid=1002):
    # 线性平移映射，保持比例
    return val - old_mid + new_mid

class SbusControlNode(Node):
    def __init__(self):
        super().__init__('sbus_control_node')

        # 获取参数
        self.declare_parameter('port', '/dev/ttyCH9344USB7')
        self.declare_parameter('baudrate', 115200)
        # WS2812 参数（可在启动时覆盖）
        self.declare_parameter('ws2812_count', 128)
        self.declare_parameter('ws2812_bus', 0)
        self.declare_parameter('ws2812_device', 0)
        self.declare_parameter('ws2812_speed', 2_400_000)
        self.declare_parameter('ws2812_brightness', 64)
        self.declare_parameter('ws2812_interval', 0.1)
        self.declare_parameter('ws2812_order', 'GRB')
        self.declare_parameter('ws2812_block_size', 8)
        self.declare_parameter('ws2812_color_a', '255,0,0')
        self.declare_parameter('ws2812_color_b', '0,0,255')
        # GPIO 参数
        self.declare_parameter('gpio_enable', True)
        self.declare_parameter('gpio_mode', 'BOARD')  # 可选 BOARD/BCM
        self.declare_parameter('gpio_pin', 7)  # 默认为物理 7 号脚（Jetson 标注 GPIO09）

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 初始化串口
        self.channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 282,
            282, 282, 1002, 1002, 300, 300, 300, 300
        ]
        self.default_channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 282,
            282, 282, 1002, 1002, 300, 300, 300, 300
        ]
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO
        )
        self.sbus_in = serial.Serial('/dev/ttyCH9344USB6', 115200, timeout=0.01)
        self.BOSS = None  # 默认不处于 BOSS 模式

        # WS2812 灯带控制器
        def _parse_color(s: str) -> Pixel:
            try:
                parts = [int(x.strip()) for x in s.split(',')]
                if len(parts) != 3:
                    raise ValueError
                r, g, b = [max(0, min(255, v)) for v in parts]
                return (r, g, b)
            except Exception:
                self.get_logger().warning(f"颜色参数无效：{s}，使用默认 255,0,0")
                return (255, 0, 0)

        ws_count = self.get_parameter('ws2812_count').value
        ws_bus = self.get_parameter('ws2812_bus').value
        ws_dev = self.get_parameter('ws2812_device').value
        ws_speed = self.get_parameter('ws2812_speed').value
        ws_bright = self.get_parameter('ws2812_brightness').value
        ws_interval = self.get_parameter('ws2812_interval').value
        ws_order = self.get_parameter('ws2812_order').value
        ws_block = self.get_parameter('ws2812_block_size').value
        ws_color_a = _parse_color(self.get_parameter('ws2812_color_a').value)
        ws_color_b = _parse_color(self.get_parameter('ws2812_color_b').value)

        self.led_blinker = LedBlinker(
            logger=self.get_logger(),
            count=ws_count,
            bus=ws_bus,
            device=ws_dev,
            speed=ws_speed,
            brightness=ws_bright,
            interval=ws_interval,
            order=ws_order,
            block_size=ws_block,
            color_a=ws_color_a,
            color_b=ws_color_b,
        )

        # GPIO 初始化
        self.gpio_enabled = bool(self.get_parameter('gpio_enable').value)
        self.gpio_pin = int(self.get_parameter('gpio_pin').value)
        self.gpio_mode = str(self.get_parameter('gpio_mode').value).upper()
        self._gpio_available = _HAS_GPIO and self.gpio_enabled
        self._gpio_state = None
        if self._gpio_available:
            try:
                if self.gpio_mode == 'BCM':
                    GPIO.setmode(GPIO.BCM)  # type: ignore[attr-defined]
                else:
                    GPIO.setmode(GPIO.BOARD)  # type: ignore[attr-defined]
                GPIO.setwarnings(False)  # type: ignore[attr-defined]
                GPIO.setup(self.gpio_pin, GPIO.OUT, initial=GPIO.LOW)  # type: ignore[attr-defined]
                self._gpio_state = False
                self.get_logger().info(f"GPIO 已初始化：mode={self.gpio_mode} pin={self.gpio_pin} 初始为低电平")
            except Exception as e:
                self._gpio_available = False
                self.get_logger().warning(f"GPIO 初始化失败，将禁用 GPIO 控制：{e}")

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 订阅 /boss 话题
        self.boss_subscription = self.create_subscription(
            Int32,
            '/boss',
            self.boss_callback,
            10
        )

        # 定时器，用于检测是否停止发布
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.check_timeout)  # 每 0.02 秒触发一次

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def boss_callback(self, msg):
        """
        接收BOSS状态消息
        """
        if msg.data == 0:
            self.BOSS = False
            self.get_logger().info("接收BOSS状态: 非BOSS模式")
            # 退出 BOSS 时停止 WS2812
            if self.led_blinker and self.led_blinker.running:
                self.led_blinker.stop()
            # 退出 BOSS 时 GPIO 拉低
            self._set_gpio(False)
        elif msg.data == 1:
            self.BOSS = True
            self.get_logger().info("接收BOSS状态: BOSS模式")
        else:
            self.BOSS = None
            self.get_logger().info(f"接收BOSS状态: 未知状态 ({msg.data})")
            if self.led_blinker and self.led_blinker.running:
                self.led_blinker.stop()
            self._set_gpio(False)

    def cmd_vel_callback(self, msg):
        """
        当接收到 /cmd_vel 消息时，更新 channels 数据
        """
        self.last_cmd_time = self.get_clock().now()  # 更新最后接收时间

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 将线速度映射到 channels[2] (前进/后退)
        if linear_x > 0:
            self.channels[2] = int(1002 - 100 * linear_x)
        elif linear_x < 0:
            self.channels[2] = int(1002 - 100 * linear_x)
        else:
            self.channels[2] = 1002

        # 将角速度映射到 channels[0] (左转/右转)
        if angular_z > 0:
            self.channels[0] = int(1002 - 100 * angular_z)
        elif angular_z < 0:
            self.channels[0] = int(1002 - 100 * angular_z)
        else:
            self.channels[0] = 1002

        # 发送 SBUS 数据帧
        self.send_sbus_frame()

    def check_timeout(self):
        """
        检查是否超时未接收到 /cmd_vel 消息
        如果超时，则发送默认的 channels 数据
        """
        now = self.get_clock().now()
        # 检查是否超时
        if (now - self.last_cmd_time).nanoseconds > 500_000_000:  # 超过 0.5 秒未接收到消息
            self.channels = self.default_channels.copy()
            self.send_sbus_frame()

    def send_sbus_frame(self):
        """
        发送 SBUS 数据帧
        """
        if self.BOSS is True:
            # 读取/dev/ttyCH9344USB6的数据并直接转发
            if self.sbus_in.in_waiting >= 35:
                # 读取前清空输入缓冲区，只处理最新一帧
                while self.sbus_in.in_waiting >= 35:
                    data = self.sbus_in.read(35)
                if 'data' in locals():
                    # 解析帧，监控第8通道
                    channels, flag, xor = parse_sbus_frame(data)
                    if channels is not None:
                        ch8 = channels[7]  # 第8通道（1-based -> index 7）
                        if ch8 > 500:
                            # 启动 WS2812 闪烁（若未启动）
                            if not self.led_blinker.running:
                                self.led_blinker.start()
                            # GPIO 拉高
                            self._set_gpio(True)
                        else:
                            # 停止 WS2812（若已启动）
                            if self.led_blinker.running:
                                self.led_blinker.stop()
                            # GPIO 拉低
                            self._set_gpio(False)
                    # 转发原始数据
                    self.serial_port.write(data)
                    # 调低日志等级避免刷屏
                    self.get_logger().debug(f"直接转发SBUS数据: {data.hex().upper()}")
        else:
            frame = create_sbus_frame(self.channels)
            self.serial_port.write(frame)
            self.get_logger().info(f"发送数据帧: {frame.hex().upper()}")

    def _set_gpio(self, high: bool):
        """设置 GPIO 引脚电平。仅在可用且启用时生效，避免重复写。"""
        if not getattr(self, '_gpio_available', False):
            return
        try:
            if self._gpio_state is None or self._gpio_state != bool(high):
                GPIO.output(self.gpio_pin, GPIO.HIGH if high else GPIO.LOW)  # type: ignore[attr-defined]
                self._gpio_state = bool(high)
        except Exception as e:
            self.get_logger().warning(f"设置 GPIO 失败：{e}")

    def destroy_node(self):
        # 清理 WS2812 线程
        try:
            if hasattr(self, 'led_blinker') and self.led_blinker.running:
                self.led_blinker.stop()
        finally:
            try:
                if hasattr(self, 'serial_port'):
                    self.serial_port.flush()
            except Exception:
                pass
            # 清理 GPIO：拉低并 cleanup
            try:
                if getattr(self, '_gpio_available', False):
                    try:
                        GPIO.output(self.gpio_pin, GPIO.LOW)  # type: ignore[attr-defined]
                    except Exception:
                        pass
                    try:
                        GPIO.cleanup(self.gpio_pin)  # type: ignore[attr-defined]
                    except Exception:
                        try:
                            GPIO.cleanup()  # type: ignore[attr-defined]
                        except Exception:
                            pass
            except Exception:
                pass
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SbusControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()