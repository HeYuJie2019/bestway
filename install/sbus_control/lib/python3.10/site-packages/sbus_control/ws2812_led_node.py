#!/usr/bin/env python3
"""
ROS2 节点：通过 SPI 驱动 WS2812 (NeoPixel) 灯带

移植自独立脚本 `ws2812_blink_spi.py`，主要改动：
- 封装为 rclpy Node
- 使用 ROS 参数配置 bus/device/speed/count/brightness/block_size/color 等
- 提供话题接口切换模式：
  * 订阅 `/ws2812_mode` (std_msgs/String)
    - 内容: "blink" / "solid" / "off"
    - 若为 "solid:R,G,B" 可动态设置常亮颜色（例: solid:0,255,0）
- 可选发布当前状态到 `/ws2812_state` (后续可扩展)

注意：
- SPI 速率默认 2.4MHz；请确保内核已启用 /dev/spidevX.Y
- 上电/供电建议与原脚本一致：5V + 大电容，数据线串约 300-470Ω。
"""

from __future__ import annotations

import sys
import time
import threading
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import spidev  # type: ignore
except Exception as e:  # noqa: BLE001
    print("未找到 spidev，请先安装: pip install spidev", file=sys.stderr)
    raise

Pixel = Tuple[int, int, int]


def _clamp(val: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, val))


def build_lut() -> List[bytes]:
    lut: List[bytes] = [b"\x00\x00\x00"] * 256
    for val in range(256):
        bits = []
        for i in range(7, -1, -1):
            bit = (val >> i) & 1
            if bit:
                bits.extend((1, 1, 0))  # 1 -> 110
            else:
                bits.extend((1, 0, 0))  # 0 -> 100
        b0 = sum(bits[j] << (7 - j) for j in range(0, 8))
        b1 = sum(bits[j] << (7 - (j - 8)) for j in range(8, 16))
        b2 = sum(bits[j] << (7 - (j - 16)) for j in range(16, 24))
        lut[val] = bytes((b0, b1, b2))
    return lut


LUT = build_lut()


def apply_brightness(color: Pixel, brightness: int) -> Pixel:
    b = _clamp(brightness, 0, 255)
    if b >= 255:
        return color
    r, g, bl = color
    return (r * b // 255, g * b // 255, bl * b // 255)


def order_channels(color: Pixel, order: str) -> Pixel:
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


def encode_pixels(pixels: Sequence[Pixel], order: str, brightness: int) -> bytes:
    out = bytearray()
    for c in pixels:
        c2 = apply_brightness(c, brightness)
        ch = order_channels(c2, order)
        for byte in ch:  # type: ignore
            out += LUT[byte]
    out += b"\x00" * 32  # reset > 50us
    return bytes(out)


def build_block_pattern(count: int, color_a: Pixel, color_b: Pixel, block_size: int, phase: int) -> list[Pixel]:
    bs = max(1, int(block_size))
    out: list[Pixel] = []
    toggle = phase & 1
    i = 0
    while i < count:
        color = color_b if toggle else color_a
        run = min(bs, count - i)
        out.extend([color] * run)
        i += run
        toggle ^= 1
    return out


class WS2812Node(Node):
    def __init__(self) -> None:
        super().__init__('ws2812_led_node')

        # 声明参数（可通过 ros2 param set 修改）
        self.declare_parameter('count', 128)
        self.declare_parameter('bus', 0)
        self.declare_parameter('device', 0)
        self.declare_parameter('speed', 2_400_000)
        self.declare_parameter('brightness', 64)
        self.declare_parameter('order', 'GRB')
        self.declare_parameter('block_size', 8)
        self.declare_parameter('color', '255,0,0')  # solid 模式默认
        self.declare_parameter('color2', '0,0,255')  # blink 第二色
        self.declare_parameter('mode', 'blink')  # 初始模式

        self._mode = self.get_parameter('mode').get_parameter_value().string_value or 'blink'

        self._count = int(self.get_parameter('count').value)
        self._order = self.get_parameter('order').value
        self._brightness = int(self.get_parameter('brightness').value)
        self._block_size = int(self.get_parameter('block_size').value)

        bus = int(self.get_parameter('bus').value)
        dev = int(self.get_parameter('device').value)
        speed = int(self.get_parameter('speed').value)

        self._color_solid = self._parse_color(self.get_parameter('color').value)
        self._color2 = self._parse_color(self.get_parameter('color2').value)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # 打开 SPI
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(bus, dev)
            self._spi.mode = 0
            self._spi.max_speed_hz = speed
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"打开 SPI 失败: {e}")
            raise

        # 订阅模式切换
        self._sub_mode = self.create_subscription(String, '/ws2812_mode', self._on_mode_msg, 10)

        # 循环线程
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"WS2812 node started: count={self._count} mode={self._mode} speed={speed}Hz order={self._order}" )

    # -------------------- 解析/编码工具 --------------------
    def _parse_color(self, s: str) -> Pixel:
        try:
            parts = [int(x.strip()) for x in s.split(',')]
            if len(parts) != 3:
                raise ValueError
            r, g, b = [_clamp(v, 0, 255) for v in parts]
            return (r, g, b)
        except Exception:
            self.get_logger().warn(f"颜色字符串无效: {s}，使用默认 255,0,0")
            return (255, 0, 0)

    # -------------------- ROS 回调 --------------------
    def _on_mode_msg(self, msg: String) -> None:
        data = msg.data.strip()
        if not data:
            return
        new_mode = data
        new_color = None
        # 支持 "solid:R,G,B" 动态设置
        if data.startswith('solid:'):
            new_mode = 'solid'
            new_color = data.split(':', 1)[1]
        if new_mode not in ('blink', 'solid', 'off'):
            self.get_logger().warn(f"未知模式: {data}")
            return
        with self._lock:
            self._mode = new_mode
            if new_color:
                self._color_solid = self._parse_color(new_color)
        self.get_logger().info(f"切换模式 -> {self._mode}, 颜色={self._color_solid if self._mode=='solid' else 'N/A'}")

    # -------------------- 主循环线程 --------------------
    def _run_loop(self) -> None:
        phase = 0
        interval = 0.1  # 默认闪烁间隔，可做成参数
        off: Pixel = (0, 0, 0)
        try:
            while not self._stop_event.is_set():
                with self._lock:
                    mode = self._mode
                    count = self._count
                    order = self._order
                    brightness = self._brightness
                    block_size = self._block_size
                    color_solid = self._color_solid
                    color2 = self._color2

                if mode == 'off':
                    frame = encode_pixels([off] * count, order, 0)
                    self._show_frame(frame)
                    time.sleep(0.2)
                    continue
                elif mode == 'solid':
                    frame = encode_pixels([color_solid] * count, order, brightness)
                    self._show_frame(frame)
                    time.sleep(0.5)
                    continue
                else:  # blink
                    # phase 0 / 1 交替
                    p_pixels = build_block_pattern(count, color_solid, color2, block_size, phase % 2)
                    frame = encode_pixels(p_pixels, order, brightness)
                    self._show_frame(frame)
                    time.sleep(interval)
                    phase += 1
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"循环异常: {e}")
        finally:
            # 退出时清空
            try:
                frame = encode_pixels([off] * self._count, self._order, 0)
                self._show_frame(frame)
            except Exception:
                pass

    # -------------------- 设备操作 --------------------
    def _show_frame(self, frame: bytes) -> None:
        try:
            self._spi.writebytes2(frame)  # type: ignore[attr-defined]
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"SPI 发送失败: {e}")

    # -------------------- 清理 --------------------
    def destroy_node(self):  # type: ignore[override]
        self._stop_event.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._spi.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WS2812Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("WS2812 node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
