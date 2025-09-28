#!/usr/bin/env python3
"""
WS2812 红蓝交替闪烁（SPI 方案，适配 Jetson Orin NX 等 Linux SBC）

思路：用 SPI 以 ~2.4MHz 速率发送编码数据，将每个 WS2812 的 1bit 编码为 3bit：
- 1 -> 110 （高电平约 0.83us）
- 0 -> 100 （高电平约 0.42us）

这样 8 个 WS2812 位就对应 24 个 SPI 位，正好 3 字节（无需填充）。
复位“低电平”通过追加若干个 0x00 字节实现（> 50us）。

接线（默认 SPI0.0）：
- MOSI -> WS2812 DIN    pin 19
- GND  -> WS2812 GND（与电源共地）
- 5V   -> WS2812 VCC（建议外部 5V 供电）
建议在数据线上串 300–470Ω 电阻、电源两端并联 ≥1000µF 电容。
若遇到识别不稳，建议使用 5V -> 3.3V 逻辑电平转换器。
"""

import argparse
import signal
import sys
import time
from typing import Iterable, List, Sequence, Tuple

try:
    import spidev
except Exception as e:  # noqa: BLE001
    print("未找到 spidev，请先安装: pip install spidev", file=sys.stderr)
    print(f"导入错误: {e}", file=sys.stderr)
    sys.exit(1)


Pixel = Tuple[int, int, int]  # (R, G, B)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WS2812 SPI 驱动：闪烁/常亮")
    parser.add_argument("--count", "-n", type=int, default=128, help="LED 数量，默认 8")
    parser.add_argument("--bus", type=int, default=0, help="SPI 总线号，默认 0（/dev/spidev0.*）")
    parser.add_argument("--device", type=int, default=0, help="SPI 设备号，默认 0（/dev/spidev*.0）")
    parser.add_argument("--speed", type=int, default=2400000, help="SPI 速率 Hz，默认 2.4MHz")
    parser.add_argument("--brightness", "-b", type=int, default=64, help="亮度 0-255，默认 64")
    parser.add_argument("--interval", "-i", type=float, default=0.1, help="闪烁模式下的切换间隔秒，默认 0.5s")
    parser.add_argument("--loops", type=int, default=0, help="循环次数，0 表示无限循环")
    parser.add_argument(
        "--order",
        choices=["GRB", "RGB", "BGR", "RBG", "GBR", "BRG"],
        default="GRB",
        help="颜色顺序，默认 GRB（多数 WS2812）",
    )
    parser.add_argument("--no-clear", action="store_true", help="退出时不清空关闭灯带")
    parser.add_argument(
        "--mode",
        choices=["blink", "solid"],
        default="blink",
        help="显示模式：blink（红蓝闪烁）或 solid（常亮单色），默认 blink",
    )
    parser.add_argument(
        "--color",
        type=str,
        default="255,0,0",
        help="常亮颜色，格式 R,G,B（0-255），默认 255,0,0（红）",
    )
    # 新增：块大小与第二颜色（用于 blink 分块红蓝交替）
    parser.add_argument(
        "--block-size",
        type=int,
        default=8,
        help="分块大小（每多少个灯为一组交替颜色），默认 8",
    )
    parser.add_argument(
        "--color2",
        type=str,
        default="0,0,255",
        help="blink 第二种颜色，格式 R,G,B（0-255），默认 0,0,255（蓝）",
    )
    return parser.parse_args()


def build_lut() -> List[bytes]:
    """预计算 0..255 -> 3 字节编码查找表。"""
    lut: List[bytes] = [b"\x00\x00\x00"] * 256
    for val in range(256):
        bits = []
        for i in range(7, -1, -1):
            bit = (val >> i) & 1
            # 1 -> 110, 0 -> 100
            if bit:
                bits.extend((1, 1, 0))
            else:
                bits.extend((1, 0, 0))
        # 将 24 个比特打包为 3 字节
        b0 = (bits[0] << 7) | (bits[1] << 6) | (bits[2] << 5) | (bits[3] << 4) | (bits[4] << 3) | (bits[5] << 2) | (bits[6] << 1) | bits[7]
        b1 = (bits[8] << 7) | (bits[9] << 6) | (bits[10] << 5) | (bits[11] << 4) | (bits[12] << 3) | (bits[13] << 2) | (bits[14] << 1) | bits[15]
        b2 = (bits[16] << 7) | (bits[17] << 6) | (bits[18] << 5) | (bits[19] << 4) | (bits[20] << 3) | (bits[21] << 2) | (bits[22] << 1) | bits[23]
        lut[val] = bytes((b0, b1, b2))
    return lut


LUT = build_lut()


def apply_brightness(color: Pixel, brightness: int) -> Pixel:
    b = max(0, min(255, brightness))
    if b >= 255:
        return color
    r, g, bl = color
    # 线性缩放
    return (
        (r * b) // 255,
        (g * b) // 255,
        (bl * b) // 255,
    )


def order_channels(color: Pixel, order: str) -> Tuple[int, int, int]:
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
    """将像素数组编码为 SPI 字节流。每个通道 1 字节 -> 3 字节。"""
    out = bytearray()
    for c in pixels:
        c2 = apply_brightness(c, brightness)
        ch = order_channels(c2, order)
        for byte in ch:  # type: ignore[assignment]
            out += LUT[byte]
    # 追加复位低电平：约 80us（在 2.4MHz 下，一个字节 ~3.33us）
    out += b"\x00" * 32  # ~106us
    return bytes(out)


def open_spi(bus: int, dev: int, speed: int) -> spidev.SpiDev:
    spi = spidev.SpiDev()
    try:
        spi.open(bus, dev)
    except PermissionError as e:
        print(
            "打开 SPI 设备失败（权限不足）。请使用 sudo 运行，或将当前用户加入 gpio 组后重登：\n"
            "  sudo usermod -aG gpio $USER\n"
            "  注：若使用 sudo，请确保 Python 环境一致。",
            file=sys.stderr,
        )
        raise e
    spi.mode = 0
    spi.max_speed_hz = speed
    return spi


def show_frame(spi: spidev.SpiDev, frame: bytes) -> None:
    # writebytes2 接受 bytes，避免将数据拆成 list 提高效率
    spi.writebytes2(frame)


def set_all(count: int, color: Pixel) -> List[Pixel]:
    return [color] * count


def build_block_pattern(
    count: int,
    color_a: Pixel,
    color_b: Pixel,
    block_size: int = 8,
    phase: int = 0,
) -> List[Pixel]:
    """生成按块交替颜色的像素数组。

    参数：
    - count: LED 数量
    - color_a, color_b: 两种交替颜色
    - block_size: 每多少个 LED 为一组
    - phase: 0 表示以 color_a 开始，1 表示以 color_b 开始
    """
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


def main() -> int:
    args = parse_args()
    print(
        f"[WS2812 SPI] bus={args.bus} device={args.device} speed={args.speed}Hz "
        f"count={args.count} brightness={args.brightness} interval={args.interval}s order={args.order} "
        f"loops={args.loops} mode={args.mode}")

    try:
        spi = open_spi(args.bus, args.device, args.speed)
    except Exception as e:  # 已打印权限建议
        print(f"打开 SPI 失败: {e}", file=sys.stderr)
        return 1

    red: Pixel = (255, 0, 0)
    blue: Pixel = (0, 0, 255)
    off: Pixel = (0, 0, 0)

    # 解析 --color
    def parse_color(s: str) -> Pixel:
        try:
            parts = [int(x.strip()) for x in s.split(',')]
            if len(parts) != 3:
                raise ValueError
            r, g, b = [max(0, min(255, v)) for v in parts]
            return (r, g, b)
        except Exception:
            print(f"颜色参数无效：{s}，使用默认 255,0,0", file=sys.stderr)
            return (255, 0, 0)
    solid_color = parse_color(args.color)
    color2 = parse_color(args.color2)

    def cleanup(signum=None, frame=None):  # noqa: ARG001
        try:
            if not args.no_clear:
                off_frame = encode_pixels(set_all(args.count, off), args.order, 0)
                show_frame(spi, off_frame)
        finally:
            try:
                spi.close()
            finally:
                sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    iteration = 0
    try:
        if args.mode == "solid":
            frame = encode_pixels(set_all(args.count, solid_color), args.order, args.brightness)
            show_frame(spi, frame)
            if args.loops > 0:
                # 短暂停留后退出
                time.sleep(max(0.05, args.interval))
            else:
                # 常亮保持直到 Ctrl+C
                while True:
                    time.sleep(1)
        else:  # blink：按块交替颜色，并在两种相反相位间闪烁
            # 预构建两个相位的帧：phase0 从 colorA 开始，phase1 从 colorB 开始
            pixels_p0 = build_block_pattern(args.count, red, color2, args.block_size, phase=0)
            pixels_p1 = build_block_pattern(args.count, red, color2, args.block_size, phase=1)
            frame_p0 = encode_pixels(pixels_p0, args.order, args.brightness)
            frame_p1 = encode_pixels(pixels_p1, args.order, args.brightness)

            while True:
                show_frame(spi, frame_p0)
                time.sleep(max(0.01, args.interval))

                show_frame(spi, frame_p1)
                time.sleep(max(0.01, args.interval))

                if args.loops > 0:
                    iteration += 1
                    if iteration >= args.loops:
                        break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not args.no_clear:
                off_frame = encode_pixels(set_all(args.count, off), args.order, 0)
                show_frame(spi, off_frame)
        finally:
            spi.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
