#!/usr/bin/env python3
"""
Jetson Orin NX GPIO 测试：将 40Pin 排针的物理 7 号脚每 5 秒在高/低电平间切换。

注意：
- 使用 BOARD 编号方式，PIN=7 即物理引脚 7。
- 需要 root 权限或当前用户属于 gpio 组。
- 引脚为 3.3V 电平，请勿直接驱动大电流负载，必要时通过三极管/继电器。
"""

import sys
import time
import signal

try:
	import Jetson.GPIO as GPIO
except Exception as e:
	print("无法导入 Jetson.GPIO，请在 Jetson 平台上运行并确认已安装库。错误：", e)
	sys.exit(1)


PIN = 7           # 物理引脚编号（BOARD 模式）
INTERVAL = 5.0    # 切换间隔（秒）


def cleanup_and_exit(code: int = 0):
	try:
		# 尝试将引脚拉低，避免悬空高电平
		GPIO.output(PIN, GPIO.LOW)
	except Exception:
		pass
	finally:
		try:
			GPIO.cleanup()
		except Exception:
			pass
	sys.exit(code)


def handle_signal(signum, frame):
	print(f"\n收到信号 {signum}，正在清理 GPIO 并退出…")
	cleanup_and_exit(0)


def main():
	# 注册信号处理，便于 Ctrl+C 或系统停止时清理资源
	signal.signal(signal.SIGINT, handle_signal)
	signal.signal(signal.SIGTERM, handle_signal)

	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)

	print(f"开始切换 GPIO（BOARD 引脚 {PIN}），每 {INTERVAL:.1f} 秒切换一次。按 Ctrl+C 退出。")

	try:
		# 先置高，再循环高/低切换
		state = GPIO.HIGH
		while True:
			GPIO.output(PIN, state)
			print("引脚状态:", "HIGH" if state == GPIO.HIGH else "LOW")
			time.sleep(INTERVAL)
			# 翻转状态
			state = GPIO.LOW if state == GPIO.HIGH else GPIO.HIGH
	except KeyboardInterrupt:
		pass
	finally:
		cleanup_and_exit(0)


if __name__ == "__main__":
	main()

