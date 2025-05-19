import serial
import time

# 打开串口
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)  # 端口和波特率请根据实际情况修改

try:
    while True:
        # 发送十六进制数据
        hex_data = '01'  # 你要发送的十六进制内容
        ser.write(bytes.fromhex(hex_data))
        print(f"发送: {hex_data}")
        time.sleep(0.1)

        # 读串口数据
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            if data.hex() == '01':
                print("自动控制")
            elif data.hex() == '02':
                print("手动控制")
            print(f"接收: {data.hex()}")  # 以十六进制显示接收内容

except KeyboardInterrupt:
    print("退出程序")

finally:
    ser.close()