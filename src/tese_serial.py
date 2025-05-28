import serial
import time

def main():
    port = '/dev/ttyTHS1'
    baudrate = 115200
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"打开串口 {port} 成功，开始接收数据...")
        while True:
            hex_data = '01'  # 你要发送的十六进制内容
            # ser.reset_output_buffer()  # 清空输入缓冲区
            # ser.reset_input_buffer()  # 清空输出缓冲区
            ser.write(bytes.fromhex(hex_data))
            if ser.in_waiting:
                # 读取所有数据，只取最后一个字节
                data = ser.read(ser.in_waiting)
                if data:
                    last_byte = data[-1]
                    print(f"最新字节: {last_byte:02X}")
            time.sleep(0.1)
    except serial.SerialException as e:
        print(f"串口打开失败: {e}")
    except KeyboardInterrupt:
        print("退出程序")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == '__main__':
    main()