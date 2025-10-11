import serial
import time

def parse_uart_frame(frame):
    # 协议: 帧头(0xFF) + Data_H + Data_L + SUM
    if len(frame) != 4:
        return None
    if frame[0] != 0xFF:
        return None
    data_h = frame[1]
    data_l = frame[2]
    sum_check = frame[3]
    # 校验
    calc_sum = (frame[0] + data_h + data_l) & 0xFF
    if sum_check != calc_sum:
        return None
    # 距离值
    distance_raw = data_h * 256 + data_l
    return distance_raw

def main():
    port = '/dev/ttyCH9344USB0'
    baudrate = 9600  # 根据实际设备设置   单头
    ser = serial.Serial(port, baudrate, timeout=0.5)
    print(f"打开串口 {port}，波特率 {baudrate}")

    while True:
        data = ser.read(4)
        if len(data) == 4:
            distance = parse_uart_frame(data)
            if distance is not None:
                print(f"距离原始值: {distance}，距离(mm): {distance} mm")
            else:
                print("数据校验失败或帧头错误")
        else:
            time.sleep(0.01)

if __name__ == "__main__":
    main()