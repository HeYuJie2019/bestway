import serial
import time

def calculate_xor(data):
    """
    计算校验码 (XOR)
    :param data: 数据帧（不包括帧头）
    :return: 校验码
    """
    xor = 0
    for byte in data:
        xor ^= byte
    return xor

def create_sbus_frame(channels, flag=0x00):
    """
    创建 SBUS 数据帧
    :param channels: 包含 16 个通道值的列表（0-2047）
    :param flag: Flag 字节，默认值为 0x0C
    :return: 完整的 35 字节数据帧
    """
    if len(channels) != 16:
        raise ValueError("需要 16 个通道值")

    # 限制通道值范围为 0-2047
    channels = [min(max(ch, 0), 2047) for ch in channels]

    # 起始字节
    frame = [0x0F]

    # 编码 16 个通道值为 32 字节
    for ch in channels:
        frame.append((ch >> 8) & 0xFF)  # 高字节
        frame.append(ch & 0xFF)        # 低字节

    # 添加 Flag 字节
    frame.append(flag)

    # 计算校验码 (XOR)
    xor = calculate_xor(frame[1:])  # 不包括帧头
    frame.append(xor)

    return bytes(frame)

def send_sbus_frame(port, baudrate, channels):
    """
    通过串口发送 SBUS 数据帧
    :param port: 串口端口，例如 '/dev/ttyUSB0'
    :param baudrate: 波特率，例如 115200
    :param channels: 包含 16 个通道值的列表（0-2047）
    """
    frame = create_sbus_frame(channels)
    with serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO) as ser:
        ser.write(frame)
        print(f"发送数据帧: {frame.hex().upper()}")

if __name__ == "__main__":
    # 示例用法
    port = '/dev/ttyUSB_SBUS'  # 使用固定的USB设备名称
    baudrate = 115200
    channels = [1200] * 16  # 16 个通道值均为 1002 (0x03EA)
    channels[0] = 1002
    channels[1] = 1002
    channels[2] = 1002
    channels[3] = 1002
    channels[4] = 1002
    channels[5] = 1002
    channels[6] = 1002
    channels[7] = 1002
    channels[8] = 1002
    channels[9] = 1002
    channels[10] = 1002
    channels[11] = 1002
    channels[12] = 1002
    channels[13] = 1002
    channels[14] = 1002
    channels[15] = 1002
    try:
        t1 = time.time()
        t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 < 2:
                channels[0] = 1002  #1002减小左转，增大右转
                channels[1] = 1002  #1002
                channels[2] = 800  #1002
                channels[3] = 1002  #1002
                channels[4] = 1002  #1002
                channels[5] = 1002  #1002
                channels[6] = 282   #282
                channels[7] = 1722   #282关闭灯   1722打开灯
                channels[8] = 282   #282
                channels[9] = 282   #282
                channels[10] = 1002 #1002
                channels[11] = 1002 #1002
                channels[12] = 1002 #1002
                channels[13] = 1002 #1002
                channels[14] = 1002 #1002
            elif 2 < t2 - t1 < 4:
                channels[0] = 1002  #1002减小左转，增大右转
                channels[1] = 1002  #1002
                channels[2] = 1200  #1002
                channels[3] = 1002  #1002
                channels[4] = 1002  #1002
                channels[5] = 1002  #1002
                channels[6] = 282   #282
                channels[7] = 1722   #282关闭灯   1722打开灯
                channels[8] = 282   #282
                channels[9] = 282   #282
                channels[10] = 1002 #1002
                channels[11] = 1002 #1002
                channels[12] = 1002 #1002
                channels[13] = 1002 #1002
                channels[14] = 1002 #1002
                channels[15] = 1002 #1002
            elif 4 < t2 - t1 < 6:
                channels[0] = 1002  #1002减小左转，增大右转
                channels[1] = 1002  #1002
                channels[2] = 800  #1002
                channels[3] = 1002  #1002
                channels[4] = 1002  #1002
                channels[5] = 1002  #1002
                channels[6] = 282   #282
                channels[7] = 1722   #282关闭灯   1722打开灯
                channels[8] = 282   #282
                channels[9] = 282   #282
                channels[10] = 1002 #1002
                channels[11] = 1002 #1002
                channels[12] = 1002 #1002
                channels[13] = 1002 #1002
                channels[14] = 1002 #1002
                channels[15] = 1002 #1002
            else:
                channels[0] = 1002  #1002减小左转，增大右转
                channels[1] = 1002  #1002
                channels[2] = 1002  #1002
                channels[3] = 1002  #1002
                channels[4] = 1002  #1002
                channels[5] = 1002  #1002
                channels[6] = 282   #282
                channels[7] = 282   #282关闭灯   1722打开灯
                channels[8] = 282   #282
                channels[9] = 282   #282
                channels[10] = 1002 #1002
                channels[11] = 1002 #1002
                channels[12] = 1002 #1002
                channels[13] = 1002 #1002
                channels[14] = 1002 #1002
                channels[15] = 1002 #1002
            send_sbus_frame(port, baudrate, channels)
            time.sleep(0.02)
    except Exception as e:
        print(f"发送失败: {e}")