import board
from adafruit_pca9685 import PCA9685

def percentage_to_duty_cycle(percentage):
    """
    将百分比转换为16位整数，用于设置占空比。
    
    参数:
        percentage (float): 占空比的百分比值 (0-100)。
    
    返回:
        int: 对应的16位整数值 (0x0000 - 0xFFFF)。
    """
    if percentage < 0 or percentage > 100:
        raise ValueError("Percentage must be between 0 and 100")
    
    return int((percentage / 100) * 0xFFFF)

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
pca.channels[0].duty_cycle = 0x7FFF  # 50% duty cycle