#!/usr/bin/env python3
"""
Orin NX CAN硬件检测工具
帮助确定CAN接口的硬件配置和连接状态
"""

import subprocess
import os
import time

def run_command(cmd):
    """执行命令并返回结果"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except Exception as e:
        return False, "", str(e)

def check_hardware_info():
    """检查硬件信息"""
    print("=== Orin NX CAN硬件检测 ===\n")
    
    # 1. 检查设备型号
    print("1. 设备信息:")
    success, stdout, stderr = run_command("cat /proc/device-tree/model 2>/dev/null")
    if success and stdout:
        print(f"   设备型号: {stdout}")
    else:
        print("   无法读取设备型号")
    
    # 2. 检查CAN控制器
    print("\n2. CAN控制器信息:")
    success, stdout, stderr = run_command("lspci | grep -i can")
    if stdout:
        print(f"   PCI CAN设备: {stdout}")
    else:
        print("   未找到PCI CAN设备")
    
    # 检查平台设备
    success, stdout, stderr = run_command("ls /sys/bus/platform/devices/ | grep can")
    if stdout:
        print(f"   平台CAN设备: {stdout}")
        
        # 获取设备详细信息
        for device in stdout.split('\n'):
            if 'can' in device:
                device_path = f"/sys/bus/platform/devices/{device}"
                print(f"   设备路径: {device_path}")
                
                # 尝试读取设备属性
                for attr in ['uevent', 'modalias', 'driver']:
                    attr_path = f"{device_path}/{attr}"
                    if os.path.exists(attr_path):
                        try:
                            with open(attr_path, 'r') as f:
                                content = f.read().strip()
                                print(f"     {attr}: {content}")
                        except:
                            pass
    
def check_can_interfaces():
    """检查CAN网络接口"""
    print("\n3. CAN网络接口:")
    
    # 检查所有网络接口
    success, stdout, stderr = run_command("ip link show")
    can_interfaces = []
    
    for line in stdout.split('\n'):
        if 'can' in line.lower():
            can_interfaces.append(line.strip())
            print(f"   {line.strip()}")
    
    if not can_interfaces:
        print("   未找到CAN接口")
        return []
    
    # 获取CAN接口详细信息
    for i, interface_line in enumerate(can_interfaces):
        if ': can' in interface_line:
            interface_name = interface_line.split(':')[1].split('@')[0].strip()
            print(f"\n   CAN接口 {interface_name} 详细信息:")
            
            success, stdout, stderr = run_command(f"ip -details link show {interface_name}")
            if success:
                for detail_line in stdout.split('\n'):
                    if 'can' in detail_line or 'bitrate' in detail_line or 'state' in detail_line:
                        print(f"     {detail_line.strip()}")
    
    return [line.split(':')[1].split('@')[0].strip() for line in can_interfaces if ': can' in line]

def check_can_drivers():
    """检查CAN驱动模块"""
    print("\n4. CAN驱动模块:")
    
    # 检查已加载的CAN相关模块
    success, stdout, stderr = run_command("lsmod | grep can")
    if stdout:
        print("   已加载的CAN模块:")
        for line in stdout.split('\n'):
            print(f"     {line}")
    else:
        print("   未找到已加载的CAN模块")
    
    # 检查可用的CAN模块
    kernel_version = subprocess.run("uname -r", shell=True, capture_output=True, text=True).stdout.strip()
    modules_path = f"/lib/modules/{kernel_version}"
    
    success, stdout, stderr = run_command(f"find {modules_path} -name '*can*' -type f")
    if stdout:
        print("   可用的CAN模块文件:")
        for module in stdout.split('\n')[:10]:  # 只显示前10个
            module_name = os.path.basename(module)
            print(f"     {module_name}")
        if len(stdout.split('\n')) > 10:
            print(f"     ... 还有 {len(stdout.split('\n')) - 10} 个模块")

def check_gpio_pins():
    """检查GPIO引脚配置"""
    print("\n5. GPIO和引脚配置:")
    
    # 检查设备树中的CAN配置
    success, stdout, stderr = run_command("find /proc/device-tree -name '*can*' -type d 2>/dev/null")
    if stdout:
        print("   设备树中的CAN配置:")
        for path in stdout.split('\n'):
            print(f"     {path}")
    
    # 检查GPIO导出
    gpio_path = "/sys/class/gpio"
    if os.path.exists(gpio_path):
        success, stdout, stderr = run_command(f"ls {gpio_path}")
        if stdout:
            print(f"   GPIO控制器: {stdout}")
    else:
        print("   未找到GPIO控制接口")

def test_can_loopback(interface):
    """测试CAN回环功能"""
    print(f"\n6. CAN接口 {interface} 功能测试:")
    
    # 检查接口状态
    success, stdout, stderr = run_command(f"ip -details link show {interface}")
    if not success:
        print(f"   错误: 无法获取接口 {interface} 信息")
        return False
    
    print("   接口状态:")
    for line in stdout.split('\n'):
        if any(keyword in line for keyword in ['state', 'bitrate', 'can']):
            print(f"     {line.strip()}")
    
    # 尝试配置接口
    print("   正在配置CAN接口...")
    commands = [
        f"sudo ip link set {interface} down",
        f"sudo ip link set {interface} type can bitrate 500000 restart-ms 100",
        f"sudo ip link set {interface} up"
    ]
    
    for cmd in commands:
        success, stdout, stderr = run_command(cmd)
        if not success:
            print(f"   错误: {cmd} 失败: {stderr}")
            return False
    
    print("   CAN接口配置成功")
    
    # 检查最终状态
    success, stdout, stderr = run_command(f"ip -details link show {interface}")
    if success:
        for line in stdout.split('\n'):
            if 'can state' in line:
                print(f"   最终状态: {line.strip()}")
                return 'ERROR-ACTIVE' in line
    
    return False

def generate_connection_guide(can_interfaces):
    """生成连接指导"""
    print("\n" + "="*50)
    print("🔌 连接指导建议")
    print("="*50)
    
    if not can_interfaces:
        print("\n❌ 未检测到可用的CAN接口")
        print("可能的原因:")
        print("1. CAN功能未在设备树中启用")
        print("2. 需要加载CAN驱动模块")
        print("3. 硬件不支持或未正确配置")
        return
    
    print(f"\n✅ 检测到CAN接口: {', '.join(can_interfaces)}")
    
    for interface in can_interfaces:
        print(f"\n📋 {interface} 连接建议:")
        
        # 测试接口功能
        if test_can_loopback(interface):
            print("   ✅ CAN控制器工作正常")
            print("   📌 硬件连接步骤:")
            print("   1. 确认您的载板是否有内置CAN收发器")
            print("   2. 如果没有，需要添加外部CAN收发器模块 (如TJA1050)")
            print("   3. 连接CAR28F雷达:")
            print("      - CAR28F CAN-H <-> 载板/收发器 CAN-H")
            print("      - CAR28F CAN-L <-> 载板/收发器 CAN-L") 
            print("      - CAR28F GND   <-> 载板 GND")
            print("   4. 确保总线两端有120Ω终端电阻")
            
            print(f"\n   🧪 测试命令:")
            print(f"   candump {interface}           # 监听消息")
            print(f"   cansend {interface} 123#DEADBEEF  # 发送测试")
            
        else:
            print("   ❌ CAN控制器配置失败")
            print("   需要检查驱动和硬件配置")

def main():
    """主函数"""
    try:
        # 检查权限
        if os.geteuid() != 0:
            print("⚠️  建议使用sudo运行此脚本以获取完整信息")
            print("   sudo python3 can_hardware_detector.py\n")
        
        # 执行检测
        check_hardware_info()
        can_interfaces = check_can_interfaces()
        check_can_drivers()
        check_gpio_pins()
        
        # 生成连接指导
        generate_connection_guide(can_interfaces)
        
        print("\n" + "="*50)
        print("🔍 检测完成")
        print("="*50)
        print("如需更多帮助，请查看 Orin_NX_CAN_Connection_Guide.md")
        
    except KeyboardInterrupt:
        print("\n检测被用户中断")
    except Exception as e:
        print(f"\n检测过程中发生错误: {e}")

if __name__ == "__main__":
    main()
