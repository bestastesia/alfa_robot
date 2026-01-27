#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leadshine LD2-CAN 伺服驱动器多圈位置控制脚本
使用CANopen协议 (CiA402 Profile) 实现位置控制

硬件: Leadshine LD2-CAN Servo Driver
协议: CANopen (CiA402 Profile)
接口: USB-CAN (Python canopen library)
设置: Node-ID = 1, Baudrate = 500k
"""

import canopen
import time
import sys
from typing import Optional


class LeadshineServoController:
    """Leadshine LD2-CAN 伺服驱动器控制器类"""
    
    # 对象字典索引定义
    CONTROLWORD = 0x6040
    STATUSWORD = 0x6041
    MODES_OF_OPERATION = 0x6060
    MODES_DISPLAY = 0x6061
    TARGET_POSITION = 0x607A
    PROFILE_VELOCITY = 0x6081
    TARGET_VELOCITY = 0x60FF
    POSITION_ACTUAL = 0x6064
    VELOCITY_ACTUAL = 0x606C
    MAX_CURRENT = 0x6071  # 最大电流限制 (单位: mA)
    
    # 单位转换常数
    PULSES_PER_REVOLUTION = 10000  # 10,000脉冲 = 1转
    
    # 控制字定义
    CONTROLWORD_ENABLE_SEQUENCE = [0x06, 0x07, 0x0F]  # 使能序列
    CONTROLWORD_RESET_FAULT = 0x80  # 复位故障
    CONTROLWORD_TRIGGER_MOTION = 0x1F  # 触发运动 (位4上升沿)
    
    # 状态字位定义
    STATUS_BIT_FAULT = 3
    STATUS_BIT_TARGET_REACHED = 10
    STATUS_BIT_SETPOINT_ACK = 12
    
    # 操作模式
    MODE_PROFILE_POSITION = 1  # PP模式
    MODE_PROFILE_VELOCITY = 3  # PV模式
    MODE_HOMING = 6  # 回零模式
    
    def __init__(self, channel: str = "can0", node_id: int = 1, baudrate: int = 500000):
        """
        初始化伺服控制器
        
        Args:
            channel: CAN接口通道 (例如: "can0", "slcan0", "PCAN_USBBUS1")
            node_id: 节点ID (默认: 1)
            baudrate: CAN波特率 (默认: 500000)
        """
        self.channel = channel
        self.node_id = node_id
        self.baudrate = baudrate
        self.network = None
        self.node = None
        self._connected = False
        
    def connect(self) -> bool:
        """
        连接到CAN总线和节点
        
        Returns:
            bool: 连接是否成功
        """
        try:
            # 创建CANopen网络
            self.network = canopen.Network()
            self.network.connect(bustype='socketcan', channel=self.channel, bitrate=self.baudrate)
            
            # 添加节点 (使用CiA402配置文件)
            self.node = self.network.add_node(self.node_id, 'CiA402.eds')
            
            # 如果EDS文件不存在，使用通用配置
            if self.node is None:
                print(f"警告: 无法加载EDS文件，使用通用配置")
                self.node = self.network.add_node(self.node_id)
            
            self._connected = True
            print(f"✓ 成功连接到节点 {self.node_id} (通道: {self.channel}, 波特率: {self.baudrate})")
            return True
            
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            print("\n提示: 请确保:")
            print("  1. CAN接口已正确连接 (例如: sudo ip link set can0 up type can bitrate 500000)")
            print("  2. canopen库已安装 (pip install canopen)")
            print("  3. 节点ID和波特率设置正确")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.network:
            try:
                self.network.disconnect()
                self._connected = False
                print("✓ 已断开连接")
            except Exception as e:
                print(f"✗ 断开连接时出错: {e}")
    
    def read_statusword(self) -> Optional[int]:
        """
        读取状态字
        
        Returns:
            int: 状态字值，失败返回None
        """
        try:
            status = self.node.sdo[self.STATUSWORD].raw
            return status
        except Exception as e:
            print(f"✗ 读取状态字失败: {e}")
            return None
    
    def check_fault(self) -> bool:
        """
        检查是否有故障
        
        Returns:
            bool: True表示有故障
        """
        status = self.read_statusword()
        if status is None:
            return True
        return bool(status & (1 << self.STATUS_BIT_FAULT))
    
    def check_target_reached(self) -> bool:
        """
        检查是否到达目标位置
        
        Returns:
            bool: True表示已到达目标
        """
        status = self.read_statusword()
        if status is None:
            return False
        return bool(status & (1 << self.STATUS_BIT_TARGET_REACHED))
    
    def reset_fault(self) -> bool:
        """
        复位故障
        
        Returns:
            bool: 复位是否成功
        """
        try:
            self.node.sdo[self.CONTROLWORD].raw = self.CONTROLWORD_RESET_FAULT
            time.sleep(0.1)
            
            # 清除复位位
            self.node.sdo[self.CONTROLWORD].raw = 0x00
            time.sleep(0.1)
            
            if not self.check_fault():
                print("✓ 故障已复位")
                return True
            else:
                print("✗ 故障复位失败，仍有故障")
                return False
                
        except Exception as e:
            print(f"✗ 复位故障失败: {e}")
            return False
    
    def enable(self) -> bool:
        """
        执行使能序列
        状态机转换: Switch On Disabled -> Ready to Switch On -> Switched On -> Operation Enabled
        
        Returns:
            bool: 使能是否成功
        """
        if not self._connected:
            print("✗ 未连接到设备")
            return False
        
        try:
            # 检查故障
            if self.check_fault():
                print("⚠ 检测到故障，尝试复位...")
                if not self.reset_fault():
                    return False
            
            # 使能序列: 0x06 -> 0x07 -> 0x0F
            print("开始使能序列...")
            
            for i, cw in enumerate(self.CONTROLWORD_ENABLE_SEQUENCE):
                self.node.sdo[self.CONTROLWORD].raw = cw
                time.sleep(0.1)
                
                status = self.read_statusword()
                print(f"  步骤 {i+1}/3: Controlword=0x{cw:02X}, Statusword=0x{status:04X}")
            
            # 验证使能状态
            time.sleep(0.2)
            status = self.read_statusword()
            if status and not self.check_fault():
                print("✓ 使能成功")
                return True
            else:
                print("✗ 使能失败")
                return False
                
        except Exception as e:
            print(f"✗ 使能过程出错: {e}")
            return False
    
    def set_mode(self, mode: int) -> bool:
        """
        设置操作模式
        
        Args:
            mode: 操作模式 (1=PP, 3=PV, 6=Homing)
        
        Returns:
            bool: 设置是否成功
        """
        try:
            self.node.sdo[self.MODES_OF_OPERATION].raw = mode
            time.sleep(0.1)
            
            # 验证模式
            actual_mode = self.node.sdo[self.MODES_DISPLAY].raw
            if actual_mode == mode:
                mode_names = {1: "PP (Profile Position)", 3: "PV (Profile Velocity)", 6: "Homing"}
                print(f"✓ 模式设置成功: {mode_names.get(mode, f'Mode {mode}')}")
                return True
            else:
                print(f"✗ 模式设置失败: 期望 {mode}, 实际 {actual_mode}")
                return False
                
        except Exception as e:
            print(f"✗ 设置模式失败: {e}")
            return False
    
    def set_profile_velocity(self, velocity_pulses_per_sec: int) -> bool:
        """
        设置轮廓速度 (用于PP模式)
        
        Args:
            velocity_pulses_per_sec: 速度值 (脉冲/秒)
        
        Returns:
            bool: 设置是否成功
        """
        try:
            self.node.sdo[self.PROFILE_VELOCITY].raw = velocity_pulses_per_sec
            print(f"✓ 轮廓速度设置: {velocity_pulses_per_sec} 脉冲/秒 "
                  f"({self.pulses_to_rpm(velocity_pulses_per_sec):.2f} RPM)")
            return True
        except Exception as e:
            print(f"✗ 设置轮廓速度失败: {e}")
            return False
    
    def set_target_position(self, position_pulses: int) -> bool:
        """
        设置目标位置
        
        Args:
            position_pulses: 目标位置 (脉冲数，支持多圈)
        
        Returns:
            bool: 设置是否成功
        """
        try:
            # 处理负数位置 (如果需要反向)
            if position_pulses < 0:
                # 转换为32位有符号整数
                position_pulses = position_pulses & 0xFFFFFFFF
            
            self.node.sdo[self.TARGET_POSITION].raw = position_pulses
            revolutions = position_pulses / self.PULSES_PER_REVOLUTION
            print(f"✓ 目标位置设置: {position_pulses} 脉冲 ({revolutions:.2f} 圈)")
            return True
        except Exception as e:
            print(f"✗ 设置目标位置失败: {e}")
            return False
    
    def trigger_motion(self) -> bool:
        """
        触发运动 (切换Controlword位4)
        
        Returns:
            bool: 触发是否成功
        """
        try:
            # 先清除位4
            current_cw = self.node.sdo[self.CONTROLWORD].raw
            self.node.sdo[self.CONTROLWORD].raw = current_cw & ~0x10
            time.sleep(0.05)
            
            # 设置位4 (上升沿触发)
            self.node.sdo[self.CONTROLWORD].raw = self.CONTROLWORD_TRIGGER_MOTION
            print("✓ 运动已触发")
            return True
        except Exception as e:
            print(f"✗ 触发运动失败: {e}")
            return False
    
    def get_actual_position(self) -> Optional[int]:
        """
        读取实际位置
        
        Returns:
            int: 实际位置 (脉冲数)，失败返回None
        """
        try:
            position = self.node.sdo[self.POSITION_ACTUAL].raw
            # 处理32位有符号整数
            if position > 0x7FFFFFFF:
                position = position - 0x100000000
            return position
        except Exception as e:
            print(f"✗ 读取实际位置失败: {e}")
            return None
    
    def get_actual_velocity(self) -> Optional[int]:
        """
        读取实际速度
        
        Returns:
            int: 实际速度 (脉冲/秒)，失败返回None
        """
        try:
            velocity = self.node.sdo[self.VELOCITY_ACTUAL].raw
            # 处理有符号整数
            if velocity > 0x7FFF:
                velocity = velocity - 0x10000
            return velocity
        except Exception as e:
            print(f"✗ 读取实际速度失败: {e}")
            return None
    
    def wait_for_target(self, timeout: float = 30.0, poll_interval: float = 0.1) -> bool:
        """
        等待到达目标位置
        
        Args:
            timeout: 超时时间 (秒)
            poll_interval: 轮询间隔 (秒)
        
        Returns:
            bool: True表示到达目标，False表示超时
        """
        start_time = time.time()
        
        print("等待到达目标位置...")
        while time.time() - start_time < timeout:
            if self.check_target_reached():
                print("✓ 已到达目标位置")
                return True
            
            # 检查故障
            if self.check_fault():
                print("✗ 检测到故障，运动中断")
                return False
            
            # 显示当前位置和速度
            pos = self.get_actual_position()
            vel = self.get_actual_velocity()
            if pos is not None and vel is not None:
                revolutions = pos / self.PULSES_PER_REVOLUTION
                rpm = self.pulses_to_rpm(vel)
                print(f"  位置: {pos} 脉冲 ({revolutions:.3f} 圈), "
                      f"速度: {vel} 脉冲/秒 ({rpm:.2f} RPM)", end='\r')
            
            time.sleep(poll_interval)
        
        print(f"\n✗ 等待超时 ({timeout}秒)")
        return False
    
    @staticmethod
    def rpm_to_pulses_per_sec(rpm: float) -> int:
        """
        将RPM转换为脉冲/秒
        
        Args:
            rpm: 转速 (转/分钟)
        
        Returns:
            int: 脉冲/秒
        """
        return int((rpm / 60) * LeadshineServoController.PULSES_PER_REVOLUTION)
    
    @staticmethod
    def pulses_to_rpm(pulses_per_sec: int) -> float:
        """
        将脉冲/秒转换为RPM
        
        Args:
            pulses_per_sec: 脉冲/秒
        
        Returns:
            float: RPM
        """
        return (pulses_per_sec * 60) / LeadshineServoController.PULSES_PER_REVOLUTION
    
    @staticmethod
    def revolutions_to_pulses(revolutions: float) -> int:
        """
        将圈数转换为脉冲数
        
        Args:
            revolutions: 圈数
        
        Returns:
            int: 脉冲数
        """
        return int(revolutions * LeadshineServoController.PULSES_PER_REVOLUTION)
    
    def move_to_position(self, target_revolutions: float, velocity_rpm: float = 10.0, 
                        timeout: float = 30.0) -> bool:
        """
        移动到指定位置 (多圈位置控制)
        
        Args:
            target_revolutions: 目标圈数 (支持小数，例如 2.5 圈)
            velocity_rpm: 运动速度 (RPM，默认10 RPM)
            timeout: 超时时间 (秒)
        
        Returns:
            bool: 运动是否成功完成
        """
        if not self._connected:
            print("✗ 未连接到设备")
            return False
        
        # 转换为脉冲
        target_pulses = self.revolutions_to_pulses(target_revolutions)
        velocity_pulses = self.rpm_to_pulses_per_sec(velocity_rpm)
        
        print(f"\n开始位置控制运动:")
        print(f"  目标: {target_revolutions} 圈 ({target_pulses} 脉冲)")
        print(f"  速度: {velocity_rpm} RPM ({velocity_pulses} 脉冲/秒)")
        
        # 1. 设置PP模式
        if not self.set_mode(self.MODE_PROFILE_POSITION):
            return False
        
        # 2. 设置轮廓速度
        if not self.set_profile_velocity(velocity_pulses):
            return False
        
        # 3. 设置目标位置
        if not self.set_target_position(target_pulses):
            return False
        
        # 4. 触发运动
        if not self.trigger_motion():
            return False
        
        # 5. 等待到达目标
        success = self.wait_for_target(timeout=timeout)
        
        # 显示最终位置
        final_pos = self.get_actual_position()
        if final_pos is not None:
            final_rev = final_pos / self.PULSES_PER_REVOLUTION
            error = abs(final_pos - target_pulses)
            error_rev = error / self.PULSES_PER_REVOLUTION
            print(f"\n最终位置: {final_pos} 脉冲 ({final_rev:.3f} 圈)")
            print(f"位置误差: {error} 脉冲 ({error_rev:.4f} 圈)")
        
        return success


def main():
    """主函数 - 示例用法"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Leadshine LD2-CAN 伺服驱动器多圈位置控制',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 移动到2.5圈位置，速度10 RPM
  python3 leadshine_can_position_control.py --position 2.5 --velocity 10
  
  # 移动到5圈位置，速度20 RPM，使用can1接口
  python3 leadshine_can_position_control.py --position 5 --velocity 20 --channel can1
  
  # 移动到-1.5圈 (反向)
  python3 leadshine_can_position_control.py --position -1.5 --velocity 10
        """
    )
    
    parser.add_argument('--channel', type=str, default='can0',
                       help='CAN接口通道 (默认: can0)')
    parser.add_argument('--node-id', type=int, default=1,
                       help='节点ID (默认: 1)')
    parser.add_argument('--baudrate', type=int, default=500000,
                       help='CAN波特率 (默认: 500000)')
    parser.add_argument('--position', type=float, required=True,
                       help='目标位置 (圈数，支持小数，例如: 2.5)')
    parser.add_argument('--velocity', type=float, default=10.0,
                       help='运动速度 (RPM，默认: 10.0)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='超时时间 (秒，默认: 30.0)')
    parser.add_argument('--enable-only', action='store_true',
                       help='仅执行使能，不执行运动')
    
    args = parser.parse_args()
    
    # 创建控制器
    controller = LeadshineServoController(
        channel=args.channel,
        node_id=args.node_id,
        baudrate=args.baudrate
    )
    
    try:
        # 连接
        if not controller.connect():
            sys.exit(1)
        
        # 使能
        if not controller.enable():
            print("✗ 使能失败，退出")
            sys.exit(1)
        
        # 如果只是使能，则退出
        if args.enable_only:
            print("✓ 使能完成，退出")
            return
        
        # 执行位置控制
        success = controller.move_to_position(
            target_revolutions=args.position,
            velocity_rpm=args.velocity,
            timeout=args.timeout
        )
        
        if success:
            print("\n✓ 运动完成")
            sys.exit(0)
        else:
            print("\n✗ 运动失败")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断")
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
