# radar_worker.py
import can
from PyQt5.QtCore import QThread, pyqtSignal
from datetime import datetime
import numpy as np
import sys

class RadarWorker(QThread):
    new_target = pyqtSignal(list)  # 目标数据信号 [x, y, z, tid]
    raw_data = pyqtSignal(str)  # 原始HEX数据信号
    no_data = pyqtSignal()  # 无数据信号
    status_signal = pyqtSignal(str)  # 状态信号
    radar_status = pyqtSignal(dict)  # 雷达状态信号

    def __init__(self, channel='PCAN_USBBUS1', bitrate=500000):
        super().__init__()
        # 解析接口类型
        self.bitrate = bitrate
        self.running = True
        self.can_bus = None
        self.last_message_time = None
        
        # 根据操作系统和通道名称确定接口类型
        if sys.platform.startswith('linux'):
            # Linux下使用socketcan接口
            self.interface = 'socketcan'
            self.channel = channel  # 例如: 'can0'
        else:
            # Windows系统
            if "Kvaser" in channel:
                self.interface = 'kvaser'
                self.channel = int(channel.split('_')[-1])  # 提取通道号
            else:
                self.interface = 'pcan'
                self.channel = channel  # PCAN通道名

    def run(self):
        try:
            # 动态创建总线实例
            bus_args = {
                'bitrate': self.bitrate
            }
            
            if sys.platform.startswith('linux'):
                # Linux: socketcan
                bus_args['interface'] = 'socketcan'
                bus_args['channel'] = self.channel
                bus_args['receive_own_messages'] = True
            else:
                # Windows: pcan or kvaser
                bus_args['interface'] = self.interface
                if self.interface == 'kvaser':
                    bus_args['channel'] = self.channel
                    bus_args['bus_type'] = "CAN"  # 明确总线类型
                else:  # PCAN
                    bus_args['channel'] = self.channel

            self.can_bus = can.interface.Bus(**bus_args)
            self.status_signal.emit("connected")  # 连接成功

            # 初始检测是否有数据
            initial_check = True
            check_timer = 0

            while self.running:
                msg = self.can_bus.recv(timeout=0.1)
                if msg:
                    self.last_message_time = datetime.now()
                    self.status_signal.emit("active")  # 数据活跃
                    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    self.raw_data.emit(f"[{timestamp}] ID:{msg.arbitration_id:04X} Data:{msg.data.hex()}")

                    if msg.arbitration_id in [0x60A, 0x60B]:
                        target = self.parse_message(msg.data)
                        if target:
                            self.new_target.emit(target)
                    elif msg.arbitration_id == 0x201:  # 假设0x201为雷达状态消息
                        status = self.parse_radar_status(msg.data)
                        self.radar_status.emit(status)

                    # 如果初始检测时收到数据，取消初始检测状态
                    if initial_check:
                        initial_check = False
                else:
                    self.status_signal.emit("inactive")  # 无数据
                    # 每5秒检查一次是否有数据
                    check_timer += 1
                    if check_timer >= 50:  # 0.1s * 50 = 5s
                        check_timer = 0
                        if initial_check or (self.last_message_time and
                                           (datetime.now() - self.last_message_time).total_seconds() > 5):
                            self.no_data.emit()

        except Exception as e:
            print(f"CAN Error:", e)
            self.status_signal.emit("error")  # 错误状态
            self.no_data.emit()
        finally:
            if self.can_bus:
                self.can_bus.shutdown()

    def parse_message(self, data):
        try:
            target_id = data[0]
            distance = (data[1] << 5 | (data[2] & 0xF8)) * 0.2 - 500
            angle = np.deg2rad((data[5] << 8 | data[6]) * 0.1 - 180)
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            return [x, y, 0, target_id]
        except:
            return None
            
    def parse_radar_status(self, data):
        """解析雷达状态信息"""
        return {
            'temperature': data[0] - 40,  # 温度 (°C)
            'voltage': data[1] * 0.1,     # 电压 (V)
            'error_code': data[2],         # 错误代码
            'output_type': data[3] & 0x0F  # 输出类型
        }