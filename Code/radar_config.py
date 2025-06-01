# radar_config.py
import can
from datetime import datetime

class RadarConfig:
    """SR111雷达配置参数管理类"""
    
    def __init__(self, can_bus):
        self.can_bus = can_bus
        self.sensor_id = 0
        self.distance_range = 70  # 默认70米
        self.angle_resolution = 0.4  # 默认0.4度
        self.measurement_mode = 0  # 默认标准模式
        self.output_mode = 1  # 默认仅输出目标
        self.update_rate = 20  # 默认20Hz
        self.rcs_threshold = 0  # 默认不过滤
        
    def set_sensor_id(self, id):
        """设置雷达传感器ID"""
        self.sensor_id = id
        self._send_config_message(0x200, [id & 0xFF, 0, 0, 0, 0, 0, 0, 0])
        
    def set_distance_range(self, range_meters):
        """设置雷达探测距离范围"""
        self.distance_range = range_meters
        # 计算对应的距离配置值
        range_values = {15: 0, 25: 1, 50: 2, 70: 3}
        range_code = range_values.get(range_meters, 1)
        self._send_config_message(0x201, [range_code, 0, 0, 0, 0, 0, 0, 0])
        
    def set_angle_resolution(self, resolution_degrees):
        """设置角度分辨率"""
        self.angle_resolution = resolution_degrees
        # 0.4° = 0, 0.2° = 1
        res_code = 0 if resolution_degrees >= 0.4 else 1
        self._send_config_message(0x202, [res_code, 0, 0, 0, 0, 0, 0, 0])
        
    def set_measurement_mode(self, mode):
        """设置测量模式: 0=标准, 1=高精度, 2=远距离"""
        self.measurement_mode = mode
        self._send_config_message(0x203, [mode, 0, 0, 0, 0, 0, 0, 0])
        
    def set_output_mode(self, mode):
        """设置输出模式: 0=仅聚类, 1=仅目标, 2=聚类和目标, 3=仅聚类质量, 
                        4=仅目标质量, 5=聚类和目标质量, 6=扩展目标, 7=点云"""
        self.output_mode = mode
        self._send_config_message(0x204, [mode, 0, 0, 0, 0, 0, 0, 0])
        
    def set_update_rate(self, rate_hz):
        """设置数据更新频率(Hz)"""
        self.update_rate = rate_hz
        # 计算对应的更新率配置值
        rate_values = {10: 0, 20: 1, 25: 2, 33: 3, 50: 4}
        rate_code = rate_values.get(rate_hz, 1)
        self._send_config_message(0x205, [rate_code, 0, 0, 0, 0, 0, 0, 0])
        
    def set_cluster_config(self):
        """设置聚类配置参数"""
        # 启用聚类，设置默认聚类参数
        data = [
            1,  # 聚类使能
            0,  # 聚类算法选择
            0,  # 聚类质量阈值
            0,  # 聚类距离阈值
            0,  # 聚类角度阈值
            0,  # 聚类最小点数
            0,  # 聚类最大点数
            0   # 保留
        ]
        self._send_config_message(0x206, data)
        
    def set_target_config(self, rcs_threshold=0):
        """设置目标配置参数"""
        self.rcs_threshold = rcs_threshold
        # RCS阈值: 0-15 (0=无过滤，15=最高过滤)
        data = [
            1,          # 目标使能
            0,          # 目标分类模式
            rcs_threshold,  # RCS阈值
            0,          # 最小相对速度
            0,          # 最大相对速度
            0,          # 最小距离
            0,          # 最大距离
            0           # 保留
        ]
        self._send_config_message(0x207, data)
        
    def _send_config_message(self, can_id, data):
        """发送配置消息到雷达"""
        if not self.can_bus:
            return
            
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False
            )
            self.can_bus.send(msg)
            timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 发送配置: ID=0x{can_id:03X}, Data={data}")
        except can.CanError as e:
            print(f"发送配置失败 (CAN错误): {e}")
        except Exception as e:
            print(f"发送配置失败: {e}")