# SR111雷达监控系统

## 系统要求
- Windows或Linux系统
- Python 3.7+
- 标准CAN卡(PCAN/Kvaser)

## Linux系统通信要求
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up


## 安装依赖
### Windows
```bash
pip install -r requirements.txt