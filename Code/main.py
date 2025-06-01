# main.py
import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from radar_gui import RadarGUI

if __name__ == "__main__":
    # 确保中文显示正常
    import matplotlib
    matplotlib.use('Agg')  # 在Linux下使用Agg后端
    
    app = QtWidgets.QApplication(sys.argv)
    window = RadarGUI()
    window.show()
    
    # 创建定时器，定时刷新图表
    timer = QTimer()
    timer.timeout.connect(window.refresh_plots)
    timer.start(50)  # 50ms刷新周期
    
    sys.exit(app.exec_())