# radar_gui.py
import sys
import json
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QMessageBox, QScrollArea, QSizePolicy
import numpy as np
from constants import TITLE_FONT, LABEL_FONT, BUTTON_STYLE, COMBOBOX_STYLE, TEXTEDIT_STYLE, SLIDER_STYLE
from point_cloud_viewer import PointCloudViewer
from radar_worker import RadarWorker
from radar_config import RadarConfig

class RadarGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SR111 PCAN/Kvaser/SocketCAN 500KB上位机")
        self.resize(1400, 800)
        self.point_cloud = PointCloudViewer()
        self.init_ui()
        self.points_3d = []  # 存储3D点云数据
        self.points_2d = []  # 存储2D点云数据
        self.target_tracks = {}  # 目标轨迹 {target_id: [ (x, y) ]}
        self.track_length = 50  # 每条轨迹最大点数
        self.data_received = False
        self.original_raw_text = ""  # 用于保存原始的CAN报文数据
        self.radar_config = None
        self.alarm_zones = []  # 报警区域列表 [(x1,y1,x2,y2)]
        self.alarm_rects = []  # 存储报警区域ROI对象
        self.alarm_active = False
        self.track_visible = False  # 轨迹是否显示（默认不显示）

    def init_ui(self):
        # 创建菜单栏
        menubar = self.menuBar()
        menubar.setFont(TITLE_FONT)
        file_menu = menubar.addMenu("文件")
        save_data_action = QtWidgets.QAction("保存数据", self)
        save_data_action.triggered.connect(self.save_data)
        save_config_action = QtWidgets.QAction("保存配置", self)
        save_config_action.triggered.connect(self.save_config)
        load_config_action = QtWidgets.QAction("加载配置", self)
        load_config_action.triggered.connect(self.load_config)
        file_menu.addAction(save_data_action)
        file_menu.addAction(save_config_action)
        file_menu.addAction(load_config_action)
        
        view_menu = menubar.addMenu("视图")
        self.toggle_tracks_action = QtWidgets.QAction("显示目标轨迹", self, checkable=True, checked=False)
        self.toggle_tracks_action.triggered.connect(self.toggle_tracks)
        view_menu.addAction(self.toggle_tracks_action)
        
        help_menu = menubar.addMenu("帮助")
        about_action = QtWidgets.QAction("关于", self)
        about_action.triggered.connect(self.show_about_dialog)
        help_menu.addAction(about_action)

        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # 左侧面板容器（带滚动条）
        left_scroll_area = QScrollArea()
        left_scroll_area.setWidgetResizable(True)
        left_scroll_area.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        left_scroll_area.setMinimumWidth(350)
        
        left_container = QtWidgets.QWidget()
        left_scroll_area.setWidget(left_container)
        left_layout = QtWidgets.QVBoxLayout(left_container)
        left_layout.setContentsMargins(5, 5, 15, 5)
        left_layout.setSpacing(10)
        
        # 控制与调试区域
        control_group = QtWidgets.QGroupBox("控制与调试")
        control_group.setFont(TITLE_FONT)
        control_group_layout = QtWidgets.QVBoxLayout(control_group)
        control_group_layout.setContentsMargins(10, 15, 10, 15)
        
        # 通道选择与启停按钮
        channel_box = QtWidgets.QWidget()
        channel_layout = QtWidgets.QHBoxLayout(channel_box)
        channel_layout.setContentsMargins(0, 0, 0, 0)
        channel_label = QtWidgets.QLabel("CAN通道选择:")
        channel_label.setFont(LABEL_FONT)
        channel_layout.addWidget(channel_label)
        self.channel_combo = QtWidgets.QComboBox()
        self.channel_combo.setStyleSheet(COMBOBOX_STYLE)
        self.channel_combo.setMinimumWidth(150)
        # 根据操作系统添加不同的CAN通道选项
        if sys.platform.startswith('linux'):
            self.channel_combo.addItems(['can0', 'can1', 'vcan0'])
        else:  # Windows
            self.channel_combo.addItems([
                'PCAN_USBBUS1', 'PCAN_USBBUS2', 'PCAN_USBBUS3',
                'Kvaser_0', 'Kvaser_1', 'Kvaser_2'
            ])
        self.btn_toggle = QtWidgets.QPushButton("启 动")
        self.btn_toggle.setStyleSheet(BUTTON_STYLE)
        self.btn_toggle.setCheckable(True)
        self.btn_toggle.clicked.connect(self.toggle_can_connection)

        # 状态指示灯
        self.status_light = QtWidgets.QLabel()
        self.status_light.setFixedSize(20, 20)
        self.set_status_color("disconnected")

        # 添加到布局
        status_box = QtWidgets.QHBoxLayout()
        status_box.setContentsMargins(0, 5, 0, 5)
        status_label = QtWidgets.QLabel("通信状态:")
        status_label.setFont(LABEL_FONT)
        status_box.addWidget(status_label)
        status_box.addWidget(self.status_light)
        status_box.addStretch()

        channel_layout.addWidget(self.channel_combo)
        channel_layout.addWidget(self.btn_toggle)
        control_group_layout.addWidget(channel_box)
        control_group_layout.addLayout(status_box)

        # 数据保存按钮
        self.btn_save = QtWidgets.QPushButton("保存数据")
        self.btn_save.setStyleSheet(BUTTON_STYLE)
        self.btn_save.clicked.connect(self.save_data)
        control_group_layout.addWidget(self.btn_save)

        # 数据过滤框
        filter_box = QtWidgets.QWidget()
        filter_layout = QtWidgets.QHBoxLayout(filter_box)
        filter_layout.setContentsMargins(0, 0, 0, 0)
        filter_label = QtWidgets.QLabel("过滤条件 (CAN ID):")
        filter_label.setFont(LABEL_FONT)
        filter_layout.addWidget(filter_label)
        self.filter_input = QtWidgets.QLineEdit()
        self.filter_input.setStyleSheet(TEXTEDIT_STYLE)
        self.filter_input.setMinimumWidth(100)
        self.btn_filter = QtWidgets.QPushButton("过滤")
        self.btn_filter.setStyleSheet(BUTTON_STYLE)
        self.btn_filter.clicked.connect(self.apply_filter)
        filter_layout.addWidget(self.filter_input)
        filter_layout.addWidget(self.btn_filter)
        control_group_layout.addWidget(filter_box)

        # 可视化参数调整
        size_box = QtWidgets.QWidget()
        size_layout = QtWidgets.QHBoxLayout(size_box)
        size_layout.setContentsMargins(0, 0, 0, 0)
        size_label = QtWidgets.QLabel("点大小:")
        size_label.setFont(LABEL_FONT)
        size_layout.addWidget(size_label)
        self.size_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.size_slider.setStyleSheet(SLIDER_STYLE)
        self.size_slider.setRange(1, 20)
        self.size_slider.setValue(10)
        self.size_slider.setMinimumWidth(100)
        self.size_slider.valueChanged.connect(self.update_point_size)
        size_layout.addWidget(self.size_slider)
        control_group_layout.addWidget(size_box)

        # 原始数据框
        raw_label = QtWidgets.QLabel("原始CAN报文:")
        raw_label.setFont(LABEL_FONT)
        control_group_layout.addWidget(raw_label)
        self.raw_text = QtWidgets.QTextEdit()
        self.raw_text.setStyleSheet(TEXTEDIT_STYLE + " font-size: 10px;")
        self.raw_text.setReadOnly(True)
        self.raw_text.setMinimumHeight(150)
        control_group_layout.addWidget(self.raw_text, 1)
        
        # 雷达配置区域
        config_group = QtWidgets.QGroupBox("雷达配置")
        config_group.setFont(TITLE_FONT)
        config_layout = QtWidgets.QVBoxLayout(config_group)
        config_layout.setContentsMargins(10, 15, 10, 15)

        # 配置项布局函数
        def create_config_row(label_text, widget):
            row = QtWidgets.QWidget()
            layout = QtWidgets.QHBoxLayout(row)
            layout.setContentsMargins(0, 5, 0, 5)
            label = QtWidgets.QLabel(label_text)
            label.setFont(LABEL_FONT)
            layout.addWidget(label)
            layout.addWidget(widget)
            return row

        # 距离范围配置
        self.distance_combo = QtWidgets.QComboBox()
        self.distance_combo.setStyleSheet(COMBOBOX_STYLE)
        self.distance_combo.addItems(["15m", "25m", "50m", "70m"])
        self.distance_combo.currentIndexChanged.connect(self.on_distance_changed)
        config_layout.addWidget(create_config_row("探测距离范围:", self.distance_combo))

        # 角度分辨率配置
        self.resolution_combo = QtWidgets.QComboBox()
        self.resolution_combo.setStyleSheet(COMBOBOX_STYLE)
        self.resolution_combo.addItems(["0.4°", "0.2°"])
        self.resolution_combo.currentIndexChanged.connect(self.on_resolution_changed)
        config_layout.addWidget(create_config_row("角度分辨率:", self.resolution_combo))

        # 测量模式配置
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.setStyleSheet(COMBOBOX_STYLE)
        self.mode_combo.addItems(["标准", "高精度", "远距离"])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        config_layout.addWidget(create_config_row("测量模式:", self.mode_combo))

        # 传感器ID配置
        self.id_spin = QtWidgets.QSpinBox()
        self.id_spin.setRange(0, 255)
        self.id_spin.setValue(0)
        self.id_spin.setStyleSheet(TEXTEDIT_STYLE)
        self.id_spin.setMaximumWidth(80)
        self.id_spin.valueChanged.connect(self.on_id_changed)
        config_layout.addWidget(create_config_row("传感器ID:", self.id_spin))

        # 输出模式配置
        self.output_combo = QtWidgets.QComboBox()
        self.output_combo.setStyleSheet(COMBOBOX_STYLE)
        self.output_combo.addItems([
            "仅聚类", "仅目标", "聚类和目标", "仅聚类质量", 
            "仅目标质量", "聚类和目标质量", "扩展目标", "点云"
        ])
        self.output_combo.currentIndexChanged.connect(self.on_output_changed)
        config_layout.addWidget(create_config_row("输出模式:", self.output_combo))

        # 更新速率配置
        self.rate_combo = QtWidgets.QComboBox()
        self.rate_combo.setStyleSheet(COMBOBOX_STYLE)
        self.rate_combo.addItems(["10", "20", "25", "33", "50"])
        self.rate_combo.setCurrentText("20")
        self.rate_combo.currentIndexChanged.connect(self.on_rate_changed)
        config_layout.addWidget(create_config_row("更新速率 (Hz):", self.rate_combo))

        # RCS阈值配置
        rcs_box = QtWidgets.QWidget()
        rcs_layout = QtWidgets.QHBoxLayout(rcs_box)
        rcs_layout.setContentsMargins(0, 0, 0, 0)
        rcs_label = QtWidgets.QLabel("RCS阈值:")
        rcs_label.setFont(LABEL_FONT)
        self.rcs_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.rcs_slider.setStyleSheet(SLIDER_STYLE)
        self.rcs_slider.setRange(0, 15)
        self.rcs_slider.setValue(0)
        self.rcs_slider.setMinimumWidth(100)
        self.rcs_slider.valueChanged.connect(self.on_rcs_changed)
        self.rcs_value = QtWidgets.QLabel("0")
        self.rcs_value.setFixedWidth(30)
        rcs_layout.addWidget(rcs_label)
        rcs_layout.addWidget(self.rcs_slider)
        rcs_layout.addWidget(self.rcs_value)
        config_layout.addWidget(rcs_box)
        
        # 点云滤波
        cloud_filter_box = QtWidgets.QWidget()
        cloud_filter_layout = QtWidgets.QHBoxLayout(cloud_filter_box)
        cloud_filter_layout.setContentsMargins(0, 0, 0, 0)
        cloud_filter_label = QtWidgets.QLabel("点云滤波:")
        cloud_filter_label.setFont(LABEL_FONT)
        self.cloud_filter_check = QtWidgets.QCheckBox("启用")
        self.cloud_filter_check.stateChanged.connect(self.toggle_cloud_filter)
        self.cloud_filter_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.cloud_filter_slider.setStyleSheet(SLIDER_STYLE)
        self.cloud_filter_slider.setRange(0.2, 70)
        self.cloud_filter_slider.setValue(70)
        self.cloud_filter_slider.setMinimumWidth(70)
        self.cloud_filter_slider.valueChanged.connect(self.update_cloud_filter)
        self.cloud_filter_value = QtWidgets.QLabel("70m")
        cloud_filter_layout.addWidget(cloud_filter_label)
        cloud_filter_layout.addWidget(self.cloud_filter_check)
        cloud_filter_layout.addWidget(self.cloud_filter_slider)
        cloud_filter_layout.addWidget(self.cloud_filter_value)
        config_layout.addWidget(cloud_filter_box)

        # 应用配置按钮
        self.apply_config_btn = QtWidgets.QPushButton("应用配置")
        self.apply_config_btn.setStyleSheet(BUTTON_STYLE)
        self.apply_config_btn.clicked.connect(self.apply_config)
        config_layout.addWidget(self.apply_config_btn)
        
        # 配置保存和加载按钮
        config_btn_box = QtWidgets.QWidget()
        config_btn_layout = QtWidgets.QHBoxLayout(config_btn_box)
        config_btn_layout.setContentsMargins(0, 10, 0, 0)
        self.btn_save_config = QtWidgets.QPushButton("保存配置")
        self.btn_save_config.setStyleSheet(BUTTON_STYLE)
        self.btn_save_config.clicked.connect(self.save_config)
        self.btn_load_config = QtWidgets.QPushButton("加载配置")
        self.btn_load_config.setStyleSheet(BUTTON_STYLE)
        self.btn_load_config.clicked.connect(self.load_config)
        config_btn_layout.addWidget(self.btn_save_config)
        config_btn_layout.addWidget(self.btn_load_config)
        config_layout.addWidget(config_btn_box)

        # 报警区域
        alarm_group = QtWidgets.QGroupBox("报警区域")
        alarm_group.setFont(TITLE_FONT)
        alarm_layout = QtWidgets.QVBoxLayout(alarm_group)
        alarm_layout.setContentsMargins(10, 15, 10, 15)
        
        # 添加报警区域按钮行
        alarm_btn_row = QtWidgets.QWidget()
        alarm_btn_layout = QtWidgets.QHBoxLayout(alarm_btn_row)
        alarm_btn_layout.setContentsMargins(0, 0, 0, 0)
        
        self.btn_add_zone = QtWidgets.QPushButton("添加矩形区域")
        self.btn_add_zone.setStyleSheet(BUTTON_STYLE)
        self.btn_add_zone.clicked.connect(self.add_alarm_zone)
        alarm_btn_layout.addWidget(self.btn_add_zone)
        
        self.btn_remove_zone = QtWidgets.QPushButton("删除最近区域")
        self.btn_remove_zone.setStyleSheet(BUTTON_STYLE)
        self.btn_remove_zone.clicked.connect(self.remove_last_alarm_zone)
        alarm_btn_layout.addWidget(self.btn_remove_zone)
        
        self.btn_clear_zones = QtWidgets.QPushButton("清除所有区域")
        self.btn_clear_zones.setStyleSheet(BUTTON_STYLE)
        self.btn_clear_zones.clicked.connect(self.clear_all_alarm_zones)
        alarm_btn_layout.addWidget(self.btn_clear_zones)
        
        alarm_layout.addWidget(alarm_btn_row)
        
        # 清除轨迹按钮
        self.btn_clear_tracks = QtWidgets.QPushButton("清除目标轨迹")
        self.btn_clear_tracks.setStyleSheet(BUTTON_STYLE)
        self.btn_clear_tracks.clicked.connect(self.clear_target_tracks)
        alarm_layout.addWidget(self.btn_clear_tracks)

        # 报警状态指示灯
        alarm_status_box = QtWidgets.QHBoxLayout()
        alarm_status_label = QtWidgets.QLabel("报警状态:")
        alarm_status_label.setFont(LABEL_FONT)
        self.alarm_indicator = QtWidgets.QLabel()
        self.alarm_indicator.setFixedSize(20, 20)
        self.set_alarm_color(False)
        alarm_status_box.addWidget(alarm_status_label)
        alarm_status_box.addWidget(self.alarm_indicator)
        alarm_status_box.addStretch()
        alarm_layout.addLayout(alarm_status_box)

        # 添加到左侧布局
        left_layout.addWidget(control_group)
        left_layout.addWidget(config_group)
        left_layout.addWidget(alarm_group)
        left_layout.addStretch(1)
        
        # 右侧面板分割器（上下分割）
        right_splitter = QtWidgets.QSplitter(Qt.Vertical)
        right_splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 2D可视化
        self.plot_2d = pg.PlotWidget()
        self.plot_2d.setLabel('left', 'Y坐标 (m)')
        self.plot_2d.setLabel('bottom', 'X坐标 (m)')
        self.scatter_2d = pg.ScatterPlotItem(size=10)
        self.plot_2d.addItem(self.scatter_2d)
        
        # 存储轨迹线对象
        self.track_lines = []

        # 3D可视化
        self.point_cloud.setMinimumSize(400, 300)
        right_splitter.addWidget(self.plot_2d)
        right_splitter.addWidget(self.point_cloud)
        right_splitter.setSizes([500, 500])

        # 添加到主布局
        main_layout.addWidget(left_scroll_area, 1)
        main_layout.addWidget(right_splitter, 2)

    def set_status_color(self, status):
        color_map = {
            "disconnected": "gray",
            "connected": "#FFA500",  # 橙色（已连接无数据）
            "active": "#00FF00",  # 绿色（数据正常）
            "inactive": "#FF0000",  # 红色（无数据）
            "error": "#FFFF00"  # 黄色（错误）
        }
        self.status_light.setStyleSheet(
            f"background-color: {color_map[status]};"
            "border-radius: 10px; border: 2px solid darkgray;"
        )
        
    def set_alarm_color(self, active):
        color = "#FF0000" if active else "#00FF00"
        self.alarm_indicator.setStyleSheet(
            f"background-color: {color};"
            "border-radius: 10px; border: 2px solid darkgray;"
        )

    def toggle_can_connection(self):
        """切换CAN连接状态"""
        if self.btn_toggle.isChecked():
            self.set_status_color("connected")  # 启动时显示连接中
            self.start_radar()
            self.btn_toggle.setText("停 止")
            self.channel_combo.setEnabled(False)
        else:
            self.set_status_color("disconnected")  # 停止时恢复灰色
            self.stop_radar()
            self.btn_toggle.setText("启 动")
            self.channel_combo.setEnabled(True)

    def start_radar(self):
        """启动雷达线程"""
        if hasattr(self, 'radar_thread') and self.radar_thread.isRunning():
            return

        self.data_received = False
        self.radar_thread = RadarWorker(self.channel_combo.currentText())
        self.radar_thread.new_target.connect(self.update_data)
        self.radar_thread.raw_data.connect(self.update_raw_display)
        self.radar_thread.no_data.connect(self.show_no_data_warning)
        self.radar_thread.status_signal.connect(self.handle_status_change)
        self.radar_thread.radar_status.connect(self.update_radar_status)
        self.radar_thread.start()
        
        # 创建雷达配置实例
        self.radar_config = RadarConfig(self.radar_thread.can_bus)
        
        # 应用默认配置
        self.apply_config()

    def stop_radar(self):
        """停止雷达线程"""
        if hasattr(self, 'radar_thread'):
            self.radar_thread.running = False
            self.radar_thread.wait(1000)
            self.radar_thread.quit()

    def handle_status_change(self, status):
        if status == "connected":
            self.set_status_color("connected")
        elif status == "active":
            self.set_status_color("active")
        elif status in ("inactive", "error"):
            self.set_status_color("inactive")

    def update_data(self, point):
        self.data_received = True
        x, y, z, tid = point
        # 添加到当前帧
        self.points_2d.append({'x': x, 'y': y, 'tid': tid})
        self.points_3d.append([x, y, z])
        
        # 目标轨迹追踪
        if tid not in self.target_tracks:
            self.target_tracks[tid] = []
        self.target_tracks[tid].append((x, y))
        if len(self.target_tracks[tid]) > self.track_length:
            self.target_tracks[tid].pop(0)

    def refresh_plots(self):
        # 更新2D视图
        if self.points_2d:
            self.scatter_2d.setData(
                x=[p['x'] for p in self.points_2d],
                y=[p['y'] for p in self.points_2d],
                brush=pg.mkBrush('r')
            )
            
            # 清除之前的轨迹线
            for line in self.track_lines:
                self.plot_2d.removeItem(line)
            self.track_lines = []
            
            # 绘制目标轨迹（如果开启）
            if self.track_visible and self.target_tracks:
                for tid, track in self.target_tracks.items():
                    if len(track) > 1:
                        # 使用不同颜色区分不同目标
                        color = pg.intColor(tid % 10, hues=10, maxValue=200)
                        # 绘制轨迹线
                        line = self.plot_2d.plot(
                            [p[0] for p in track],
                            [p[1] for p in track],
                            pen=pg.mkPen(color, width=1),
                            connect="all"
                        )
                        self.track_lines.append(line)
            
            # 检查报警区域
            alarm_triggered = False
            for point in self.points_2d:
                x, y = point['x'], point['y']
                for zone in self.alarm_zones:
                    if zone[0] <= x <= zone[2] and zone[1] <= y <= zone[3]:
                        alarm_triggered = True
                        break
                if alarm_triggered:
                    break
            
            # 更新报警状态
            if alarm_triggered != self.alarm_active:
                self.alarm_active = alarm_triggered
                self.set_alarm_color(alarm_triggered)
                if alarm_triggered:
                    # 播放报警声音
                    QtWidgets.QApplication.beep()
            
            # 清空当前帧
            self.points_2d.clear()

        # 更新3D视图
        if self.points_3d:
            self.point_cloud.update_points(np.array(self.points_3d))
            self.points_3d.clear()

    def update_raw_display(self, text):
        self.data_received = True
        self.raw_text.append(text)
        if not self.original_raw_text:
            self.original_raw_text = text
        else:
            self.original_raw_text += '\n' + text
        if self.raw_text.document().blockCount() > 200:
            cursor = self.raw_text.textCursor()
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.select(QtGui.QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()

    def show_no_data_warning(self):
        msg = "未检测到CAN数据，请检查:\n"
        current_channel = self.channel_combo.currentText()
        if sys.platform.startswith('linux'):
            msg += "1. SocketCAN接口是否配置正确\n2. 通道是否已经启动（例如：sudo ip link set can0 up type can bitrate 500000）\n"
        elif "Kvaser" in current_channel:
            msg += "1. Kvaser设备驱动是否安装\n2. 设备是否被其他程序占用\n"
        else:
            msg += "1. PCAN设备是否正确连接\n"
        msg += "2. 通道选择是否正确\n3. 雷达设备是否上电并发送数据"
        QMessageBox.warning(self, "警告", msg)

    def show_about_dialog(self):
        about_text = (
            "<b>SR111 PCAN/Kvaser/SocketCAN 上位机</b><br><br>"
            "版本: 5.10<br>"
            "© 2025 上海德威兰机器人科技公司. 保留所有权利.<br><br>"
            "基于PyQt5和python-can开发的雷达数据可视化工具，支持PCAN、Kvaser和SocketCAN设备通信。<br>"
            "本程序已实现的功能如下：<br>"
            "1. 文件操作：<br>"
            "- 保存数据：可将当前雷达数据保存到文件。<br>"
            "- 保存配置：保存雷达的配置参数。<br>"
            "- 加载配置：从文件中加载之前保存的雷达配置参数。<br>"
            "2. 视图操作：<br>"
            "- 显示目标轨迹：可切换是否显示目标的轨迹。<br>"
            "3. 控制与调试：<br>"
            "- CAN通道选择：支持不同操作系统的CAN通道选择，如Linux的'socketcan'和Windows的'pcan'、'kvaser'。<br>"
            "- 启动/停止CAN通信：通过按钮控制CAN通信的启动和停止。<br>"
            "- 通信状态显示：实时显示CAN通信的状态（连接、活跃、无数据、错误）。<br>"
            "- 数据保存：手动保存雷达数据。<br>"
            "- 数据过滤：根据CAN ID对数据进行过滤。<br>"
            "- 可视化参数调整：可调整点云显示的点大小。<br>"
            "- 原始CAN报文显示：实时显示接收到的原始CAN报文。<br>"
            "4. 雷达配置：<br>"
            "- 探测距离范围配置：可选择不同的探测距离范围（15m、25m、50m、70m）。<br>"
            "- 角度分辨率配置：可选择不同的角度分辨率（0.4°、0.2°）。<br>"
            "- 测量模式配置：可选择标准、高精度、远距离等测量模式。<br>"
            "- 传感器ID配置：可设置雷达传感器的ID。<br>"
            "- 输出模式配置：可选择不同的输出模式（仅聚类、仅目标等）。<br>"
            "- 更新速率配置：可设置雷达数据的更新速率（10Hz、20Hz等）。<br>"
            "- RCS阈值配置：可调整RCS阈值。<br>"
            "许可证: MIT<br>"
            "联系方式: alen@devlanetech.com"
        )
        QMessageBox.about(self, "关于", about_text)

    def save_data(self):
        file_dialog = QtWidgets.QFileDialog()
        file_path, _ = file_dialog.getSaveFileName(self, "保存数据", "", "文本文件 (*.txt)")
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    # 保存原始 CAN 报文
                    f.write("原始 CAN 报文:\n")
                    f.write(self.original_raw_text)
                    # 保存解析后的目标数据
                    f.write("\n解析后的目标数据:\n")
                    # 这里可以添加保存解析后目标数据的逻辑
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"保存数据时出现错误: {e}")

    def apply_filter(self):
        filter_text = self.filter_input.text()
        if filter_text:
            try:
                filter_id = int(filter_text, 16)
                filtered_text = '\n'.join(
                    [line for line in self.original_raw_text.split('\n') if f"ID:{filter_id:04X}" in line])
                self.raw_text.setPlainText(filtered_text)
            except ValueError:
                QMessageBox.warning(self, "输入错误", "请输入有效的十六进制 CAN ID。")
        else:
            # 清空过滤条件，显示全部数据
            self.raw_text.setPlainText(self.original_raw_text)

    def update_point_size(self, size):
        self.scatter_2d.setSize(size)
        self.point_cloud.scatter.setData(size=size)

    def on_distance_changed(self, index):
        if self.radar_config:
            distances = [15, 25, 50, 70]  # 修正：按照组合框的顺序，单位是米
            self.radar_config.set_distance_range(distances[index])
            
    def on_resolution_changed(self, index):
        if self.radar_config:
            resolutions = [0.4, 0.2]
            self.radar_config.set_angle_resolution(resolutions[index])
            
    def on_mode_changed(self, index):
        if self.radar_config:
            self.radar_config.set_measurement_mode(index)
            
    def on_id_changed(self, value):
        if self.radar_config:
            self.radar_config.set_sensor_id(value)
            
    def on_output_changed(self, index):
        if self.radar_config:
            self.radar_config.set_output_mode(index)
            
    def on_rate_changed(self, index):
        if self.radar_config:
            rates = [10, 20, 25, 33, 50]  # 注意：组合框有5个选项，对应5个值
            self.radar_config.set_update_rate(rates[index])
            
    def on_rcs_changed(self, value):
        self.rcs_value.setText(str(value))
        if self.radar_config:
            self.radar_config.set_target_config(rcs_threshold=value)
            
    def apply_config(self):
        if not self.radar_config:
            return
            
        # 应用所有当前配置
        distances = [15, 25, 50, 70]
        self.radar_config.set_distance_range(distances[self.distance_combo.currentIndex()])
        
        resolutions = [0.4, 0.2]
        self.radar_config.set_angle_resolution(resolutions[self.resolution_combo.currentIndex()])
        
        self.radar_config.set_measurement_mode(self.mode_combo.currentIndex())
        self.radar_config.set_sensor_id(self.id_spin.value())
        self.radar_config.set_output_mode(self.output_combo.currentIndex())
        
        rates = [10, 20, 25, 33, 50]
        self.radar_config.set_update_rate(rates[self.rate_combo.currentIndex()])
        
        self.radar_config.set_cluster_config()
        self.radar_config.set_target_config(rcs_threshold=self.rcs_slider.value())
        
        # 显示配置成功消息
        self.raw_text.append("\n[配置] 已应用所有雷达配置参数")
        
    def update_radar_status(self, status):
        # 状态更新函数保留但不再使用
        pass
        
    def add_alarm_zone(self):
        """添加矩形报警区域"""
        # 默认在原点附近一个矩形
        zone = (-10, -5, 10, 5)  # (x_min, y_min, x_max, y_max)
        self.alarm_zones.append(zone)
        
        # 在2D图上绘制区域
        rect = pg.RectROI(
            [zone[0], zone[1]], 
            [zone[2]-zone[0], zone[3]-zone[1]],
            pen=pg.mkPen('r', width=2),
            movable=True,  # 允许用户移动区域
            resizable=True  # 允许用户调整大小
        )
        rect.setZValue(-10)  # 确保在点之下
        self.plot_2d.addItem(rect)
        self.alarm_rects.append(rect)  # 保存ROI对象以便后续删除
        
        self.raw_text.append(f"添加报警区域: X={zone[0]}~{zone[2]}m, Y={zone[1]}~{zone[3]}m")

    def remove_last_alarm_zone(self):
        """删除最近添加的报警区域"""
        if not self.alarm_zones:
            return
            
        # 移除最后一个区域
        zone = self.alarm_zones.pop()
        rect = self.alarm_rects.pop()
        self.plot_2d.removeItem(rect)
        
        self.raw_text.append(f"删除报警区域: X={zone[0]}~{zone[2]}m, Y={zone[1]}~{zone[3]}m")
        
    def clear_all_alarm_zones(self):
        """清除所有报警区域"""
        if not self.alarm_zones:
            return
            
        count = len(self.alarm_zones)
        # 移除所有区域
        while self.alarm_rects:
            rect = self.alarm_rects.pop()
            self.plot_2d.removeItem(rect)
        self.alarm_zones.clear()
        
        self.raw_text.append(f"已清除所有报警区域 ({count}个)")
        
    def clear_target_tracks(self):
        """清除所有目标轨迹"""
        # 清除轨迹线
        for line in self.track_lines:
            self.plot_2d.removeItem(line)
        self.track_lines = []
        
        # 清除轨迹数据
        if self.target_tracks:
            count = len(self.target_tracks)
            self.target_tracks.clear()
            self.raw_text.append(f"已清除所有目标轨迹 ({count}条)")
            # 刷新显示
            self.refresh_plots()
        else:
            self.raw_text.append("无目标轨迹可清除")

    def toggle_cloud_filter(self, state):
        self.point_cloud.filter_enabled = (state == Qt.Checked)
        
    def update_cloud_filter(self, value):
        self.point_cloud.filter_distance = value
        self.cloud_filter_value.setText(f"{value}m")
        
    def toggle_tracks(self):
        self.track_visible = self.toggle_tracks_action.isChecked()
        # 刷新显示
        self.refresh_plots()
        
    def save_config(self):
        config = {
            'distance': self.distance_combo.currentIndex(),
            'resolution': self.resolution_combo.currentIndex(),
            'mode': self.mode_combo.currentIndex(),
            'id': self.id_spin.value(),
            'output': self.output_combo.currentIndex(),
            'rate': self.rate_combo.currentIndex(),
            'rcs': self.rcs_slider.value(),
            'point_size': self.size_slider.value(),
            'cloud_filter_enabled': self.cloud_filter_check.isChecked(),
            'cloud_filter_distance': self.cloud_filter_slider.value(),
            'track_visible': self.toggle_tracks_action.isChecked(),
            'alarm_zones': self.alarm_zones  # 保存报警区域配置
        }
        
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存配置", "", "配置文件 (*.json)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(config, f)
                self.raw_text.append(f"\n[配置] 配置已保存到: {file_path}")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"保存配置时出错: {e}")
                
    def load_config(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载配置", "", "配置文件 (*.json)"
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    config = json.load(f)
                
                # 应用配置到UI
                self.distance_combo.setCurrentIndex(config['distance'])
                self.resolution_combo.setCurrentIndex(config['resolution'])
                self.mode_combo.setCurrentIndex(config['mode'])
                self.id_spin.setValue(config['id'])
                self.output_combo.setCurrentIndex(config['output'])
                self.rate_combo.setCurrentIndex(config['rate'])
                self.rcs_slider.setValue(config['rcs'])
                self.size_slider.setValue(config['point_size'])
                self.cloud_filter_check.setChecked(config['cloud_filter_enabled'])
                self.cloud_filter_slider.setValue(config['cloud_filter_distance'])
                self.toggle_tracks_action.setChecked(config['track_visible'])
                
                # 更新点云滤波
                self.toggle_cloud_filter(self.cloud_filter_check.checkState())
                self.update_cloud_filter(self.cloud_filter_slider.value())
                self.toggle_tracks()
                
                # 加载报警区域
                if 'alarm_zones' in config:
                    # 清除现有区域
                    self.clear_all_alarm_zones()
                    # 添加新区域
                    for zone in config['alarm_zones']:
                        self.alarm_zones.append(zone)
                        rect = pg.RectROI(
                            [zone[0], zone[1]], 
                            [zone[2]-zone[0], zone[3]-zone[1]],
                            pen=pg.mkPen('r', width=2),
                            movable=True,
                            resizable=True
                        )
                        rect.setZValue(-10)
                        self.plot_2d.addItem(rect)
                        self.alarm_rects.append(rect)
                    
                    self.raw_text.append(f"加载报警区域: {len(config['alarm_zones'])}个")
                
                # 应用配置到雷达
                self.apply_config()
                
                self.raw_text.append(f"\n[配置] 从 {file_path} 加载配置成功")
            except Exception as e:
                QMessageBox.warning(self, "加载失败", f"加载配置时出错: {e}")

    def closeEvent(self, event):
        self.stop_radar()
        super().closeEvent(event)