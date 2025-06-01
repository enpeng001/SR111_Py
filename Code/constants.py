# constants.py
from PyQt5 import QtGui

# 定义一些全局的样式常量
TITLE_FONT = QtGui.QFont("Arial", 14, QtGui.QFont.Bold)
LABEL_FONT = QtGui.QFont("Arial", 12)
BUTTON_STYLE = "QPushButton { background-color: #4CAF50; color: white; padding: 8px 16px; border-radius: 4px; }" \
               "QPushButton:hover { background-color: #45a049; }"
COMBOBOX_STYLE = "QComboBox { padding: 6px; border: 1px solid #ccc; border-radius: 4px; }"
TEXTEDIT_STYLE = "QTextEdit { border: 1px solid #ccc; border-radius: 4px; padding: 6px; }"
SLIDER_STYLE = "QSlider::handle:horizontal { background: #4CAF50; width: 18px; border-radius: 9px; }" \
               "QSlider::groove:horizontal { background: #ddd; height: 8px; border-radius: 4px; }"