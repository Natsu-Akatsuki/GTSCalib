#!/usr/bin/env python3
import sys
from pathlib import Path

import rospy
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from rviz import bindings as rviz


class RvizWidget(QWidget):
    def __init__(self, x, y, w, h, parent=None):
        super().__init__(parent)
        # rviz.OgreLogging.useRosLog()
        self.frame = rviz.VisualizationFrame()  # 实例化rviz
        self.frame.initialize()  # 初始化rviz，构建关键对象 VisualizationManager

        self.load_rviz_cfg()
        self.set_rviz_attr()

        layout = QVBoxLayout()
        layout.addWidget(self.frame)

        self.setLayout(layout)
        self.setGeometry(x, y, w, h)
        self.full_screen_flag = False

    def set_rviz_attr(self):
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(True)

    def load_rviz_cfg(self):
        """ 读取rviz配置文件 """
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        self.cfg_path = str(Path(__file__).parent / "IRCal.rviz")
        reader.readFile(config, self.cfg_path)
        self.frame.load(config)

    def save_rviz_cfg(self):
        """保存rviz配置文件"""
        self.frame.saveDisplayConfig(self.cfg_path)
        rospy.loginfo(f"\033[1;32msave the rviz configuration to: {self.cfg_path}\033[0m")

    def full_screen(self):
        self.full_screen_flag = not self.full_screen_flag
        self.frame.setFullScreen(self.full_screen_flag)
