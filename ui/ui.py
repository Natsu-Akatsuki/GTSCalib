#!/usr/bin/env python
import math
import sys

import cv2
import numpy as np
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QAction, QMainWindow
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import utils
from data_collectiton import DataCollection
from img_chessboard_ui import ImgWithChessboardUI
from main_ui import Ui_MainWindow
from rviz_ui import RvizWidget
from timer_message_box import TimerMessageBox


class UI(QMainWindow, Ui_MainWindow):

    def __init__(self, business_logic):
        super(UI, self).__init__()
        self.setupUi(self)
        self.business_logic = business_logic

        self.signal_connect_slot()
        self.add_widgets()
        self.add_action()

        self.img_np = None
        self.bridge = CvBridge()
        self.img_topic = self.business_logic.img_topic
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb, queue_size=1)

        self.frame_id = self.business_logic.lidar_frame
        self.markers_pub = rospy.Publisher('/markers', MarkerArray, queue_size=10, latch=True)
        self.markers = MarkerArray()
        self.paint_circle()
        self.paint_text()
        self.markers_pub.publish(self.markers)

    def paint_circle(self):
        for circle_id in range(15):
            circle = Marker()

            # meta information
            circle.header.frame_id = self.frame_id
            circle.header.stamp = rospy.Time.now()
            circle.ns = "circle"
            circle.id = circle_id
            circle.action = Marker.ADD
            circle.type = Marker.LINE_STRIP
            circle.lifetime = rospy.Duration(0)
            distance_delta = 1  # meter
            # geometry information
            self.set_orientation(circle)
            for theta in np.arange(0, 2 * 3.24, 0.1):
                point = Point()
                point.x = circle_id * distance_delta * math.cos(theta)
                point.y = circle_id * distance_delta * math.sin(theta)
                point.z = 0
                circle.points.append(point)
            # color information
            self.set_color(circle, (0.5, 0.5, 0.5, 0.9))
            # style information
            circle.scale.x = 0.1  # line width

            self.markers.markers.append(circle)

    def paint_text(self):
        for text_id in range(15):
            text = Marker()

            # meta information
            text.header.frame_id = self.frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = "text"
            text.id = text_id
            text.action = Marker.ADD
            text.type = Marker.TEXT_VIEW_FACING
            text.lifetime = rospy.Duration(0)
            distance_delta = 1  # meter
            # geometry information
            theta = -45 * math.pi / 180
            text.pose.position.x = (text_id * distance_delta) * math.cos(theta)
            text.pose.position.y = (text_id * distance_delta) * math.sin(theta)
            text.pose.position.z = 0
            # color information
            self.set_color(text, (1, 1, 1, 0.9))
            # style information
            text.scale.z = 1.0  # font size
            text.text = f"{text_id * distance_delta}"

            self.markers.markers.append(text)

    def create_point(self, position):
        p = Point()
        p.x = position[0]
        p.y = position[1]
        p.z = position[2]
        return p

    def set_color(self, marker, rgba):
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]  # Required, otherwise rviz can not be displayed

    def set_orientation(self, marker):
        marker.pose.orientation.x = 0.0  # suppress the uninitialized quaternion, assuming identity warning
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

    @pyqtSlot()
    def is_img_available(self):
        self.btn_is_img_ready.setEnabled(False)
        if self.img_np is None:
            TimerMessageBox("Img is unavailable", color='red', count=4).exec_()
            self.btn_is_img_ready.setEnabled(True)
            return

        gray = cv2.cvtColor(self.img_np, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray, self.business_logic.chessboard_pattern, None)

        if ret:
            TimerMessageBox("Corners can be detected in image\nCurrent position is available", color='green',
                            count=4).exec_()
            img_with_chessboard = np.copy(self.img_np)
            img_with_chessboard = cv2.drawChessboardCorners(img_with_chessboard, self.business_logic.chessboard_pattern, corners, ret)
            self.img_with_chessboard_popup.toggle_visualize(img_with_chessboard)
        else:
            TimerMessageBox("Corners can't be detected in image\nCurrent position is unavailable", color='red',
                            count=4).exec_()
        self.btn_is_img_ready.setEnabled(True)

    def img_cb(self, img_msg):
        img_cv2 = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        if self.business_logic.do_rgb2bgr:
            img_cv2 = cv2.cvtColor(img_cv2, cv2.COLOR_RGB2BGR)
        if self.business_logic.do_horizon_flip:
            img_cv2 = cv2.flip(img_cv2, 1)
        if self.business_logic.do_vertical_flip:
            img_cv2 = cv2.flip(img_cv2, 0)

        self.img_np = img_cv2.copy()
        qim = cv2.resize(img_cv2, (359, 283))
        qim_h, qim_w, qimd = qim.shape
        qim = QImage(qim.data, qim_w, qim_h, qim_w * qimd, QImage.Format_RGB888)
        self.canvas_img.setPixmap(QPixmap.fromImage(qim))

    def signal_connect_slot(self):
        self.btn_record_pc_img.clicked.connect(self.popup_do_subscribe_data)
        self.btn_record_img.clicked.connect(self.popup_save_img)
        self.btn_is_img_ready.clicked.connect(self.is_img_available)
        self.business_logic.save_signal.connect(self.popup_save_pc_img)

    def add_widgets(self):
        self.rviz = RvizWidget(10, 30, 1000, 850, self)
        self.img_with_chessboard_popup = ImgWithChessboardUI(self)

    @pyqtSlot()
    def popup_do_subscribe_data(self):
        result = QtWidgets.QMessageBox.question(self,
                                                "Confirm",
                                                "Confirm to record？",
                                                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if result == QtWidgets.QMessageBox.Yes:
            self.btn_record_pc_img.setEnabled(False)
            self.business_logic.do_subscribe_data()

    @pyqtSlot()
    def popup_save_img(self):
        result = QtWidgets.QMessageBox.question(self,
                                                "Confirm",
                                                "Confirm to save？",
                                                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if result == QtWidgets.QMessageBox.Yes:
            self.business_logic.save_img(self.canvas_log)

        self.btn_record_img.setEnabled(True)

    @pyqtSlot()
    def popup_save_pc_img(self):
        result = QtWidgets.QMessageBox.question(self,
                                                "Confirm",
                                                "Confirm to save？",
                                                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if result == QtWidgets.QMessageBox.Yes:
            self.business_logic.save_pc_img(self.canvas_log)
        else:
            self.business_logic.reset()

        self.btn_record_pc_img.setEnabled(True)

    def add_action(self):
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("&File")
        view_menu = menu_bar.addMenu("&View")

        temp_action = QAction("Save Rviz Config", parent=self, shortcut="Ctrl+S", enabled=True,
                              triggered=self.rviz.save_rviz_cfg)
        file_menu.addAction(temp_action)

        temp_action = QAction("Close", parent=self, shortcut=Qt.Key_Escape, enabled=True, triggered=self.close)
        view_menu.addAction(temp_action)

    def closeEvent(self, event):
        messagebox = QtWidgets.QMessageBox()
        result = messagebox.question(self,
                                     "Confirm",
                                     "Confirm to exit？",
                                     QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if result == QtWidgets.QMessageBox.Yes:
            self.showMinimized()
            event.accept()
        else:
            event.ignore()


if __name__ == '__main__':
    utils.launch_roscore()
    rospy.init_node('data_collection', log_level=rospy.INFO)
    app = QtWidgets.QApplication(sys.argv)
    QtWidgets.QApplication.font()

    # create a business thread
    qthread = QThread()
    qthread.start()
    business_logic = DataCollection()
    business_logic.moveToThread(qthread)

    main_window = UI(business_logic=business_logic)
    main_window.show()
    app.exec_()
