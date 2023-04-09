#!/usr/bin/env python
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rospy
import std_msgs
import yaml
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor
from ampcl.io import save_pointcloud
from ampcl.ros import (pointcloud2_to_xyzi_array,
                       xyzi_numpy_to_pointcloud2)
from cv_bridge import CvBridge
from easydict import EasyDict
from sensor_msgs.msg import Image, PointCloud2


class DataCollection(QObject):
    save_signal = pyqtSignal(bool)

    def __init__(self, parent=None):
        super(DataCollection, self).__init__(parent)

        self.parent = parent

        with open('data_collection.yaml', 'r') as f:
            yaml_config = yaml.load(f, Loader=yaml.FullLoader)
            dot_config = EasyDict(yaml_config)

        # ChessboardParam
        self.corner_num_along_left_side = dot_config.ChessboardParam.CornerNumAlongLeftSide
        self.corner_num_along_bottom_side = dot_config.ChessboardParam.CornerNumAlongBottomSide
        self.chessboard_pattern = (self.corner_num_along_left_side, self.corner_num_along_bottom_side)

        # ROSParam
        self.lidar_topic = dot_config.ROSParam.lidar_topic
        self.img_topic = dot_config.ROSParam.img_topic
        self.overlap_frame = dot_config.ROSParam.overlap_frame
        self.lidar_frame = dot_config.ROSParam.lidar_frame

        # CameraParam
        self.do_horizon_flip = dot_config.CameraParam.do_horizon_flip
        self.do_vertical_flip = dot_config.CameraParam.do_vertical_flip
        self.do_rgb2bgr = dot_config.CameraParam.do_rgb2bgr

        self.bridge = CvBridge()
        self.current_frame = 0
        self.pointcloud_collection = None
        self.pointcloud_pub = rospy.Publisher(
            "/accumulated_pointcloud", PointCloud2, queue_size=1
        )

        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb, queue_size=1)
        self.img_msg = None

        self.item = 0
        self.item_for_instrinsic = 0
        self.save_dir = (Path() / "../data" / datetime.now().strftime('%m-%d-%H-%M-%S')).resolve()
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.pointcloud_dir = self.save_dir / "pointcloud"
        self.img_dir = self.save_dir / "img"
        self.img_dir_for_intrinsic = self.save_dir / "img_for_intrinsic"
        self.pointcloud_dir.mkdir(parents=True, exist_ok=True)
        self.img_dir.mkdir(parents=True, exist_ok=True)
        self.img_dir_for_intrinsic.mkdir(parents=True, exist_ok=True)

    def pointcloud_cb(self, pointcloud_msg):
        pointcloud_np = pointcloud2_to_xyzi_array(pointcloud_msg, remove_nans=True).astype(np.float32)

        if self.current_frame >= self.overlap_frame:
            self.pointcloud_sub.unregister()
            self.save_signal.emit(True)
            return

        if self.pointcloud_collection is None:
            self.pointcloud_collection = pointcloud_np
        else:
            self.pointcloud_collection = np.append(self.pointcloud_collection, pointcloud_np, axis=0)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.lidar_frame
        pointcloud_msg = xyzi_numpy_to_pointcloud2(self.pointcloud_collection, header)
        self.pointcloud_pub.publish(pointcloud_msg)
        self.current_frame = self.current_frame + 1

    def img_cb(self, img_msg):
        self.img_msg = img_msg

    @pyqtSlot()
    def do_subscribe_data(self):
        self.pointcloud_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.pointcloud_cb, queue_size=1)

    def save_pc_img(self, message):

        if self.pointcloud_collection is None or self.img_msg is None:
            message.setTextColor(QColor(255, 0, 0))
            message.append(" O_o no image or pointcloud can be saved")
            message.setTextColor(QColor(0, 0, 0))
            message.append("---------")
            return

        save_pointcloud(self.pointcloud_collection, file_name=f"{str(self.pointcloud_dir)}/{self.item:0>6d}.pcd")

        img_cv2 = self.bridge.imgmsg_to_cv2(self.img_msg, "passthrough")
        img_np = self.img_preprocess(img_cv2)
        cv2.imwrite(f"{str(self.img_dir)}/{self.item:0>6d}.png", img_np)
        message.setTextColor(QColor(0, 255, 0))
        message.append(f" O_o save {self.item}-th pair of data")
        message.setTextColor(QColor(0, 0, 0))
        message.append("---------")
        self.item = self.item + 1
        self.current_frame = 0
        self.pointcloud_collection = None

    def img_preprocess(self, img_cv2):
        if self.do_rgb2bgr:
            img_cv2 = cv2.cvtColor(img_cv2, cv2.COLOR_RGB2BGR)
        if self.do_horizon_flip:
            img_cv2 = cv2.flip(img_cv2, 1)
        if self.do_vertical_flip:
            img_cv2 = cv2.flip(img_cv2, 0)
        return img_cv2

    def save_img(self, message):

        if self.img_msg is not None:
            img_cv2 = self.bridge.imgmsg_to_cv2(self.img_msg, "passthrough")
            img_np = self.img_preprocess(img_cv2)
            cv2.imwrite(f"{str(self.img_dir_for_intrinsic)}/{self.item_for_instrinsic:0>6d}.png", img_np)
        else:
            message.setTextColor(QColor(255, 0, 0))
            message.append(" O_o no image can be saved")
            message.setTextColor(QColor(0, 0, 0))
            message.append("---------")
            return
        message.setTextColor(QColor(0, 255, 0))
        message.append(f" O_o save {self.item_for_instrinsic}-th image...")
        message.append("---")
        self.item_for_instrinsic = self.item_for_instrinsic + 1

    def reset(self):
        self.current_frame = 0
        self.pointcloud_collection = None


if __name__ == "__main__":
    rospy.init_node('data_collection', log_level=rospy.INFO)
    data_collection = DataCollection()
    rospy.spin()
