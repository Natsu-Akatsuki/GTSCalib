import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QDialog, QLabel, QVBoxLayout


class ImgWithChessboardUI(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Corners detection in image")
        self.resize(900, 300)
        self.setMinimumSize(400, 200)
        layout = QVBoxLayout()
        self.label_img = QLabel(None)
        layout.addWidget(self.label_img)
        self.setLayout(layout)
        self.bridge = CvBridge()

    def toggle_visualize(self, img_np):
        scale_percent = 50
        width = int(img_np.shape[1] * scale_percent / 100)
        height = int(img_np.shape[0] * scale_percent / 100)
        dim = (width, height)

        qim = cv2.resize(img_np, dim)
        qim_h, qim_w, qim_d = qim.shape
        qim = QImage(qim.data, qim_w, qim_h, qim_w * qim_d, QImage.Format_RGB888)
        self.label_img.setPixmap(QPixmap.fromImage(qim))
        self.show()
