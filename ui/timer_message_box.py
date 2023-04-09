from PyQt5 import QtGui
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QLabel, QMessageBox, QPushButton


class TimerMessageBox(QMessageBox):
    """
    a messagebox with gif and function of auto close
    inspired by @https://stackoverflow.com/questions/50023853/how-do-i-integrate-an-animated-gif-in-a-qmessagebox

    simple usage:
         TimerMessageBox("the global trajectory is replayed successfully ").exec_()
    """

    def __init__(self, text="", auto_close=True, time_=500, count=1, color='green'):
        super(TimerMessageBox, self).__init__(parent=None)
        self.text = text
        self.auto_close = auto_close
        self.time = time_
        self.count = count
        color_dict = {'red': '245, 0, 0',
                      'green': '0, 245, 0'}
        # remove button
        self.setStandardButtons(QMessageBox.NoButton)

        self.label = self.findChild(QLabel, "qt_msgbox_label")
        self.label.setAlignment(Qt.AlignCenter)
        font = QtGui.QFont()
        font.setBold(True)
        self.label.setFont(font)
        self.setStyleSheet(
            "QLabel#qt_msgbox_label{ color: white;" + f"background-color: rgbd({color_dict[color]}, 160); "
                                                      "border-style: outset; border-width: 1px; border-color: transparent; border-radius: 5px; }")
        self.setText(f'{text}')
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.timer = QTimer(self, timeout=self.do_count_down)
        self.timer.start(self.time)

    def do_count_down(self):
        self.count -= 1
        if self.count <= 0:
            self.timer.stop()
            if self.auto_close:
                self.accept()
                self.close()


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    btn = QPushButton('Click to Pop Up MessageBox')
    btn.resize(400, 200)
    btn.show()
    btn.clicked.connect(lambda: TimerMessageBox().exec_())
    app.exec_()
