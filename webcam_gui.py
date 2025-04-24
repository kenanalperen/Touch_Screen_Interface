import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""

import sys
import cv2
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

class CameraFeed(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Camera Feed")
        self.image_label = QLabel()
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)

        self.capture = cv2.VideoCapture("/dev/video4")  # Change number to match rgb of depth camera
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        ret, frame = self.capture.read()
        # for debugging: print("Frame captured:", ret)
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Make sure it's RGB
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.capture.release()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraFeed()
    window.show()
    sys.exit(app.exec_())


