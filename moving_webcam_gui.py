import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap, QCursor
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QPoint, Qt



class CameraThread(QThread):
    new_frame = pyqtSignal(np.ndarray)

    def __init__(self, device="/dev/video4"):
        super().__init__()
        self.device = device
        self.running = True

    def run(self):
        cap = cv2.VideoCapture(self.device)
        while self.running:
            ret, frame = cap.read()
            if ret and frame is not None:
                self.new_frame.emit(frame)
            self.msleep(30)  # ~33fps

        cap.release()

    def stop(self):
        self.running = False
        self.wait()


class CameraFeed(QWidget):
    def __init__(self):
        super().__init__()
        # self.setWindowTitle("Live Camera Feed")
        self.setWindowFlags(self.windowFlags() | Qt.FramelessWindowHint)


        self.image_label = QLabel()
        self.image_label.setFixedSize(1280, 960)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)

        # Start camera thread
        self.camera_thread = CameraThread()
        self.camera_thread.new_frame.connect(self.update_frame)
        self.camera_thread.start()

        # Start smoother, faster follow timer
        self.move_timer = QTimer()
        self.move_timer.timeout.connect(self.follow_mouse_fast)
        self.move_timer.start(5)  # Update every 5ms for better responsiveness

    def update_frame(self, frame):
        # Resize, then snip top 80px
        frame = cv2.resize(frame, (self.image_label.width(), self.image_label.height() + 80))
        frame = frame[80:, :]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def follow_mouse_fast(self):
        target = QCursor.pos() - QPoint(self.width() // 2, self.height()*3 // 4)
        current = self.pos()

        speed = 1.0  # higher = faster follow (0.3–0.5 for responsive, up to 1.0 for instant)

        new_x = int(current.x() + speed * (target.x() - current.x()))
        new_y = int(current.y() + speed * (target.y() - current.y()))
        self.move(new_x, new_y)

    def closeEvent(self, event):
        self.camera_thread.stop()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraFeed()
    window.show()
    sys.exit(app.exec_())
