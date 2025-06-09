#!/usr/bin/env python3

import sys
import time
import rospy
import cv2
import numpy as np
from threading import Lock, Thread
from collections import deque

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt5.QtGui import QImage, QPixmap, QFont, QMouseEvent


class ImageViewer(QWidget):
    new_image = pyqtSignal(np.ndarray)

    def __init__(self, mode="default"):
        super().__init__()
        self.setWindowTitle("Depth Image Viewer")
        self.mode = mode

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMouseTracking(True)
        self.setMouseTracking(True)

        self.fps_label = QLabel("FPS: - / -")
        self.fps_label.setFont(QFont("Arial", 12))
        self.fps_label.setStyleSheet("color: white; background-color: black; padding: 6px;")

        self.close_btn = QPushButton("Close")
        self.close_btn.clicked.connect(self.close)

        layout = QVBoxLayout()
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.fps_label)
        btn_layout.addStretch()
        btn_layout.addWidget(self.close_btn)

        layout.addWidget(self.image_label)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self.showFullScreen()

        self.lock = Lock()
        self.frame_queue = deque()
        self.max_queue_size = 5

        self.input_frame_count = 0
        self.output_frame_count = 0
        self.last_fps_update = time.time()

        self.process_timer = QTimer()
        self.process_timer.timeout.connect(self.process_frame_queue)
        self.process_timer.start(33)

        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)

        self.new_image.connect(self.update_image)

        self.dst_size = (640, 480)
        self.src_pts = np.float32([[73, 53], [581, 52], [639, 427], [15, 423]])
        self.dst_pts = np.float32([[0, 0], [self.dst_size[0], 0], [self.dst_size[0], self.dst_size[1]], [0, self.dst_size[1]]])
        self.perspective_matrix = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

        self.initial_position_px = (536, 301)
        self.scale_mm_per_pixel = 1.5625
        self.circle_radius_mm = 150.0
        self.cursor_radius_mm = 50.0 * 150. / 275.0
        self.adjusted_radius_mm = (self.circle_radius_mm * self.circle_radius_mm) / 275.0

        self.initial_xy_mm = None
        self.current_xy_mm = None
        self.pose_queue = deque()
        self.extra_delay_sec = 1.0

        self.ee_trail = deque(maxlen=50)
        self.cv_img = None
        self.cursor_pos = None

    def _draw_dashed_line(self, img, pt_start, pt_end, colour, thickness=1, dash_length=10):
        dist = np.linalg.norm(np.array(pt_end) - np.array(pt_start))
        dashes = int(dist // (dash_length * 2))
        for i in range(dashes + 1):
            start_frac = (2 * i) * dash_length / dist
            end_frac = min((2 * i + 1) * dash_length / dist, 1.0)
            start_pt = (
                int(pt_start[0] + (pt_end[0] - pt_start[0]) * start_frac),
                int(pt_start[1] + (pt_end[1] - pt_start[1]) * start_frac)
            )
            end_pt = (
                int(pt_start[0] + (pt_end[0] - pt_start[0]) * end_frac),
                int(pt_start[1] + (pt_end[1] - pt_start[1]) * end_frac)
            )
            cv2.line(img, start_pt, end_pt, colour, thickness)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.image_label.pixmap():
            label_pos = self.image_label.mapFrom(self, event.pos())
            self.cursor_pos = self.mapToImageCoordinates(label_pos)

    def mapToImageCoordinates(self, widget_pos):
        pixmap = self.image_label.pixmap()
        if not pixmap:
            return None
        label_size = self.image_label.size()
        pixmap_size = pixmap.size()
        scaled_pixmap_size = pixmap.scaled(label_size, Qt.KeepAspectRatio).size()
        x_offset = (label_size.width() - scaled_pixmap_size.width()) / 2
        y_offset = (label_size.height() - scaled_pixmap_size.height()) / 2
        x = (widget_pos.x() - x_offset) * self.dst_size[0] / scaled_pixmap_size.width()
        y = (widget_pos.y() - y_offset) * self.dst_size[1] / scaled_pixmap_size.height()
        return QPoint(int(x), int(y))

    def update_pose(self, msg):
        x_mm = msg.pose.position.y * 1000.0
        y_mm = msg.pose.position.x * 1000.0
        if self.initial_xy_mm is None:
            self.initial_xy_mm = (x_mm, y_mm)
        now = time.time()
        self.pose_queue.append((now, (x_mm, y_mm)))
        while self.pose_queue and self.pose_queue[0][0] < now - self.extra_delay_sec:
            _, xy_mm = self.pose_queue.popleft()
            self.current_xy_mm = xy_mm

    def add_image_to_queue(self, img_np):
        with self.lock:
            if len(self.frame_queue) < self.max_queue_size:
                self.frame_queue.append(img_np)
                self.input_frame_count += 1

    def process_frame_queue(self):
        with self.lock:
            if self.frame_queue:
                self.new_image.emit(self.frame_queue.popleft())

    def update_fps(self):
        now = time.time()
        elapsed = now - self.last_fps_update
        if elapsed > 0:
            self.fps_label.setText(f"Input FPS: {self.input_frame_count / elapsed:.1f} | Output FPS: {self.output_frame_count / elapsed:.1f}")
        self.input_frame_count = 0
        self.output_frame_count = 0
        self.last_fps_update = now

    def update_image(self, img_np):
        if img_np is None:
            return
        warped = cv2.warpPerspective(img_np, self.perspective_matrix, self.dst_size)

        if self.current_xy_mm and self.initial_xy_mm:
            dx = self.current_xy_mm[0] - self.initial_xy_mm[0]
            dy = self.current_xy_mm[1] - self.initial_xy_mm[1]
            dx_px = int(dx / self.scale_mm_per_pixel)
            dy_px = int(dy / self.scale_mm_per_pixel)
            x0, y0 = self.initial_position_px
            pt = np.array([[[x0 + dx_px, y0 + dy_px]]], dtype=np.float32)
            warped_pt = cv2.perspectiveTransform(pt, self.perspective_matrix)[0][0]

            ee_radius_px = int(self.adjusted_radius_mm / self.scale_mm_per_pixel)
            cursor_radius_px = int(self.cursor_radius_mm / self.scale_mm_per_pixel)
            self.ee_trail.append(tuple(map(int, warped_pt)))

            overlay = warped.copy()
            for pt in self.ee_trail:
                cv2.circle(overlay, pt, cursor_radius_px, (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.3, warped, 0.7, 0, warped)

            cv2.circle(warped, tuple(map(int, warped_pt)), ee_radius_px, (0, 0, 255), 2)

            if self.cursor_pos:
                cx, cy = self.cursor_pos.x(), self.cursor_pos.y()
                distance = np.linalg.norm(np.array([cx, cy]) - np.array(warped_pt))
                draw_pt = (cx, cy) if distance <= ee_radius_px else tuple(map(int, warped_pt))
                cv2.circle(warped, draw_pt, cursor_radius_px, (0, 0, 255), -1)

        # Draw shapes relative to fixed warped initial
        pt_src = np.array([[[self.initial_position_px[0], self.initial_position_px[1]]]], dtype=np.float32)
        warped_initial = cv2.perspectiveTransform(pt_src, self.perspective_matrix)[0][0]

        scale1 = self.scale_mm_per_pixel * 0.95
        width1 = int(720 / scale1)
        top1 = int(90 / scale1)
        bottom1 = int(190 / scale1)

        x_right1 = int(warped_initial[0])
        x_left1 = x_right1 - width1
        y_top1 = int(warped_initial[1]) - top1
        y_bottom1 = int(warped_initial[1]) + bottom1
        pts1 = [(x_left1, y_top1), (x_right1, y_top1), (x_right1, y_bottom1), (x_left1, y_bottom1)]

        if self.mode in ["default", "rect", "sin"]:
            for i in range(4):
                self._draw_dashed_line(warped, pts1[i], pts1[(i + 1) % 4], (255, 255, 0))

        if self.mode in ["rect", "sin"]:
            x_right2 = int(warped_initial[0] - 180 / scale1)
            x_left2 = int(warped_initial[0] - 580 / scale1)
            y_top2 = int(warped_initial[1] - 50 / scale1)
            y_bottom2 = int(warped_initial[1] + 150 / scale1)
            pts2 = [(x_left2, y_top2), (x_right2, y_top2), (x_right2, y_bottom2), (x_left2, y_bottom2)]

            for i in range(4):
                self._draw_dashed_line(warped, pts2[i], pts2[(i + 1) % 4], (255, 0, 0))

            if self.mode == "sin":
                rect_width = x_right2 - x_left2
                rect_height = y_bottom2 - y_top2
                center_y = (y_top2 + y_bottom2) // 2
                amplitude = rect_height // 2 - 10
                wave_points = [(x, int(center_y + amplitude * np.sin(2 * np.pi * 2 * (x - x_left2) / rect_width)))
                               for x in range(x_left2, x_right2)]
                for i in range(1, len(wave_points)):
                    cv2.line(warped, wave_points[i - 1], wave_points[i], (0, 255, 255), 1)

        h, w = warped.shape[:2]
        q_img = QImage(warped.data, w, h, 3 * w, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img)
        scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio)
        self.image_label.setPixmap(scaled_pixmap)
        self.output_frame_count += 1


def image_callback(msg, viewer):
    try:
        img_np = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img_np is not None:
            viewer.add_image_to_queue(img_np)
    except Exception as e:
        rospy.logwarn(f"Image callback error: {e}")


def pose_callback(msg, viewer):
    viewer.update_pose(msg)


def main():
    rospy.init_node("depth_image_gui_viewer", anonymous=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else "default"
    app = QApplication(sys.argv)
    viewer = ImageViewer(mode)
    rospy.Subscriber("/depth_img/compressed", CompressedImage, image_callback, callback_args=viewer, queue_size=5, buff_size=2**22)
    rospy.Subscriber("/ee_for_paris", PoseStamped, pose_callback, callback_args=viewer, queue_size=10)
    Thread(target=rospy.spin, daemon=True).start()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
