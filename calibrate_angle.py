#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

clicked_points = []
frame_ready = False
frame_image = None

point_labels = [
    "Click corner 1",
    "Click corner 2",
    "Click corner 3",
    "Click corner 4",
    "Click initial position",
    "Click point 1 (known dist)",
    "Click point 2 (known dist)"
]

def draw_text(img, text):
    overlay = img.copy()
    cv2.rectangle(overlay, (10, 10), (700, 50), (0, 0, 0), -1)
    alpha = 0.6
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    cv2.putText(img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

def click_event(event, x, y, flags, param):
    global clicked_points
    img = param['img']

    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < len(point_labels):
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: ({x}, {y})")

        # Draw circle & label
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(img, str(len(clicked_points)), (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Update instructions
        if len(clicked_points) < len(point_labels):
            draw_text(img, point_labels[len(clicked_points)])
        else:
            draw_text(img, "All points collected! Press any key.")
        cv2.imshow("Calibration", img)

def image_callback(msg):
    global frame_ready, frame_image

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img_np is not None:
        frame_image = img_np
        frame_ready = True

def main():
    global frame_ready, frame_image
    rospy.init_node('calibrate_clicks', anonymous=True)
    rospy.Subscriber('/depth_img/compressed', CompressedImage, image_callback)

    print("📡 Waiting for a frame from /depth_img/compressed...")
    while not frame_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if frame_image is None:
        print("❌ Failed to receive image.")
        return

    img = frame_image.copy()
    draw_text(img, point_labels[0])
    cv2.imshow("Calibration", img)
    cv2.setMouseCallback("Calibration", click_event, {'img': img})

    while not rospy.is_shutdown():
        key = cv2.waitKey(20) & 0xFF
        if len(clicked_points) == len(point_labels):
            break
        if key == 27:  # ESC to cancel
            print("Calibration cancelled.")
            cv2.destroyAllWindows()
            return

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("\n✅ Calibration complete:\n")
    print("src_pts = np.float32([")
    for pt in clicked_points[:4]:
        print(f"    [{pt[0]}, {pt[1]}],")
    print("])\n")
    print(f"initial_position = {clicked_points[4]}\n")

    pt1 = np.array(clicked_points[5])
    pt2 = np.array(clicked_points[6])
    pixel_dist = np.linalg.norm(pt1 - pt2)
    real_dist_mm = 25.0
    scale = real_dist_mm / pixel_dist

    print(f"# Known distance = {real_dist_mm} mm")
    print(f"# Pixel distance = {pixel_dist:.2f}")
    print(f"scale = {scale:.6f} mm/pixel")

if __name__ == "__main__":
    main()
