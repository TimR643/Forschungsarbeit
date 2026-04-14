#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import json
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ==== SETTINGS ====
CHECKERBOARD = (6, 8)   # (rows, columns) of inner corners
SQUARE_SIZE = 0.025     # 2.5 cm squares (in meters)
NUM_IMAGES = 30         # Collect 30 images

# ==== STORAGE ====
objpoints = []
imgpoints = []
bridge = CvBridge()

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

os.makedirs('calibration_data', exist_ok=True)

collected_images = 0
latest_gray = None

def image_callback(msg):
    global collected_images, latest_gray

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    latest_gray = gray.copy()

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    display = frame.copy()

    if ret:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret)

    cv2.putText(display, f"Images: {collected_images}/{NUM_IMAGES}", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Camera Calibration - Press s to Save Image', display)
    key = cv2.waitKey(1)

    if key == ord('s') and ret:
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        collected_images += 1
        print(f"[INFO] Saved image {collected_images}/{NUM_IMAGES}")

        if collected_images >= NUM_IMAGES:
            print("[INFO] Enough images collected. Calibrating camera...")
            calibrate_camera()

    if key == ord('q'):
        rospy.signal_shutdown("User Quit")
        cv2.destroyAllWindows()

def calibrate_camera():
    global latest_gray

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, latest_gray.shape[::-1], None, None
    )

    calibration = {
        'camera_matrix': mtx.tolist(),
        'dist_coeff': dist.tolist()
    }

    with open('calibration_data/camera_intrinsics.json', 'w') as f:
        json.dump(calibration, f, indent=4)

    print("[SUCCESS] Camera calibration completed.")
    print("[INFO] Saved to calibration_data/camera_intrinsics.json")

    rospy.signal_shutdown("Calibration Completed")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_intrinsic_calibration_node')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    print("[INFO] Node Started.")
    print("[INFO] Move checkerboard, press 's' to save image.")
    print("[INFO] Press 'q' to quit anytime.")

    rospy.spin()
