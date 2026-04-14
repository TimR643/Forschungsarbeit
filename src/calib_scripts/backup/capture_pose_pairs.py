#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import os
import json

# Checkerboard parameters (for 9x7 squares → 8x6 inner corners)
CHECKERBOARD = (8, 6)
SQUARE_SIZE = 0.025  # in meters (25mm per square)

# Camera intrinsics (replace with your calibrated intrinsics if available)
K = np.array([[615.0, 0, 320],
              [0, 615.0, 240],
              [0, 0, 1]])
D = np.zeros(5)

# Generate object points in checkerboard frame
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

def get_next_filename():
    os.makedirs("calib_data", exist_ok=True)
    idx = 0
    while os.path.exists(f"calib_data/sample_{idx:02d}.json"):
        idx += 1
    return f"calib_data/sample_{idx:02d}.json"

def get_robot_pose(listener):
    try:
        now = rospy.Time(0)
        listener.waitForTransform("panda_link0", "panda_link8", now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_link8", now)
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        return T
    except Exception as e:
        print("❌ Could not get robot pose:", str(e))
        return None

def main():
    rospy.init_node("capture_one_sample", anonymous=True)
    listener = tf.TransformListener()
    bridge = CvBridge()

    print("📷 Waiting for one image from camera...")
    try:
        msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
    except:
        print("❌ No image received from /camera/color/image_raw.")
        return

    # Convert image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect checkerboard
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if not found:
        print("❌ Checkerboard not found.")
        return

    # Refine corner points
    corners2 = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    # Solve PnP (pose estimation of checkerboard in camera frame)
    ret, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
    if not ret:
        print("❌ solvePnP failed.")
        return

    # Get robot EE pose
    robot_T = get_robot_pose(listener)
    if robot_T is None:
        print("❌ Robot pose not available.")
        return

    # Save result to file
    data = {
        "robot_pose": robot_T.tolist(),
        "rvec": rvec.tolist(),
        "tvec": tvec.tolist()
    }

    filename = get_next_filename()
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)

    print(f"✅ Sample saved to: {filename}")

if __name__ == '__main__':
    main()
