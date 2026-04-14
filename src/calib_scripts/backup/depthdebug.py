#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import tf
import json
from datetime import datetime

bridge = CvBridge()

# Load hand-eye calibration matrix
T_cam_to_ee = np.load("T_camera_to_ee.npy")

# Define camera intrinsics (update with your calibration if known)
K = np.array([[615.0, 0.0, 320.0],
              [0.0, 615.0, 240.0],
              [0.0,   0.0,   1.0]])

listener = None


def get_ee_to_base():
    try:
        now = rospy.Time(0)
        listener.waitForTransform("panda_link0", "panda_link8", now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_link8", now)
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        return T
    except Exception as e:
        rospy.logwarn("TF lookup failed: %s", str(e))
        return None


def callback(color_msg, depth_msg):
    color_img = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("🔍 No red object found.")
        return

    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    depth_raw = depth_img[cy, cx]
    if depth_img.dtype == np.uint16:
        depth_m = float(depth_raw) / 1000.0
    else:
        depth_m = float(depth_raw)

    print(f"[RED] pixel (cx,cy): ({cx}, {cy})")
    print(f"📏 Depth at pixel: {depth_m:.3f} m")

    # Backproject to camera frame
    uv = np.array([[cx], [cy], [1]])
    xyz_cam = np.linalg.inv(K) @ uv * depth_m
    print(f"📐 XYZ in camera frame: {xyz_cam.flatten()}")

    # Transform to robot base frame
    T_ee_to_base = get_ee_to_base()
    if T_ee_to_base is None:
        print("⚠️ Skipping frame due to TF error.")
        return

    T_cam_to_base = T_ee_to_base @ np.linalg.inv(T_cam_to_ee)
    point_cam = np.ones((4, 1))
    point_cam[0:3, 0] = xyz_cam.flatten()
    point_base = T_cam_to_base @ point_cam

    xyz_base = np.round(point_base[:3].flatten(), 3)
    print(f"📍 Transformed to base frame: {xyz_base}")

    # Save to file
    result = {
        "timestamp": datetime.now().isoformat(),
        "color": "RED",
        "position": xyz_base.tolist()
    }
    fname = f"detected_objects_debug/red_object_{datetime.now().strftime('%H%M%S')}.json"
    with open(fname, 'w') as f:
        json.dump(result, f, indent=4)

    print(f"💾 Saved RED object to {fname}")


def main():
    global listener
    rospy.init_node("red_object_localizer")
    listener = tf.TransformListener()

    color_sub = Subscriber("/camera/color/image_raw", Image)
    depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)

    ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    print("🔴 Localizing RED object — move into view!")
    rospy.spin()

if __name__ == '__main__':
    main()
