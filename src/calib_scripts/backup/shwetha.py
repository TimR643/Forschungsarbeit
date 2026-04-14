#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import json
import os
from datetime import datetime

bridge = CvBridge()
listener = None
depth_image = None

# Intrinsics (set yours if calibrated)
K = np.array([[615.0, 0, 320],
              [0, 615.0, 240],
              [0, 0, 1]])
D = np.zeros(5)

os.makedirs("detected_objects", exist_ok=True)

def get_robot_pose():
    try:
        now = rospy.Time(0)
        listener.waitForTransform("panda_link0", "panda_link8", now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_link8", now)
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        return T
    except Exception as e:
        print("❌ TF lookup failed:", e)
        return None

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except Exception as e:
        print("❌ Depth conversion error:", e)

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV Ranges
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    detect_and_save(frame, red_mask, (0, 0, 255), "RED")
    detect_and_save(frame, green_mask, (0, 255, 0), "GREEN")
    detect_and_save(frame, blue_mask, (255, 0, 0), "BLUE")
    detect_and_save(frame, yellow_mask, (0, 255, 255), "YELLOW")

    cv2.imshow("Multi-Color Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        rospy.signal_shutdown("User exit")

def detect_and_save(frame, mask, color_bgr, label):
    global depth_image
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > 300:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(frame, (cx, cy), 8, color_bgr, 2)
            cv2.putText(frame, label, (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

            if depth_image is None:
                print("⚠️ No depth image received yet.")
                continue

            depth_val = depth_image[cy, cx].astype(np.float32) / 1000.0  # mm → meters
            if depth_val == 0 or np.isnan(depth_val):
                print(f"⚠️ Invalid depth at ({cx},{cy})")
                continue

            uv = np.array([[cx], [cy], [1]])
            xyz_camera = np.linalg.inv(K) @ uv * depth_val

            try:
                T_cam_to_ee = np.load("T_camera_to_ee.npy")
            except:
                print("❌ Could not load calibration matrix")
                return

            T_ee_to_base = get_robot_pose()
            if T_ee_to_base is None:
                return

            T_cam_to_base = T_ee_to_base @ T_cam_to_ee
            p_cam = np.ones(4)
            p_cam[:3] = xyz_camera.flatten()
            p_base = T_cam_to_base @ p_cam
            pos = np.round(p_base[:3], 3)

            # 🔍 Debug info
            print(f"🟡 [{label}] pixel (cx,cy): ({cx}, {cy})")
            print(f"📏 Depth at pixel: {depth_val:.3f} m")
            print(f"📐 XYZ in camera frame: {xyz_camera.flatten()}")
            print(f"📍 Transformed to base frame: {pos}")

            output = {
                "timestamp": datetime.now().isoformat(),
                "color": label,
                "position": pos.tolist()
            }

            filename = f"detected_objects/{label.lower()}_object_{datetime.now().strftime('%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(output, f, indent=4)

            print(f"💾 Saved {label} object to {filename}")

def main():
    global listener
    rospy.init_node("multi_color_detection_saver")
    listener = tf.TransformListener()
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    print("🔍 Detecting RED, GREEN, BLUE, YELLOW... Press ESC to quit.")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
