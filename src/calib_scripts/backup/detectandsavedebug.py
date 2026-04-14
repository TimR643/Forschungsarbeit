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

K = np.array([[615.0, 0, 320],
              [0, 615.0, 240],
              [0, 0, 1]])

os.makedirs("detected_objects_debug", exist_ok=True)

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

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Target color range — RED
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if cv2.contourArea(c) > 300:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            print(f"🟡 [RED] pixel (cx,cy): ({cx}, {cy})")

            # Get depth image
            depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image)
            depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_val = depth[cy, cx] / 1000.0  # Convert mm to meters

            if depth_val == 0 or np.isnan(depth_val):
                print("⚠️ Depth invalid at pixel")
                continue

            print(f"📏 Depth at pixel: {depth_val:.3f} m")

            uv_point = np.array([[cx], [cy], [1]])
            xyz_camera = np.linalg.inv(K) @ uv_point * depth_val
            print(f"📐 XYZ in camera frame: {np.round(xyz_camera.flatten(), 3)}")

            T_robot = get_robot_pose()
            if T_robot is None:
                continue

            T_cam_to_ee = np.load("T_camera_to_ee.npy")
            T_base_to_camera = T_robot @ np.linalg.inv(T_cam_to_ee)

            p_cam = np.ones(4)
            p_cam[:3] = xyz_camera.flatten()
            p_base = T_base_to_camera @ p_cam
            pos_base = np.round(p_base[:3], 3)

            print(f"📍 Transformed to base frame: {pos_base}")

            # Optional height adjustment
            pos_base[2] -= 0.07  # assuming object is 8cm tall

            output = {
                "timestamp": datetime.now().isoformat(),
                "color": "RED",
                "position": pos_base.tolist()
            }

            filename = f"detected_objects_debug/red_object_{datetime.now().strftime('%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(output, f, indent=4)

            print(f"💾 Saved RED object to {filename}")
            rospy.signal_shutdown("Detection complete")

def main():
    global listener
    rospy.init_node("debug_red_detector")
    listener = tf.TransformListener()
    rospy.Subscriber("/camera/color/image_rect_raw", Image, image_callback)
    print("🔍 Looking for red object. Move it into view. Waiting...")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
