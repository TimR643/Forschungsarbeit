#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import csv
import os
import time

# === Load camera-to-gripper transform ===
calib = np.load('/home/roslab/catkin_ws/src/calib_scripts/camera_to_gripper_final.npz')
T_cam2gripper = calib['T']

# === Camera intrinsics (from camera_info) ===
fx = 229.32417297
fy = 229.09262085
cx = 158.28211975
cy = 89.11006165

bridge = CvBridge()

# HSV color ranges for detection (red with two ranges)
COLOR_RANGES = {
    "red": [
        ([0, 50, 50], [10, 255, 255]),
        ([160, 50, 50], [180, 255, 255])
    ],
    "green": [([35, 40, 40], [90, 255, 255])],
    "blue": [([100, 100, 50], [130, 255, 255])]
}

csv_file = os.path.expanduser('~/catkin_ws/src/calib_scripts/detected_objects.csv')
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as f:
        csv.writer(f).writerow(['timestamp', 'x', 'y', 'z', 'color'])

point_pub = None

def publish_point(x, y, z):
    global point_pub
    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'panda_link0'
    msg.point.x = x
    msg.point.y = y
    msg.point.z = z
    point_pub.publish(msg)

def detect_objects(rgb_img, depth_img):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
    results = {}

    for color, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            current_mask = cv2.inRange(hsv, lower_np, upper_np)
            mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)

        # Show mask for debugging
        cv2.imshow(f'{color.capitalize()} Mask', mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"[DEBUG] {color}: Found {len(contours)} contours")

        for cnt in contours:
            if cv2.contourArea(cnt) < 100:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])

            z = depth_img[v, u] / 1000.0
            if z <= 0.01 or z > 2.0:
                rospy.logwarn(f"[WARN] Skipping invalid Z={z:.3f} at ({u},{v}) for {color}")
                continue

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            point_cam = np.array([x, y, z, 1.0])
            point_gripper = T_cam2gripper @ point_cam
            xg, yg, zg = point_gripper[:3]

            if color not in results:
                results[color] = []
            results[color].append((xg, yg, zg))

    cv2.waitKey(1)
    return results

def callback(color_msg, depth_msg):
    try:
        rgb_img = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except Exception as e:
        rospy.logerr(f"CvBridge error: {e}")
        return

    detections = detect_objects(rgb_img, depth_img)
    total = sum(len(lst) for lst in detections.values())
    rospy.loginfo(f"[DETECTED {total} object(s)]")
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        for color, points in detections.items():
            for (xg, yg, zg) in points:
                rospy.loginfo(f"  → {color.upper()} at [x={xg:.3f}, y={yg:.3f}, z={zg:.3f}]")
                writer.writerow([time.time(), xg, yg, zg, color])
                publish_point(xg, yg, zg)

def main():
    global point_pub
    rospy.init_node("color_detector_debug", anonymous=True)

    point_pub = rospy.Publisher('detected_object_point', PointStamped, queue_size=10)

    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

    sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
    sync.registerCallback(callback)

    rospy.loginfo("[INFO] Color detection with debug masks started...")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
