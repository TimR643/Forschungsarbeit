#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf
from datetime import datetime
from visualization_msgs.msg import Marker

bridge = CvBridge()
listener = None
marker_pub = None
red_pose_pub = None
latest_depth = None

# Load hand-eye calibration matrix (camera to end-effector)
T_cam_to_ee = np.load("T_camera_to_ee.npy")

# ✅ Real camera intrinsics from /camera/color/camera_info
K = np.array([[229.32417, 0.0, 158.28212],
              [0.0, 229.09262, 89.11006],
              [0.0, 0.0, 1.0]])

def get_robot_pose():
    try:
        now = rospy.Time(0)
        listener.waitForTransform("panda_link0", "panda_link8", now, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_link8", now)
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        return T
    except:
        return None

def publish_marker(position, label):
    marker = Marker()
    marker.header.frame_id = "panda_link0"
    marker.header.stamp = rospy.Time.now()
    marker.ns = label
    marker.id = int(datetime.now().strftime("%f")) % 1000000
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.scale.x = marker.scale.y = marker.scale.z = 0.03

    if label == "RED":
        marker.color.r = 1.0
    elif label == "GREEN":
        marker.color.g = 1.0
    elif label == "BLUE":
        marker.color.b = 1.0
    elif label == "YELLOW":
        marker.color.r = marker.color.g = 1.0

    marker.color.a = 1.0
    marker_pub.publish(marker)

def detect_and_publish(frame, mask, label):
    global latest_depth, red_pose_pub

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) < 300:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        if latest_depth is None or cy >= latest_depth.shape[0] or cx >= latest_depth.shape[1]:
            continue
        depth_val = latest_depth[cy, cx] * 0.001  # mm to meters

        if depth_val == 0 or np.isnan(depth_val) or depth_val > 1.5:
            rospy.logwarn(f"[{label}] Invalid depth at ({cx}, {cy}): {depth_val:.3f}")
            continue

        # Backproject to camera frame
        uv = np.array([[cx], [cy], [1]])
        xyz_cam = np.linalg.inv(K) @ uv * depth_val

        # Transform to base frame
        T_robot = get_robot_pose()
        if T_robot is None:
            continue
        T_base_to_camera = T_robot @ np.linalg.inv(T_cam_to_ee)

        p_cam = np.ones(4)
        p_cam[:3] = xyz_cam.flatten()
        p_base = T_base_to_camera @ p_cam
        pos = np.round(p_base[:3], 3)

        # Optional Z-lift adjustment
        pos[2] -= 0.30  # Try 0.10–0.15 until marker hits object top

        rospy.loginfo(f"[{label}] Pixel=({cx},{cy}), depth={depth_val:.3f}m → base pos={pos}")
        publish_marker(pos, label)

        # Publish pose for RED only
        if label == "RED":
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "panda_link0"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            pose_msg.pose.orientation.w = 1.0  # Identity quaternion
            red_pose_pub.publish(pose_msg)
            rospy.loginfo("Published RED object pose to /red_object_pose")

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
    detect_and_publish(frame, red_mask, "RED")
    detect_and_publish(frame, cv2.inRange(hsv, lower_green, upper_green), "GREEN")
    detect_and_publish(frame, cv2.inRange(hsv, lower_blue, upper_blue), "BLUE")
    detect_and_publish(frame, cv2.inRange(hsv, lower_yellow, upper_yellow), "YELLOW")

def depth_callback(msg):
    global latest_depth
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth_image.dtype != np.uint16:
            rospy.logwarn("Depth image is not uint16")
            return
        latest_depth = depth_image
    except Exception as e:
        rospy.logerr(f"[Depth Error] {e}")

def main():
    global listener, marker_pub, red_pose_pub
    rospy.init_node("color_object_localizer_depth")
    listener = tf.TransformListener()
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    red_pose_pub = rospy.Publisher("/red_object_pose", PoseStamped, queue_size=10)

    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)

    rospy.loginfo("Started RED object detection with TF and camera intrinsics.")
    rospy.spin()

if __name__ == "__main__":
    main()
