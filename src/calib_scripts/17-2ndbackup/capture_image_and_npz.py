#!/usr/bin/env python3
import rospy
import tf2_ros
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import numpy as np
import json

class ManualCaptureSeparateVisual:
    def __init__(self):
        rospy.init_node("manual_image_pose_capture_visual")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.ee_frame = rospy.get_param("~ee_frame", "panda_hand")
        self.save_root = rospy.get_param("~save_dir", "/home/roslab/catkin_ws/src/calib_scripts/samples")

        self.checkerboard_size = (6, 8)  # Inner corners (rows, cols)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.latest_image = None
        self.detected_corners = False

        self.image_dir = os.path.join(self.save_root, "images")
        self.pose_dir = os.path.join(self.save_root, "poses")
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        self.counter = 0
        rospy.loginfo("[ManualCaptureSeparateVisual] Ready. Waiting for image and TF...")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
            self.detected_corners = ret

            display = self.latest_image.copy()
            if ret:
                cv2.drawChessboardCorners(display, self.checkerboard_size, corners, ret)

            cv2.putText(display, f"Images Captured: {self.counter}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if ret else (0, 0, 255), 2)

            cv2.imshow('Live Camera Feed - Checkerboard Detection', display)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"[ManualCaptureSeparateVisual] Image conversion error: {e}")

    def get_transform_matrix(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rospy.Time(0), rospy.Duration(1.0))
            trans = tf.transform.translation
            rot = tf.transform.rotation
            t = np.array([trans.x, trans.y, trans.z])
            q = np.array([rot.x, rot.y, rot.z, rot.w])
            R = self.quaternion_to_matrix(q)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            return T
        except Exception as e:
            rospy.logwarn(f"[ManualCaptureSeparateVisual] TF error: {e}")
            return None

    def quaternion_to_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

    def save(self):
        if self.latest_image is None:
            rospy.logwarn("[ManualCaptureSeparateVisual] No image received yet.")
            return

        if not self.detected_corners:
            rospy.logwarn("[ManualCaptureSeparateVisual] Checkerboard NOT detected. Move board and try again.")
            return

        T_robot = self.get_transform_matrix()
        if T_robot is None:
            rospy.logwarn("[ManualCaptureSeparateVisual] Skipping save due to missing transform.")
            return

        img_path = os.path.join(self.image_dir, f"image_{self.counter:02d}.png")
        pose_path = os.path.join(self.pose_dir, f"pose_{self.counter:02d}.json")

        cv2.imwrite(img_path, self.latest_image)

        data = {
            "robot_pose": T_robot.tolist(),
            "camera_frame": "camera_color_optical_frame"
        }

        with open(pose_path, "w") as f:
            json.dump(data, f, indent=2)

        rospy.loginfo(f"✅ Saved image + pose #{self.counter:02d}")
        self.counter += 1

    def run(self):
        while not rospy.is_shutdown():
            print("📸 Press ENTER to capture (Only if green corners visible)...")
            input()
            self.save()


if __name__ == "__main__":
    try:
        node = ManualCaptureSeparateVisual()
        node.run()
    except rospy.ROSInterruptException:
        pass
