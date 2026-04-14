#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import json
import numpy as np
from geometry_msgs.msg import TransformStamped

class ManualCapture:
    def __init__(self):
        rospy.init_node("manual_image_pose_capture")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.ee_frame = rospy.get_param("~ee_frame", "panda_hand")
        self.save_dir = rospy.get_param("~save_dir", "/home/roslab/catkin_ws/src/calib_scripts")

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.latest_image = None

        self.counter = 0
        rospy.loginfo("[ManualCapture] Initialized. Waiting for image and tf...")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"[ManualCapture] Error converting image: {e}")

    def get_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rospy.Time(0), rospy.Duration(1.0))
            T = self.transform_to_matrix(tf.transform)
            return T
        except Exception as e:
            rospy.logwarn(f"[ManualCapture] Could not get transform: {e}")
            return None

    def transform_to_matrix(self, transform):
        trans = transform.translation
        rot = transform.rotation
        t = np.array([trans.x, trans.y, trans.z])
        q = np.array([rot.x, rot.y, rot.z, rot.w])
        R = self.quaternion_to_matrix(q)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T.tolist()

    def quaternion_to_matrix(self, q):
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])
        return R

    def save(self):
        if self.latest_image is None:
            rospy.logwarn("[ManualCapture] No image received yet.")
            return

        transform = self.get_transform()
        if transform is None:
            rospy.logwarn("[ManualCapture] Skipping save due to missing transform.")
            return

        image_path = os.path.join(self.save_dir, "images")
        pose_path = os.path.join(self.save_dir, "poses")
        os.makedirs(image_path, exist_ok=True)
        os.makedirs(pose_path, exist_ok=True)

        img_filename = os.path.join(image_path, f"image_{self.counter:02d}.png")
        pose_filename = os.path.join(pose_path, f"pose_{self.counter:02d}.json")

        cv2.imwrite(img_filename, self.latest_image)
        with open(pose_filename, "w") as f:
            json.dump(transform, f, indent=2)

        rospy.loginfo(f"✅ Saved image and pose {self.counter}")
        self.counter += 1

    def run(self):
        while not rospy.is_shutdown():
            input("Press ENTER to capture image and pose...")
            self.save()


if __name__ == "__main__":
    try:
        node = ManualCapture()
        node.run()
    except rospy.ROSInterruptException:
        pass

