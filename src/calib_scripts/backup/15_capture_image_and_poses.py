#!/usr/bin/env python3
import rospy
import tf2_ros
import tf.transformations as tf_trans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

class ManualImagePoseCapture:
    def __init__(self):
        # --- ROS PARAMETERS ---
        self.save_dir  = rospy.get_param("~save_dir",  os.path.expanduser("~/calib_data"))
        self.camera_ns = rospy.get_param("~camera_ns", "/camera/color")
        self.base_link = rospy.get_param("~base_link", "panda_link0")
        self.ee_link   = rospy.get_param("~ee_link",   "panda_hand")

        # Create folders
        self.img_dir  = os.path.join(self.save_dir, "images")
        self.pose_dir = os.path.join(self.save_dir, "poses")
        os.makedirs(self.img_dir,  exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        # TF & image bridge
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge      = CvBridge()
        self.latest_image = None

        # Image topic
        self.image_topic = f"{self.camera_ns}/image_raw"
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        # Counter
        self.count = 0

        rospy.loginfo(f"[ManualCapture] Initialized.")
        rospy.loginfo(f"[ManualCapture] Image topic: {self.image_topic}")
        rospy.loginfo(f"[ManualCapture] TF: {self.base_link} → {self.ee_link}")
        rospy.loginfo(f"[ManualCapture] Save directory: {self.save_dir}")

        self.main_loop()

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = img
        except Exception as e:
            rospy.logwarn(f"CV Bridge error: {e}")

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_image is None:
                rospy.loginfo_throttle(2.0, "Waiting for image...")
                rate.sleep()
                continue

            # Wait for transform
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.base_link, self.ee_link, rospy.Time(0), rospy.Duration(0.5))
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"Waiting for TF {self.base_link} → {self.ee_link}: {e}")
                rate.sleep()
                continue

            input(f"\n[Ready] Press Enter to capture image+pose #{self.count:02d}...")

            # Convert TF to matrix
            T = np.eye(4)
            t = trans.transform.translation
            q = trans.transform.rotation
            T[:3, 3] = [t.x, t.y, t.z]
            T[:3, :3] = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

            # Save image
            img_path = os.path.join(self.img_dir, f"image_{self.count:02d}.png")
            cv2.imwrite(img_path, self.latest_image)

            # Save pose
            pose_path = os.path.join(self.pose_dir, f"pose_{self.count:02d}.json")
            with open(pose_path, "w") as f:
                json.dump({"T": T.flatten().tolist()}, f, indent=2)

            rospy.loginfo(f"[Saved] image_{self.count:02d}.png + pose_{self.count:02d}.json")
            self.count += 1

            rospy.loginfo("Now move the robot to the next pose in RViz or manually.")

if __name__ == "__main__":
    rospy.init_node("manual_capture_image_pose")
    try:
        ManualImagePoseCapture()
    except rospy.ROSInterruptException:
        pass
