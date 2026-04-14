#!/usr/bin/env python3
import rospy
import json
import numpy as np
import os
from geometry_msgs.msg import PoseStamped

class PixelTo3DPose:
    def __init__(self):
        rospy.init_node("pixel_to_3dpose_node")
        pub = rospy.Publisher("/red_object_pose", PoseStamped, queue_size=10)

        # === Load latest JSON file from detected_objects ===
        json_files = sorted([f for f in os.listdir("detected_objects") if f.endswith(".json")])
        latest_json = json_files[-1]
        with open(os.path.join("detected_objects", latest_json), "r") as f:
            data = json.load(f)
        u, v = data["pixel"]
        Z = data["depth"]  # in meters
        rospy.loginfo(f"[INFO] Using pixel ({u}, {v}) and depth {Z} from {latest_json}")

        # === Load camera intrinsics ===
        intrinsics = np.load("IntrinsicMatrix.npz")
        K = intrinsics["arr_0"]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # === Step 1: Back-project to camera frame ===
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        camera_point = np.array([X, Y, Z, 1.0])  # homogeneous
        rospy.loginfo(f"[INFO] 3D in camera frame: {camera_point[:3]}")

        # === Step 2: Transform to gripper frame ===
        T_gripper2cam = np.load("FinalTransforms/T_gripper2cam_Method_0.npz")["arr_0"]
        point_in_gripper = T_gripper2cam @ camera_point
        rospy.loginfo(f"[INFO] 3D in gripper frame: {point_in_gripper[:3]}")

        # === Step 3: Transform to base frame ===
        pose_npz_files = sorted([f for f in os.listdir("poses") if f.endswith(".npz")])
        latest_pose = pose_npz_files[-1]
        T_gripper2base = np.load(os.path.join("poses", latest_pose))["arr_0"]
        point_in_base = T_gripper2base @ point_in_gripper
        rospy.loginfo(f"[INFO] 3D in BASE frame: {point_in_base[:3]}")

        # === Step 4: Publish PoseStamped ===
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = point_in_base[0]
        pose_msg.pose.position.y = point_in_base[1]
        pose_msg.pose.position.z = point_in_base[2]

        # Fixed orientation (gripper pointing down)
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 1
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 0

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(pose_msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        PixelTo3DPose()
    except rospy.ROSInterruptException:
        pass

