#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import json
from geometry_msgs.msg import TransformStamped
import os

def load_intrinsics(path):
    data = np.load(path)
    K = data['arr_0']  # Camera intrinsic matrix
    return K

def load_object(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    u, v = data['pixel']
    depth = data['depth']
    return u, v, depth, data

def pixel_to_camera_coords(u, v, depth, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

    return np.array([X, Y, Z, 1.0])  # Homogeneous coordinates

def load_gripper2cam(path):
    data = np.load(path)
    return data['arr_0']  # Hand-eye 4x4 matrix

def lookup_base2gripper(tf_buffer, base_frame="panda_link0", gripper_frame="panda_hand"):
    try:
        tf = tf_buffer.lookup_transform(base_frame, gripper_frame, rospy.Time(0), rospy.Duration(2.0))
        t = tf.transform.translation
        r = tf.transform.rotation
        T = transform_to_matrix(t, r)
        return T
    except Exception as e:
        rospy.logerr(f"[TF ERROR] {e}")
        return None

def transform_to_matrix(t, r):
    tx, ty, tz = t.x, t.y, t.z
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    R = quaternion_to_matrix(qx, qy, qz, qw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T

def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

def process_color_object(color, json_path, K, T_base2gripper, T_gripper2cam):
    if not os.path.exists(json_path):
        rospy.logwarn(f"⚠️ {color.capitalize()} object file not found: {json_path}")
        return

    u, v, depth, data = load_object(json_path)
    point_cam = pixel_to_camera_coords(u, v, depth, K)
    point_base = T_base2gripper @ T_gripper2cam @ point_cam
    x, y, z = point_base[:3]

    data['position'] = [round(float(x), 4), round(float(y), 4), round(float(z), 4)]

    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)

    rospy.loginfo(f"✅ {color.capitalize()} object updated with 3D position: {json_path}")

def main():
    rospy.init_node("pixel_to_3d_pose_single_object")

    # === CHANGE THESE PATHS TO MATCH YOUR SETUP ===
    base_path = "/home/roslab/catkin_ws/src/calib_scripts/detected_objects"
    intrinsics_path = "/home/roslab/catkin_ws/src/calib_scripts/IntrinsicMatrix.npz"
    gripper2cam_path = "/home/roslab/catkin_ws/src/calib_scripts/FinalTransforms/T_gripper2cam_Method_3.npz"

    color_files = {
        "red": os.path.join(base_path, "red_object.json"),
        "green": os.path.join(base_path, "green_object.json"),
        "blue": os.path.join(base_path, "blue_object.json")
    }

    K = load_intrinsics(intrinsics_path)
    T_gripper2cam = load_gripper2cam(gripper2cam_path)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    T_base2gripper = lookup_base2gripper(tf_buffer)
    if T_base2gripper is None:
        rospy.logerr("❌ Could not get base→gripper transform. Aborting.")
        return

    for color, json_path in color_files.items():
        process_color_object(color, json_path, K, T_base2gripper, T_gripper2cam)

if __name__ == "__main__":
    main()
