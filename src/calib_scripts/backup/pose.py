import rospy
import tf2_ros
import numpy as np
import os
import tf.transformations as tf_trans

# Initialize ROS node
rospy.init_node("save_tf_node")

# TF buffer and listener
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# Desired save directory (this must match what CameraCalibration expects)
save_dir = os.path.expanduser("/home/roslab/catkin_ws/src/calib_scripts/pose")
os.makedirs(save_dir, exist_ok=True)

rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    try:
        # Get transform from base to end-effector
        trans = tf_buffer.lookup_transform("panda_link0", "panda_hand", rospy.Time(0), rospy.Duration(1.0))
        translation = trans.transform.translation
        rotation = trans.transform.rotation

        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, 3] = [translation.x, translation.y, translation.z]
        R = tf_trans.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        T[:3, :3] = R[:3, :3]

        # Save transform with increasing index
        existing_files = sorted(f for f in os.listdir(save_dir) if f.endswith('.npz'))
        index = len(existing_files)
        file_path = os.path.join(save_dir, f"transform_{index}.npz")
        np.savez(file_path, T)

        print(f"[✓] Saved transform to: {file_path}")
        break

    except Exception as e:
        print("[!] Transform not available:", e)

    rate.sleep()
