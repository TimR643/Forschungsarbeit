#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import os
from tf.transformations import quaternion_matrix

def save_transform_as_npz(save_path, tf_msg):
    # Convert geometry_msgs/TransformStamped to 4x4 matrix
    trans = tf_msg.transform.translation
    rot = tf_msg.transform.rotation
    T = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    T[0, 3] = trans.x
    T[1, 3] = trans.y
    T[2, 3] = trans.z
    np.savez(save_path, T)
    print(f"✅ Saved transform to {save_path}")

if __name__ == '__main__':
    rospy.init_node('get_base_to_ee_transform')
    
    base_link = rospy.get_param("~base_link", "panda_link0")
    ee_link   = rospy.get_param("~ee_link", "panda_hand")
    save_dir  = rospy.get_param("~save_dir", "/home/catkin_ws/src/calib_scripts/poses")
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)  # Allow buffer to fill
    rate = rospy.Rate(1)

    count = 0
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(base_link, ee_link, rospy.Time(0), rospy.Duration(1.0))
            filename = os.path.join(save_dir, f"pose_{count:02d}.npz")
            save_transform_as_npz(filename, trans)
            count += 1
            input("🔄 Move robot to new pose and press Enter to capture next...")
        except Exception as e:
            print(f"[❌] Error: {e}")
        rate.sleep()

