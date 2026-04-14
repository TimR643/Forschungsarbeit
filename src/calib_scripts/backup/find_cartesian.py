#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

def load_transform(path):
    T = np.load(path)
    return T

def broadcast_tf(T, parent_frame, child_frame, br):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = T[0, 3]
    t.transform.translation.y = T[1, 3]
    t.transform.translation.z = T[2, 3]

    quat = tf.transformations.quaternion_from_matrix(T)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

def publish_marker(pub):
    marker = Marker()
    marker.header.frame_id = "camera_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "test_marker"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.25  # 25 cm in front of camera

    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.05
    marker.scale.y = 0.01
    marker.scale.z = 0.01

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("camera_tf_visualizer")

    T = load_transform("T_camera_to_ee.npy")  # Ensure this file is in working dir
    br = tf2_ros.TransformBroadcaster()
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)

    rate = rospy.Rate(10)
    rospy.loginfo("📡 Broadcasting camera_frame transform and publishing marker...")

    while not rospy.is_shutdown():
        broadcast_tf(T, "panda_link8", "camera_frame", br)
        publish_marker(marker_pub)
        rate.sleep()
