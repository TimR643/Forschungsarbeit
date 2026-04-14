#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class FrameVisualizer:
    def __init__(self):
        rospy.init_node("frame_visualizer")

        self.frames = ["panda_link0", "panda_hand", "camera_link"]
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("🔧 Visualizing frames: " + ", ".join(self.frames))

        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_all_frames()
            self.rate.sleep()

    def publish_all_frames(self):
        for i, frame in enumerate(self.frames):
            try:
                transform = self.tf_buffer.lookup_transform("panda_link0", frame, rospy.Time(0), rospy.Duration(1.0))
                self.publish_axes_marker(transform, i)
            except Exception as e:
                rospy.logwarn(f"[WARN] Could not get transform to {frame}: {e}")

    def publish_axes_marker(self, tf_msg, marker_id):
        # Extract origin
        origin = tf_msg.transform.translation
        pos = Point(origin.x, origin.y, origin.z)

        # Create three arrow markers for X, Y, Z
        axes = ['x', 'y', 'z']
        colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # RGB
        directions = [
            (0.1, 0, 0),
            (0, 0.1, 0),
            (0, 0, 0.1)
        ]

        for i, axis in enumerate(axes):
            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"{tf_msg.child_frame_id}_{axis}"
            marker.id = marker_id * 10 + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # shaft diameter
            marker.scale.y = 0.02  # head diameter
            marker.scale.z = 0.0
            marker.color.r, marker.color.g, marker.color.b = colors[i]
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)

            start = Point(pos.x, pos.y, pos.z)
            end = Point(
                pos.x + directions[i][0],
                pos.y + directions[i][1],
                pos.z + directions[i][2]
            )

            marker.points = [start, end]
            self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        FrameVisualizer()
    except rospy.ROSInterruptException:
        pass
