#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

try:
    from franka_msgs.msg import FrankaState
except ImportError:
    FrankaState = None


def _quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_from_rot_matrix(m):
    # m: 3x3 nested list
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return _quat_normalize((x, y, z, w))


class SpaceMouseEquilibriumPoseNode:
    def __init__(self):
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)
        self.frame_id = rospy.get_param("~frame_id", "panda_link0")

        self.linear_gain = rospy.get_param("~linear_gain", 1.0)
        self.angular_gain = rospy.get_param("~angular_gain", 1.0)

        self.max_delta_pos = rospy.get_param("~max_delta_pos", 0.003)
        self.max_delta_rot = rospy.get_param("~max_delta_rot", 0.05)

        self.enabled = False
        self.last_twist = Twist()
        self.have_init_pose = False

        self.pos = [0.0, 0.0, 0.0]
        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.lock = threading.Lock()

        self.pose_pub = rospy.Publisher("~equilibrium_pose", PoseStamped, queue_size=10)

        if FrankaState is None:
            raise RuntimeError("franka_msgs not available; cannot subscribe to FrankaState")

        rospy.Subscriber("~franka_state", FrankaState, self._on_franka_state, queue_size=1)
        rospy.Subscriber("~twist_in", Twist, self._on_twist, queue_size=1)
        rospy.Subscriber("~enabled_in", Bool, self._on_enabled, queue_size=1)

    def _on_enabled(self, msg):
        with self.lock:
            self.enabled = bool(msg.data)

    def _on_twist(self, msg):
        with self.lock:
            self.last_twist = msg

    def _on_franka_state(self, msg):
        with self.lock:
            if self.have_init_pose:
                return

            t = msg.O_T_EE
            # libfranka uses column-major 4x4 transform.
            rot = [
                [t[0], t[4], t[8]],
                [t[1], t[5], t[9]],
                [t[2], t[6], t[10]],
            ]
            self.pos = [t[12], t[13], t[14]]
            self.quat = _quat_from_rot_matrix(rot)
            self.have_init_pose = True
            rospy.loginfo("spacemouse_equilibrium_pose_node initialized from FrankaState")

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        prev_t = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1e-4, (now - prev_t).to_sec())
            prev_t = now

            with self.lock:
                if not self.have_init_pose:
                    rate.sleep()
                    continue

                twist = self.last_twist
                enabled = self.enabled

                if enabled:
                    dx = max(-self.max_delta_pos, min(self.max_delta_pos, twist.linear.x * self.linear_gain * dt))
                    dy = max(-self.max_delta_pos, min(self.max_delta_pos, twist.linear.y * self.linear_gain * dt))
                    dz = max(-self.max_delta_pos, min(self.max_delta_pos, twist.linear.z * self.linear_gain * dt))

                    self.pos[0] += dx
                    self.pos[1] += dy
                    self.pos[2] += dz

                    rx = max(-self.max_delta_rot, min(self.max_delta_rot, twist.angular.x * self.angular_gain * dt))
                    ry = max(-self.max_delta_rot, min(self.max_delta_rot, twist.angular.y * self.angular_gain * dt))
                    rz = max(-self.max_delta_rot, min(self.max_delta_rot, twist.angular.z * self.angular_gain * dt))

                    # small-angle quaternion increment
                    dq = _quat_normalize((0.5 * rx, 0.5 * ry, 0.5 * rz, 1.0))
                    self.quat = _quat_normalize(_quat_mul(self.quat, dq))

                pose = PoseStamped()
                pose.header.stamp = now
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = self.pos[0]
                pose.pose.position.y = self.pos[1]
                pose.pose.position.z = self.pos[2]
                pose.pose.orientation.x = self.quat[0]
                pose.pose.orientation.y = self.quat[1]
                pose.pose.orientation.z = self.quat[2]
                pose.pose.orientation.w = self.quat[3]

            self.pose_pub.publish(pose)
            rate.sleep()


def main():
    rospy.init_node("spacemouse_equilibrium_pose_node")
    node = SpaceMouseEquilibriumPoseNode()
    node.run()


if __name__ == "__main__":
    main()
