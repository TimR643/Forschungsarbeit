#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    import pyspacemouse
except Exception as exc:
    pyspacemouse = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def _apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _signed(value, invert):
    return -value if invert else value


def _button_pressed(state, index):
    buttons = getattr(state, "buttons", None)
    if not buttons or index is None:
        return False
    if index < 0 or index >= len(buttons):
        return False
    return bool(buttons[index])


def _normalize_button_index(value):
    if value is None:
        return None
    idx = int(value)
    return None if idx < 0 else idx


class SpaceMouseTwistNode:
    def __init__(self):
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)
        self.deadzone = rospy.get_param("~deadzone", 0.05)

        self.max_linear = rospy.get_param("~max_linear", 0.03)
        self.max_angular = rospy.get_param("~max_angular", 0.20)

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        self.invert_linear_x = rospy.get_param("~invert_linear_x", False)
        self.invert_linear_y = rospy.get_param("~invert_linear_y", False)
        self.invert_linear_z = rospy.get_param("~invert_linear_z", False)
        self.invert_angular_x = rospy.get_param("~invert_angular_x", False)
        self.invert_angular_y = rospy.get_param("~invert_angular_y", False)
        self.invert_angular_z = rospy.get_param("~invert_angular_z", False)

        self.require_deadman = rospy.get_param("~require_deadman", True)
        self.deadman_button_index = int(rospy.get_param("~deadman_button_index", 0))
        self.enable_button_index = _normalize_button_index(rospy.get_param("~enable_button_index", -1))
        self.disable_button_index = _normalize_button_index(rospy.get_param("~disable_button_index", -1))

        self.enabled = rospy.get_param("~start_enabled", False)

        self.twist_pub = rospy.Publisher("~twist", Twist, queue_size=10)
        self.enabled_pub = rospy.Publisher("~enabled", Bool, queue_size=10, latch=True)

        self.enabled_pub.publish(Bool(data=self.enabled))
        rospy.loginfo("spacemouse_twist_node started. enabled=%s", self.enabled)

    def _compute_twist(self, state):
        lx = _signed(_apply_deadzone(getattr(state, "x", 0.0), self.deadzone), self.invert_linear_x)
        ly = _signed(_apply_deadzone(getattr(state, "y", 0.0), self.deadzone), self.invert_linear_y)
        lz = _signed(_apply_deadzone(getattr(state, "z", 0.0), self.deadzone), self.invert_linear_z)

        ax = _signed(_apply_deadzone(getattr(state, "roll", 0.0), self.deadzone), self.invert_angular_x)
        ay = _signed(_apply_deadzone(getattr(state, "pitch", 0.0), self.deadzone), self.invert_angular_y)
        az = _signed(_apply_deadzone(getattr(state, "yaw", 0.0), self.deadzone), self.invert_angular_z)

        twist = Twist()
        twist.linear.x = _clamp(lx * self.linear_scale * self.max_linear, self.max_linear)
        twist.linear.y = _clamp(ly * self.linear_scale * self.max_linear, self.max_linear)
        twist.linear.z = _clamp(lz * self.linear_scale * self.max_linear, self.max_linear)

        twist.angular.x = _clamp(ax * self.angular_scale * self.max_angular, self.max_angular)
        twist.angular.y = _clamp(ay * self.angular_scale * self.max_angular, self.max_angular)
        twist.angular.z = _clamp(az * self.angular_scale * self.max_angular, self.max_angular)
        return twist

    def _update_enable_state(self, state):
        if self.enable_button_index is not None and _button_pressed(state, self.enable_button_index):
            if not self.enabled:
                self.enabled = True
                self.enabled_pub.publish(Bool(data=True))
                rospy.logwarn("Teleop enabled via SpaceMouse button %d", self.enable_button_index)

        if self.disable_button_index is not None and _button_pressed(state, self.disable_button_index):
            if self.enabled:
                self.enabled = False
                self.enabled_pub.publish(Bool(data=False))
                rospy.logwarn("Teleop disabled via SpaceMouse button %d", self.disable_button_index)

    def run(self):
        if pyspacemouse is None:
            rospy.logfatal("Could not initialize pyspacemouse: %s", IMPORT_ERROR)
            rospy.logfatal("Hint (Noetic/Python 3.8): python3 -m pip uninstall -y pyspacemouse && python3 -m pip install --user \"pyspacemouse<2.0\"")
            raise RuntimeError("pyspacemouse unavailable or incompatible")

        rate = rospy.Rate(self.publish_rate)

        with pyspacemouse.open() as device:
            while not rospy.is_shutdown():
                state = device.read()
                if state is None:
                    rate.sleep()
                    continue

                self._update_enable_state(state)

                deadman_ok = True
                if self.require_deadman:
                    deadman_ok = _button_pressed(state, int(self.deadman_button_index))

                if self.enabled and deadman_ok:
                    twist = self._compute_twist(state)
                else:
                    twist = Twist()

                self.twist_pub.publish(twist)
                rate.sleep()


def main():
    rospy.init_node("spacemouse_twist_node")
    node = SpaceMouseTwistNode()
    node.run()


if __name__ == "__main__":
    main()
