#!/usr/bin/env python3

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    import actionlib
    from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
except Exception:
    actionlib = None
    GraspAction = None
    GraspGoal = None
    MoveAction = None
    MoveGoal = None

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



def _axis_value(state, axis_name):
    if axis_name == "x":
        return getattr(state, "x", 0.0)
    if axis_name == "y":
        return getattr(state, "y", 0.0)
    if axis_name == "z":
        return getattr(state, "z", 0.0)
    if axis_name == "roll":
        return getattr(state, "roll", 0.0)
    if axis_name == "pitch":
        return getattr(state, "pitch", 0.0)
    if axis_name == "yaw":
        return getattr(state, "yaw", 0.0)
    return 0.0


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

        # Optional axis remap for angular commands (valid: roll, pitch, yaw).
        self.angular_axis_map_x = str(rospy.get_param("~angular_axis_map_x", "roll"))
        self.angular_axis_map_y = str(rospy.get_param("~angular_axis_map_y", "yaw"))
        self.angular_axis_map_z = str(rospy.get_param("~angular_axis_map_z", "pitch"))

        self.require_deadman = rospy.get_param("~require_deadman", False)
        self.deadman_button_index = int(rospy.get_param("~deadman_button_index", 0))
        self.enable_button_index = _normalize_button_index(rospy.get_param("~enable_button_index", -1))
        self.disable_button_index = _normalize_button_index(rospy.get_param("~disable_button_index", -1))

        # Optional short-press button mapping for gripper.
        self.gripper_short_press_max_duration = float(rospy.get_param("~gripper_short_press_max_duration", 0.35))

        self.gripper_close_button_index = _normalize_button_index(rospy.get_param("~gripper_close_button_index", 1))
        self.gripper_close_width = float(rospy.get_param("~gripper_close_width", 0.0))
        self.gripper_close_speed = float(rospy.get_param("~gripper_close_speed", 0.03))
        self.gripper_close_force = float(rospy.get_param("~gripper_close_force", 40.0))

        self.gripper_open_button_index = _normalize_button_index(rospy.get_param("~gripper_open_button_index", 0))
        self.gripper_open_width = float(rospy.get_param("~gripper_open_width", 0.08))
        self.gripper_open_speed = float(rospy.get_param("~gripper_open_speed", 0.05))

        self.gripper_action_timeout = float(rospy.get_param("~gripper_action_timeout", 5.0))

        self.enabled = rospy.get_param("~start_enabled", True)

        self.twist_pub = rospy.Publisher("~twist", Twist, queue_size=10)
        self.enabled_pub = rospy.Publisher("~enabled", Bool, queue_size=10, latch=True)

        self._close_button_prev_pressed = False
        self._close_button_press_t0 = None
        self._open_button_prev_pressed = False
        self._open_button_press_t0 = None

        self._gripper_action_active = False
        self._gripper_lock = threading.Lock()

        self.gripper_close_client = None
        self.gripper_open_client = None
        if self.gripper_close_button_index is not None or self.gripper_open_button_index is not None:
            if actionlib is None:
                rospy.logwarn("Gripper button mapping is set, but actionlib/franka_gripper dependencies are unavailable")
            else:
                if self.gripper_close_button_index is not None and GraspAction is not None:
                    self.gripper_close_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
                if self.gripper_open_button_index is not None and MoveAction is not None:
                    self.gripper_open_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

        self.enabled_pub.publish(Bool(data=self.enabled))
        rospy.loginfo("spacemouse_twist_node started. enabled=%s", self.enabled)

    def _compute_twist(self, state):
        lx = _signed(_apply_deadzone(getattr(state, "x", 0.0), self.deadzone), self.invert_linear_x)
        ly = _signed(_apply_deadzone(getattr(state, "y", 0.0), self.deadzone), self.invert_linear_y)
        lz = _signed(_apply_deadzone(getattr(state, "z", 0.0), self.deadzone), self.invert_linear_z)

        ax = _signed(_apply_deadzone(_axis_value(state, self.angular_axis_map_x), self.deadzone), self.invert_angular_x)
        ay = _signed(_apply_deadzone(_axis_value(state, self.angular_axis_map_y), self.deadzone), self.invert_angular_y)
        az = _signed(_apply_deadzone(_axis_value(state, self.angular_axis_map_z), self.deadzone), self.invert_angular_z)

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

    def _trigger_gripper_close_action(self):
        if self.gripper_close_client is None:
            return

        with self._gripper_lock:
            if self._gripper_action_active:
                return
            self._gripper_action_active = True

        def _run():
            try:
                if not self.gripper_close_client.wait_for_server(rospy.Duration(1.0)):
                    rospy.logwarn("Could not reach /franka_gripper/grasp action server")
                    return

                goal = GraspGoal()
                goal.width = self.gripper_close_width
                goal.speed = self.gripper_close_speed
                goal.force = self.gripper_close_force
                goal.epsilon.inner = 0.01
                goal.epsilon.outer = 0.01

                rospy.loginfo("SpaceMouse short press: closing gripper (width=%.3f, speed=%.3f, force=%.1f)",
                              self.gripper_close_width, self.gripper_close_speed, self.gripper_close_force)
                self.gripper_close_client.send_goal(goal)
                self.gripper_close_client.wait_for_result(rospy.Duration(self.gripper_action_timeout))
            finally:
                with self._gripper_lock:
                    self._gripper_action_active = False

        threading.Thread(target=_run, daemon=True).start()

    def _trigger_gripper_open_action(self):
        if self.gripper_open_client is None:
            return

        with self._gripper_lock:
            if self._gripper_action_active:
                return
            self._gripper_action_active = True

        def _run():
            try:
                if not self.gripper_open_client.wait_for_server(rospy.Duration(1.0)):
                    rospy.logwarn("Could not reach /franka_gripper/move action server")
                    return

                goal = MoveGoal()
                goal.width = self.gripper_open_width
                goal.speed = self.gripper_open_speed

                rospy.loginfo("SpaceMouse short press: opening gripper (width=%.3f, speed=%.3f)",
                              self.gripper_open_width, self.gripper_open_speed)
                self.gripper_open_client.send_goal(goal)
                self.gripper_open_client.wait_for_result(rospy.Duration(self.gripper_action_timeout))
            finally:
                with self._gripper_lock:
                    self._gripper_action_active = False

        threading.Thread(target=_run, daemon=True).start()

    def _update_short_press_action(self, state, button_index, prev_pressed, press_t0, action_cb):
        if button_index is None:
            return prev_pressed, press_t0

        pressed = _button_pressed(state, button_index)
        now = rospy.Time.now()

        if pressed and not prev_pressed:
            press_t0 = now

        if (not pressed) and prev_pressed and press_t0 is not None:
            dt = (now - press_t0).to_sec()
            if dt <= self.gripper_short_press_max_duration:
                action_cb()

        if not pressed:
            press_t0 = None

        prev_pressed = pressed
        return prev_pressed, press_t0

    def _update_gripper_short_press(self, state):
        self._close_button_prev_pressed, self._close_button_press_t0 = self._update_short_press_action(
            state,
            self.gripper_close_button_index,
            self._close_button_prev_pressed,
            self._close_button_press_t0,
            self._trigger_gripper_close_action,
        )

        self._open_button_prev_pressed, self._open_button_press_t0 = self._update_short_press_action(
            state,
            self.gripper_open_button_index,
            self._open_button_prev_pressed,
            self._open_button_press_t0,
            self._trigger_gripper_open_action,
        )

    def run(self):
        if pyspacemouse is None:
            rospy.logfatal("Could not initialize pyspacemouse: %s", IMPORT_ERROR)
            rospy.logfatal("Hint (Noetic/Python 3.8): python3 -m pip uninstall -y pyspacemouse && python3 -m pip install --user \"pyspacemouse<2.0\"")
            raise RuntimeError("pyspacemouse unavailable or incompatible")

        rate = rospy.Rate(self.publish_rate)

        opened = pyspacemouse.open()

        # pyspacemouse >=2.0: context manager/device object with .read()
        # pyspacemouse 1.x: open() returns bool and read() is module-level
        if hasattr(opened, "__enter__"):
            with opened as device:
                while not rospy.is_shutdown():
                    state = device.read()
                    if state is None:
                        rate.sleep()
                        continue

                    self._update_enable_state(state)
                    self._update_gripper_short_press(state)

                    deadman_ok = True
                    if self.require_deadman:
                        deadman_ok = _button_pressed(state, int(self.deadman_button_index))

                    if self.enabled and deadman_ok:
                        twist = self._compute_twist(state)
                    else:
                        twist = Twist()

                    self.twist_pub.publish(twist)
                    rate.sleep()
        else:
            if not opened:
                raise RuntimeError("Could not open SpaceMouse device")

            try:
                while not rospy.is_shutdown():
                    state = pyspacemouse.read()
                    if state is None:
                        rate.sleep()
                        continue

                    self._update_enable_state(state)
                    self._update_gripper_short_press(state)

                    deadman_ok = True
                    if self.require_deadman:
                        deadman_ok = _button_pressed(state, int(self.deadman_button_index))

                    if self.enabled and deadman_ok:
                        twist = self._compute_twist(state)
                    else:
                        twist = Twist()

                    self.twist_pub.publish(twist)
                    rate.sleep()
            finally:
                close_fn = getattr(pyspacemouse, "close", None)
                if callable(close_fn):
                    close_fn()


def main():
    rospy.init_node("spacemouse_twist_node")
    node = SpaceMouseTwistNode()
    node.run()


if __name__ == "__main__":
    main()
