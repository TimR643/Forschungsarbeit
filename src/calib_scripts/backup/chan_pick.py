#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon
import actionlib
from geometry_msgs.msg import PoseStamped

class PickPlaceExecutor:
    def __init__(self):
        rospy.init_node("pick_place_executor")

        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)

        self.gripper_grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        rospy.loginfo("⏳ Waiting for gripper action servers...")
        self.gripper_grasp_client.wait_for_server()
        self.gripper_move_client.wait_for_server()
        rospy.loginfo("✅ Gripper servers ready.")

        rospy.Subscriber("/red_object_pose", PoseStamped, self.pick_and_place_callback)
        rospy.loginfo("🎯 Ready to pick RED objects.")
        rospy.spin()

    def open_gripper(self):
        goal = MoveGoal(width=0.08, speed=0.1)
        self.gripper_move_client.send_goal(goal)
        self.gripper_move_client.wait_for_result()

    def close_gripper(self):
        goal = GraspGoal()
        goal.width = 0.01
        goal.epsilon = GraspEpsilon(inner=0.005, outer=0.005)
        goal.speed = 0.05
        goal.force = 5.0
        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result()

    def move_to_pose(self, x, y, z, orientation=None, desc="target"):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        if orientation is None:
            # Default: downward facing
            #quat = tf.transformations.quaternion_from_euler(-3.14, 0, -3.14 / 2)
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
        else:
            pose.orientation = orientation

        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Moving to {desc}...")
            self.arm_group.execute(plan[1], wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            return True
        else:
            rospy.logwarn(f"❌ Failed to plan to {desc}")
            return False

    def pick_and_place_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        rospy.loginfo(f"📥 Received RED object pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        z_above = z + 0.15
        z_grasp = z + 0.015

        self.open_gripper()
        self.move_to_pose(x, y, z_above, desc="above object")
        self.move_to_pose(x, y, z_grasp, desc="at grasp")
        self.close_gripper()
        rospy.sleep(1.0)
        self.move_to_pose(x, y, z_above, desc="lifted")

        # Fixed drop location
        drop_x, drop_y, drop_z = 0.4, -0.2, 0.3
        self.move_to_pose(drop_x, drop_y, drop_z + 0.15, desc="above drop")
        self.move_to_pose(drop_x, drop_y, drop_z, desc="at drop")
        self.open_gripper()
        rospy.sleep(1.0)
        self.move_to_pose(drop_x, drop_y, drop_z + 0.15, desc="lift after drop")
        rospy.loginfo("✅ Pick and place completed.")

if __name__ == "__main__":
    PickPlaceExecutor()
