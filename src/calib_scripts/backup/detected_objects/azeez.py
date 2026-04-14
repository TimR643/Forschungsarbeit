#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import json
import sys
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

# === Load object position from JSON ===
try:
    with open("red_object_160428.json") as f:
        data = json.load(f)
    target_position = np.array(data["position"])
    print(f"📥 Loaded object position: {target_position.tolist()}")
except Exception as e:
    print(f"❌ Error loading JSON file: {e}")
    sys.exit(1)

# === Set safe pre-grasp position ===
pre_grasp_offset = 0.25  # 25 cm above object
pre_grasp_position = target_position.copy()
pre_grasp_position[2] += pre_grasp_offset
pre_grasp_position[2] = max(pre_grasp_position[2], 0.02)  # never go below 2cm

# === Init ROS and MoveIt Commander ===
rospy.init_node("pick_and_place_debug", anonymous=True)
moveit_commander.roscpp_initialize([])
arm = moveit_commander.MoveGroupCommander("panda_arm")

arm.set_pose_reference_frame("panda_link0")
arm.set_planning_time(5)
arm.set_max_velocity_scaling_factor(0.1)
arm.set_max_acceleration_scaling_factor(0.1)

print(f"📌 Planning frame used: {arm.get_planning_frame()}")  # 🔍 Confirm frame

# === Move to a pose with fixed top-down orientation ===
def move_to_pose(position, label):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    # Use Euler angles: roll=-pi (flip), pitch=0, yaw=pi/2 (rotate)
    qx, qy, qz, qw = quaternion_from_euler(-np.pi, 0, np.pi / 2)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    print(f"\n📌 Moving to [{label}] pose:")
    print(f"   ➤ Position: X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}")
    print(f"   ➤ Orientation: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}")

    arm.set_pose_target(pose)
    success = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    if success:
        print(f"✅ Moved to [{label}] pose.")
    else:
        print(f"❌ Failed to move to [{label}] pose.")
    return success

# === Main pick logic ===
print("🔼 Moving to pre-grasp...")
if move_to_pose(pre_grasp_position, "pre-grasp"):
    print("⬇️ Moving to grasp position...")
    if move_to_pose(target_position, "grasp"):
        print("🤏 Simulating gripper close...")
        rospy.sleep(1.0)
        print("⬆️ Lifting object...")
        lifted_position = target_position.copy()
        lifted_position[2] += 0.1
        move_to_pose(lifted_position, "lift")
    else:
        print("❌ Grasp move failed.")
else:
    print("❌ Pre-grasp move failed.")

moveit_commander.roscpp_shutdown()
