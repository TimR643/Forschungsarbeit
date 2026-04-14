#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import json

# Load object position from saved JSON
with open("red_object_173629.json") as f:
    data = json.load(f)

xyz_base = np.array(data["position"])

# Adjust for pre-grasp and lift positions
pre_grasp_offset = 0.15  # meters above
lift_offset = 0.10       # meters after grasp

pre_grasp_pos = xyz_base.copy()
pre_grasp_pos[2] += pre_grasp_offset

lift_pos = xyz_base.copy()
lift_pos[2] += lift_offset

# Initialize ROS node and MoveIt commander
rospy.init_node("pick_red_object", anonymous=True)
moveit_commander.roscpp_initialize([])

arm = moveit_commander.MoveGroupCommander("panda_arm")
arm.set_pose_reference_frame("panda_link0")
arm.set_planning_time(5)
arm.set_max_velocity_scaling_factor(0.1)
arm.set_max_acceleration_scaling_factor(0.1)

# Set fixed downward gripper orientation
def set_orientation(pose):
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose

# Move to a specific pose and print debug info
def move_to_position(position, label):
    pose = arm.get_current_pose().pose
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose = set_orientation(pose)

    print(f"\n📌 Moving to [{label}] pose:")
    print(f"   ➤ Position: X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f}")
    print(f"   ➤ Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")

    arm.set_pose_target(pose)
    success = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    if success:
        print(f"✅ Moved to [{label}] pose.")
    else:
        print(f"❌ Failed to move to [{label}] pose.")
    return success

# Execute pick sequence
if move_to_position(pre_grasp_pos, "pre-grasp"):
    if move_to_position(xyz_base, "grasp"):
        print("🤏 Simulating gripper close...")
        rospy.sleep(1.0)
        move_to_position(lift_pos, "lift")

moveit_commander.roscpp_shutdown()
