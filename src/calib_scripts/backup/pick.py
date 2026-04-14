#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import json

# Load object position from JSON
with open("red_object_124438.json") as f:
    data = json.load(f)

target_position = np.array(data["position"])  # [x, y, z]

# Define pre-grasp and grasp heights
pre_grasp_offset = 0.20  # 10 cm above object
grasp_height = target_position[2]  # original height
pre_grasp_position = target_position.copy()
pre_grasp_position[2] += pre_grasp_offset

# Initialize ROS node and MoveIt
rospy.init_node("pick_red_object", anonymous=True)
moveit_commander.roscpp_initialize([])

arm = moveit_commander.MoveGroupCommander("panda_manipulator")
arm.set_pose_reference_frame("panda_link0")
arm.set_planning_time(5)
arm.set_max_velocity_scaling_factor(0.1)
arm.set_max_acceleration_scaling_factor(0.1)

# Helper: move to a target pose
def move_to_pose(position):
    pose = arm.get_current_pose().pose
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    # Set orientation: vertical gripper pointing down (identity rotation)
    pose.orientation.x = 0.707
    pose.orientation.y = 0.0
    pose.orientation.z = 0.707
    pose.orientation.w = 0.0
    arm.set_pose_target(pose)
    success = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    return success

print("🔼 Moving to pre-grasp...")
if move_to_pose(pre_grasp_position):
    print("⬇️ Moving to grasp position...")
    if move_to_pose(target_position):
        print("🤏 Simulating gripper close...")
        rospy.sleep(1.0)
        print("⬆️ Lifting object...")
        lifted_position = target_position.copy()
        lifted_position[2] += 0.1
        move_to_pose(lifted_position)
    else:
        print("❌ Failed to reach grasp pose")
else:
    print("❌ Failed to reach pre-grasp pose")

moveit_commander.roscpp_shutdown()
