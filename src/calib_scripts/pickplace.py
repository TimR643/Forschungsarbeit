#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon
import actionlib
import json

def open_gripper(move_client):
    goal = MoveGoal(width=0.08, speed=0.05)
    move_client.send_goal(goal)
    move_client.wait_for_result()

def close_gripper(grasp_client):
    goal = GraspGoal()
    goal.width = 0.005
    goal.speed = 0.02
    goal.force = 90.0
    goal.epsilon = GraspEpsilon(inner=0.05, outer=0.05)
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()

def move_to_pose(group, x, y, z=0.2, name="target"):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    # Gripper pointing straight down
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0

    rospy.loginfo(f"[MOVE] Moving to {name}: ({x:.3f}, {y:.3f}, {z:.3f})")
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def go_to_ready_pose(group):
    try:
        group.set_named_target("ready")
        group.go(wait=True)
        group.stop()
    except:
        rospy.logwarn("Named target 'ready' not found. Using fallback joint angles.")
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -0.785
        joint_goal[2] = 0.0
        joint_goal[3] = -2.356
        joint_goal[4] = 0.0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785
        group.go(joint_goal, wait=True)
        group.stop()

def load_xy_from_json(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data['position'][0], data['position'][1]  # x, y

def main():
    rospy.init_node('pick_place_from_json')
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_planning_time(10)
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()
    move_client.wait_for_server()

    # Load only pick x, y from JSON
    red_object_path = "/home/roslab/catkin_ws/src/calib_scripts/detected_objects/red_object_1752922638_with_pose.json"  # Update this path
    x, y = load_xy_from_json(red_object_path)

    # Step 1: Ready Pose
    go_to_ready_pose(group)
    open_gripper(move_client)

    # Step 2: Move above pick (from JSON)
    move_to_pose(group, x+0.1, y, z=0.02, name="above pick")

    # Step 3: Move to pick
    move_to_pose(group, x+0.1, y, z=0.023, name="pick")
    close_gripper(grasp_client)

    # Step 4: Lift (fixed location)
    move_to_pose(group, x+0.1, y=0.0, z=0.40, name="lift")

    # Step 5: Place (fixed location)
    move_to_pose(group, x=0.608, y=0.302, z=0.02, name="place")
    open_gripper(move_client)
    move_to_pose(group, x=0.608, y=0.302, z=0.02, name="lift from place")

    # Step 6: Return to Ready
    go_to_ready_pose(group)
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

