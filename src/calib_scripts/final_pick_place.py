#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon
import actionlib
import json

def open_gripper(move_client):
    goal = MoveGoal(width=0.08, speed=0.05)
    move_client.send_goal(goal)
    move_client.wait_for_result()

def close_gripper(grasp_client):
    goal = GraspGoal(width=0.005, speed=0.02, force=90.0, epsilon=GraspEpsilon(inner=0.05, outer=0.05))
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()

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
    return data['position'][0], data['position'][1]

def move_to_pose(group, pose_target):
    group.set_pose_target(pose_target)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def plan_pose(x, y, z):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0
    return pose

def pick_and_place_for_color(group, grasp_client, move_client, color):
    json_path = f"/home/roslab/catkin_ws/src/calib_scripts/detected_objects/{color}_object.json"
    x, y = load_xy_from_json(json_path)

    if color in ["red", "blue"]:
        x += 0.1  # Offset for red and blue pick positions

    safe_lift_z = 0.20
    above_pick_z = 0.05
    pick_z = 0.022

    place_positions = {
        "red":   (0.659,  0.228, 0.033),
        "green": (0.655,  0.063, 0.033),
        "blue":  (0.608, -0.146, 0.033)
    }

    place_x, place_y, place_z = place_positions[color]

    # ---- PICK PHASE ----
    pose_above_pick = plan_pose(x, y, safe_lift_z)
    move_to_pose(group, pose_above_pick)

    pose_approach = plan_pose(x, y, above_pick_z)
    move_to_pose(group, pose_approach)

    pose_pick = plan_pose(x, y, pick_z)
    move_to_pose(group, pose_pick)

    close_gripper(grasp_client)

    move_to_pose(group, pose_approach)
    move_to_pose(group, pose_above_pick)

    # ---- PLACE PHASE ----
    pose_above_place = plan_pose(place_x, place_y, safe_lift_z)
    move_to_pose(group, pose_above_place)

    pose_place = plan_pose(place_x, place_y, place_z)
    move_to_pose(group, pose_place)

    open_gripper(move_client)

    move_to_pose(group, pose_above_place)

def main():
    rospy.init_node('safe_pick_place_joint_space')
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_planning_time(10)
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()
    move_client.wait_for_server()

    go_to_ready_pose(group)
    open_gripper(move_client)

    for color in ["red", "blue", "green"]:
        rospy.loginfo(f"🚦 Starting pick and place for {color} object.")
        pick_and_place_for_color(group, grasp_client, move_client, color)

    go_to_ready_pose(group)
    rospy.loginfo("✅ Pick and place completed for all objects.")

if __name__ == '__main__':
    main()