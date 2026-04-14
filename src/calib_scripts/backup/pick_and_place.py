#!/usr/bin/env python3

import sys
import csv
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon
import actionlib
import os

def open_gripper(move_client):
    goal = MoveGoal(width=0.08, speed=0.05)
    move_client.send_goal(goal)
    move_client.wait_for_result()

def close_gripper(grasp_client):
    goal = GraspGoal()
    goal.width = 0.02
    goal.speed = 0.02
    goal.force = 50.0
    goal.epsilon = GraspEpsilon(inner=0.01, outer=0.01)
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()

def move_to_pose(group, x, y, z=0.2, name="target"):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    # Gripper orientation exactly same as your detection node
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

def main():
    rospy.init_node('pick_place_red_objects', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_planning_time(10)
    group.set_max_velocity_scaling_factor(0.05)
    group.set_max_acceleration_scaling_factor(0.05)

    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()
    move_client.wait_for_server()

    z_offset = -0.02  # lower pick height by 2cm to avoid picking in air

    csv_path = os.path.expanduser('~/catkin_ws/src/calib_scripts/detected_objects.csv')
    red_objects = []

    # Read red objects from CSV
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['color'].strip().lower() == 'red':
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                red_objects.append((x, y, z))

    if not red_objects:
        rospy.logwarn("No red objects found in CSV.")
        return

    go_to_ready_pose(group)
    open_gripper(move_client)

    for i, (x, y, z) in enumerate(red_objects):
        rospy.loginfo(f"[{i+1}/{len(red_objects)}] Picking red object at ({x:.3f}, {y:.3f}, {z:.3f})")

        # Approach position above object
        move_to_pose(group, x=x, y=y, z=z + 0.2, name=f"above red object {i+1}")

        # Move down to pick position with offset
        move_to_pose(group, x=x, y=y, z=z + z_offset, name=f"pick red object {i+1}")
        close_gripper(grasp_client)

        # Lift up
        move_to_pose(group, x=x, y=y, z=z + 0.2, name=f"lift red object {i+1}")
        rospy.sleep(1)

        # Fixed place location for red objects
        place_x, place_y, place_z = 0.45, -0.2, z + 0.02
        move_to_pose(group, x=place_x, y=place_y, z=place_z, name=f"place red object {i+1}")
        open_gripper(move_client)

        # Retract
        move_to_pose(group, x=place_x, y=place_y, z=place_z + 0.2, name=f"retract after placing red object {i+1}")
        rospy.sleep(1)

        go_to_ready_pose(group)

    rospy.loginfo("Finished pick and place for all red objects.")

if __name__ == '__main__':
    main()
