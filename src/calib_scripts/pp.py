#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal ,GraspEpsilon
import actionlib

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

    # Gripper pointing straight down
    quat = tf_trans.quaternion_from_euler(0, 0, 0)
    #pose.orientation.x = quat[0]
    #pose.orientation.y = quat[1]
    #pose.orientation.z = quat[2]
    #pose.orientation.w = quat[3]
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0
    print(quat)
    
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
    rospy.init_node('pick_place_left_to_right')
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_planning_time(10)
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()
    move_client.wait_for_server()

    # Z height for pick and place
    z_height = 0.2

    # Step 1: Ready Pose
    go_to_ready_pose(group)
    open_gripper(move_client)
    
    # Move
    move_to_pose(group, x=0.337 , y=-0.016, z=0.02, name="pick")

    # Step 2: Pick Pose 
    move_to_pose(group, x=0.337 , y=-0.016, z=0.02, name="pick")
    close_gripper(grasp_client)
    move_to_pose(group, x=0.337, y=-0.016, z=0.02, name="pick")
    rospy.sleep(1)

    # Step 3: Lift slightly
    move_to_pose(group, x=0.45, y=0.0, z=0.4, name="carry")

    # Step 4: Place Pose
    move_to_pose(group, x=0.45, y=-0.2, z=0.02, name="move to place")
    open_gripper(move_client)
    move_to_pose(group, x=0.45, y=-0.2, z=0.4, name="place")
    rospy.sleep(1)

    # Step 5: Return to Ready
    go_to_ready_pose(group)
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()

