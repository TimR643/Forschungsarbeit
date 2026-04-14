#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
import actionlib

def open_gripper(move_client):
    rospy.loginfo("[GRIPPER] Opening gripper...")
    goal = MoveGoal(width=0.08, speed=0.05)
    move_client.send_goal(goal)
    move_client.wait_for_result()

def close_gripper(grasp_client):
    rospy.loginfo("[GRIPPER] Closing gripper...")
    goal = GraspGoal()
    goal.width = 0.0
    goal.speed = 0.05
    goal.force = 5.0
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()

def move_to_pose(group, x, y, z, name="pose"):
    rospy.loginfo(f"[MOVEIT] Moving to {name}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Orientation: Gripper pointing straight down (no tilt)
    quat = tf_trans.quaternion_from_euler(3.14, 0, -1.57)
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]

    group.set_pose_target(pose_target)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def main():
    rospy.init_node('pick_and_place_fixed')
    moveit_commander.roscpp_initialize(sys.argv)

    # MoveIt interfaces
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_planning_time(10)
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    # Gripper clients
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

    rospy.loginfo("[ACTION] Waiting for gripper action servers...")
    grasp_client.wait_for_server()
    move_client.wait_for_server()
    rospy.loginfo("[ACTION] Gripper action servers connected.")

    # Start from ready pose
    move_to_pose(group, 0.3, 0.0, 0.5, name="ready")
    open_gripper(move_client)

    # Pick
    move_to_pose(group, 0.4, 0.0, 0.2, name="pick")
    rospy.sleep(1.0)
    close_gripper(grasp_client)
    rospy.sleep(1.0)

    # Lift
    move_to_pose(group, 0.4, 0.0, 0.4, name="lift")

    # Place
    move_to_pose(group, 0.3, -0.3, 0.3, name="place")
    open_gripper(move_client)
    rospy.sleep(1.0)

    # Return to ready pose
    move_to_pose(group, 0.3, 0.0, 0.5, name="ready")

    rospy.loginfo("[INFO] Pick and place task completed successfully.")

if __name__ == '__main__':
    main()

