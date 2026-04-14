#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import tf.transformations
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
import actionlib
import time

def move_to_pose(group, x, y, z, orientation=None, name="target"):
    rospy.loginfo(f"[INFO] Moving to {name}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    if orientation is None:
        # Final working orientation: horizontal gripper, forward Z
        quat = tf.transformations.quaternion_from_euler(-3.14, 0.0, -3.14 / 2)
        orientation = [quat[3], quat[0], quat[1], quat[2]]  # w, x, y, z

    pose.orientation.w = orientation[0]
    pose.orientation.x = orientation[1]
    pose.orientation.y = orientation[2]
    pose.orientation.z = orientation[3]

    group.set_start_state_to_current_state()
    group.set_pose_target(pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return success

def move_to_ready_pose(group):
    rospy.loginfo("[INFO] Moving to 'ready' position...")
    group.set_named_target("ready")
    plan = group.plan()
    if not plan or not plan[0]:
        rospy.logerr("[ERROR] Planning to ready pose failed.")
        return False
    group.execute(plan[1], wait=True)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo("[✓] Reached ready position.")
    return True

def open_gripper(move_client):
    goal = MoveGoal(width=0.08, speed=0.02)
    move_client.send_goal(goal)
    move_client.wait_for_result()
    time.sleep(1)

def close_gripper(grasp_client):
    goal = GraspGoal(width=0.03, speed=0.02, force=5.0)
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()
    time.sleep(1)

def pick_and_place(arm_group, grasp_client, move_client, pick_xy, place_xy, z_hover=0.5, z_pick=0.35):
    x_pick, y_pick = pick_xy
    x_place, y_place = place_xy

    # Use the same correct orientation here
    quat = tf.transformations.quaternion_from_euler(-3.14, 0.0, -3.14 / 2)
    orientation = [quat[3], quat[0], quat[1], quat[2]]

    move_to_pose(arm_group, x_pick, y_pick, z_hover, orientation, "pick_approach")
    move_to_pose(arm_group, x_pick, y_pick, z_pick, orientation, "pick")
    close_gripper(grasp_client)
    move_to_pose(arm_group, x_pick, y_pick, z_hover, orientation, "pick_retreat")

    move_to_pose(arm_group, x_place, y_place, z_hover, orientation, "place_approach")
    move_to_pose(arm_group, x_place, y_place, z_pick, orientation, "place")
    open_gripper(move_client)
    move_to_pose(arm_group, x_place, y_place, z_hover, orientation, "place_retreat")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gazebo_xy_pick_place', anonymous=True)

    arm_group = moveit_commander.MoveGroupCommander("panda_arm")
    arm_group.set_max_velocity_scaling_factor(0.05)
    arm_group.set_max_acceleration_scaling_factor(0.05)
    arm_group.set_planning_time(10)

    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

    rospy.loginfo("[INFO] Waiting for gripper action servers...")
    move_client.wait_for_server()
    grasp_client.wait_for_server()
    rospy.loginfo("Gripper connected.")

    if not move_to_ready_pose(arm_group):
        rospy.logerr("Startup: Failed to reach ready pose. Exiting.")
        return

    open_gripper(move_client)

    pick_xy = (0.5, 0.0)   # Pick location
    place_xy = (0.0, -0.5) # Place location

    pick_and_place(arm_group, grasp_client, move_client, pick_xy, place_xy)

    if not move_to_ready_pose(arm_group):
        rospy.logwarn("End: Failed to return to ready pose.")

    rospy.loginfo("Task completed.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

