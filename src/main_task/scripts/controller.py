#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, HomingAction, HomingGoal, MoveAction, MoveGoal
from tf.transformations import quaternion_from_euler
import math
from tf.transformations import euler_from_quaternion
from moveit_msgs.msg import OrientationConstraint, Constraints


class controller:
    def __init__(self):
        self.robot   = moveit_commander.RobotCommander()
        self.scene   = moveit_commander.PlanningSceneInterface()
        self.arm     = moveit_commander.MoveGroupCommander("panda_arm")
        self.gripper = moveit_commander.MoveGroupCommander("panda_hand")


    def open_gripper(self, width=0.05, speed=0.05, timeout=10.0):
        rospy.loginfo("Start Gripper opening...")
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()

        goal = MoveGoal()
        goal.width = width  
        goal.speed = speed 
        client.send_goal(goal)

        if not client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn("Open Gripper Failed")
            client.cancel_goal()
            return False
        else:
            result = client.get_result()
            rospy.loginfo("Open Gripper success: %s", result.success)
            return result.success


    def close_gripper(self, width=0.025, speed=0.05, force=40.0, timeout=10.0):
        rospy.loginfo("Start Gripper closing...")
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        client.wait_for_server()

        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.01
        goal.epsilon.outer = 0.01

        rospy.loginfo("Sende Greif-Befehl...")
        client.send_goal(goal)

        if not client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn("Close Gripper Failed")
            client.cancel_goal()
            
            return False
        else:
            result = client.get_result()
            rospy.loginfo("Close Gripper success: %s", result.success)
            if result.success == False:
                #self.close_gripper()   # geht nur wenn auch tatsächlich objekt vorhanden zum testen auskommentiert
                pass
            return result.success


    def init_gripper(self, timeout=10.0):
        rospy.loginfo("Start Homing...")
        client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
        client.wait_for_server()

        goal = HomingGoal()
        client.send_goal(goal)

        if not client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn("Init Gripper Failed")
            client.cancel_goal()
            return False
        else:
            result = client.get_result()
            rospy.loginfo("Init Gripper success: %s", result.success)
            return result.success


    def move_to_position(self, position=(0.5, 0.5, 0.5), orientation_deg=(180, 0, -45)):
        rospy.loginfo("Move Arm to Position: %s with Orientation (°): %s", position, orientation_deg)

        if (position[2] + 0.107) < 0.108:
            rospy.logwarn("Kollision mit Tisch! Wert automatisch auf Tischebene gesetzt")
            position[2] = 0.0
            #return False

        # Orientierung in Grad → Radiant umrechnen
        roll    = math.radians(orientation_deg[0])  # um x-Achse
        pitch   = math.radians(orientation_deg[1])  # um y-Achse
        yaw     = math.radians(orientation_deg[2])  # um z-achse

        # Quaternion berechnen
        q = quaternion_from_euler(roll, pitch, yaw)

        pose_target                 = geometry_msgs.msg.Pose()
        pose_target.position.x      = position[0]
        pose_target.position.y      = position[1]
        pose_target.position.z      = position[2] + 0.107 #höhe korrigiert
        pose_target.orientation.x   = q[0]
        pose_target.orientation.y   = q[1]
        pose_target.orientation.z   = q[2]
        pose_target.orientation.w   = q[3]





        # Orientierungs-Constraint setzen
        #orientation_constraint = OrientationConstraint()
        #orientation_constraint.link_name = self.arm.get_end_effector_link()
        #orientation_constraint.header.frame_id = self.arm.get_planning_frame()
        #orientation_constraint.orientation = pose_target.orientation
        #orientation_constraint.absolute_x_axis_tolerance = 0.1   # Roll-Toleranz
        #orientation_constraint.absolute_y_axis_tolerance = 0.2   # Pitch-Toleranz
        #orientation_constraint.absolute_z_axis_tolerance = math.pi   # Yaw etwas lockerer
        #orientation_constraint.weight = 1.0

        #constraints = Constraints()
        #constraints.orientation_constraints.append(orientation_constraint)
        #self.arm.set_path_constraints(constraints)






        # Planungsparameter
        self.arm.set_planning_time(20.0)
        self.arm.set_num_planning_attempts(30)
        self.arm.set_max_velocity_scaling_factor(0.7)      # Geschwindigkeit 
        self.arm.set_max_acceleration_scaling_factor(0.5)  # Beschleunigung 
        self.arm.set_goal_position_tolerance(0.0005)
        self.arm.set_goal_orientation_tolerance(0.005)
        self.arm.allow_replanning(True)

        success = False
        attempt = 0

        while not success and attempt < 10:
            attempt += 1
            rospy.loginfo("Versuch %d: Bewege Arm zu Zielposition...", attempt)

            self.arm.set_pose_target(pose_target)
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()

            if not success:
                rospy.logwarn("Bewegung fehlgeschlagen (Versuch %d)", attempt)



        # Constraints wieder löschen
        #self.arm.clear_path_constraints()




        if success:
            rospy.loginfo("Bewegung erfolgreich nach %d Versuch(en)", attempt)
        else:
            rospy.logerr("Bewegung gescheitert nach %d Versuch(en)", attempt)
            return False
        
        return success


    def get_current_pose(self):
        pose = self.arm.get_current_pose().pose  # PoseStamped → Pose
        rospy.loginfo("Aktuelle Pose:\nPosition: (%.3f, %.3f, %.3f)\nOrientation (quaternion): (%.3f, %.3f, %.3f, %.3f)",
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return pose


    def get_current_pose_grad(self):
        pose = self.arm.get_current_pose().pose  # PoseStamped → Pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        roll_deg  = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg   = math.degrees(yaw)
        pose.orientation.x = roll_deg
        pose.orientation.y = pitch_deg
        pose.orientation.z = yaw_deg
        rospy.loginfo("Current Pose:\n Position: (%.3f, %.3f, %.3f)\n Orientation (quaternion): (%.3f, %.3f, %.3f, %.3f)\n Orientation (rpy in °): Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°",
                      pose.position.x, pose.position.y, pose.position.z,
                      q[0], q[1], q[2], q[3],
                      roll_deg, pitch_deg, yaw_deg)
        return pose


    def home_position(self):
        home_coords     = (0.4, 0.4, 0.25)      # Beispiel
        home_rotation   = (180, 0, -45)
        self.move_to_position(home_coords, home_rotation)


    def detection_position(self):
        detection_coords    = (0.5, -0.10, 0.45)  # Beispiel
        detection_rotation  = (180, 0, -45)
        self.move_to_position(detection_coords, detection_rotation)


    def camera_position_1x1(self):
        detection_coords    = (0.0, 0.4, 0.45)  
        detection_rotation  = (180, 0, -45)
        self.move_to_position(detection_coords, detection_rotation)


    def move_small_step(self):
        pose = self.get_current_pose_grad()
        position = (pose.position.x + 0.01, pose.position.y +0.01, pose.position.z - 0.1)
        rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z)
        self.move_to_position(position, rotation)

    def object_to_trash(self):
        position    = (0.3, -0.3, 0)
        rotation    = (180, 0, -45)
        above_object    = (position[0], position[1], position[2] + 0.1)    # 10cm above the object
        above_object_2  = (position[0], position[1], position[2] + 0.2)    # 10cm above the object
        
        self.home_position()
        self.move_to_position(above_object_2, rotation)                # move above the target box
        self.move_to_position(above_object, rotation)                  # move above the target box
        self.open_gripper()                                                                 # Open the grasp function
        self.move_to_position(above_object_2, rotation)                # move in safety height

        self.home_position()


    def pick_object(self,object_base_coords,object_base_rotation):
        above_object    = (object_base_coords[0], object_base_coords[1], object_base_coords[2] + 0.1)          # 10cm above the object
        above_object_2  = (object_base_coords[0], object_base_coords[1], object_base_coords[2] + 0.2)          # 10cm above the object

        self.move_to_position(above_object_2, object_base_rotation)       # move a few cm above the object
        self.open_gripper()                                               # open the grasp function
        self.move_to_position(above_object, object_base_rotation)         # move a few cm above the object
        self.move_to_position(object_base_coords, object_base_rotation)   # move down to the object
        if self.close_gripper() == False:
            self.move_to_position(above_object_2, object_base_rotation)       # move in safety height
            return False                                       # close the grasp function
        self.move_to_position(above_object_2, object_base_rotation)       # move in safety height
        return True


    def place_object(self,object_target_positon_coords, object_target_rotation_coords):
        above_object    = (object_target_positon_coords[0], object_target_positon_coords[1], object_target_positon_coords[2] + 0.1)    # 10cm above the object
        above_object_2  = (object_target_positon_coords[0], object_target_positon_coords[1], object_target_positon_coords[2] + 0.1)    # 10cm above the object
        on_object       = (object_target_positon_coords[0], object_target_positon_coords[1], object_target_positon_coords[2] + 0.005)
        
        self.home_position()
        self.move_to_position(above_object_2, object_target_rotation_coords)                # move above the target box
        self.move_to_position(above_object, object_target_rotation_coords)                  # move above the target box
        self.move_to_position(on_object, object_target_rotation_coords)                     # Move to target box
        self.open_gripper()                                                                 # Open the grasp function
        self.move_to_position(above_object_2, object_target_rotation_coords)                # move in safety height

        self.home_position()


if __name__ == "__main__":
    rospy.init_node("controller_test_node", anonymous=True)
    controller = controller()

    rospy.loginfo("Starte Test Steuerung ")
    #controller.init_gripper()
    #controller.close_gripper()
    #controller.open_gripper()
    #controller.move_to_position(position=(0.4829446257119817, -0.1818618076980798, 0.0), orientation_deg=(180, 0, -45))
    #controller.close_gripper()
    #controller.home_position()
    #controller.get_current_pose_grad()
    #controller.detection_position()
    #controller.get_current_pose_grad()
    controller.camera_position_1x1()
    #controller.move_small_step()
    #controller.object_to_trash()