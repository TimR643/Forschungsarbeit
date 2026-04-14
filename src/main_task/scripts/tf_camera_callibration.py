#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import math
from tf.transformations import quaternion_matrix, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import geometry_msgs.msg
import random
import moveit_commander
import tf.transformations as tft  # quaternion_from_matrix


class hand_eye_calibration():
    def __init__(self):
        self.checkerboard   = (6,8)         # Anzahl innerer Ecken (Breite x Höhe)
        self.square_size    = 0.025         # Quadratgröße in Metern

        self.bridge         = CvBridge()
        self.arm            = moveit_commander.MoveGroupCommander("panda_arm")

        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Listen zum Speichern
        self.objpoints      = []
        self.imgpoints      = []
        self.robot_poses    = []
        self.img_count      = 0
        self.latest_image   = None
        self.image_received = False


    def get_robot_pose(self):
        pose = self.arm.get_current_pose(end_effector_link='panda_hand').pose
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T = quaternion_matrix(q)
        T[0:3,3] = t
        return T

    def image_callback(self, msg):
        try:
            self.latest_image   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            rospy.logerr(f"Fehler beim Konvertieren des Bildes: {e}")


    def amove_to_position(self, position=(0.5, 0.5, 0.5), orientation_deg=(180, 0, -45)):
        rospy.loginfo(f"Bewege Arm zu Position: {position} mit Orientierung (°): {orientation_deg}")

        if (position[2] + 0.11) < 0.108:
            rospy.logwarn("Kollision mit Tisch! Bewegung abgebrochen!")
            return False

        roll    = math.radians(orientation_deg[0])
        pitch   = math.radians(orientation_deg[1])
        yaw     = math.radians(orientation_deg[2])

        q = quaternion_from_euler(roll, pitch, yaw)

        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = position[0]
        pose_target.position.y = position[1]
        pose_target.position.z = position[2] + 0.11  # Höhe korrigiert
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        #self.arm.set_start_state_to_current_state()

        self.arm.set_pose_target(pose_target)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        if success:
            rospy.loginfo("Bewegung erfolgreich")
        else:
            rospy.logwarn("Bewegung fehlgeschlagen")
        return success
    
    def move_to_position(self, position=(0.5, 0.5, 0.5), orientation_deg=(180, 0, -45), max_retries=3):
        rospy.loginfo(f"Bewege Arm zu Position: {position} mit Orientierung (°): {orientation_deg}")

        if (position[2] + 0.11) < 0.108:
            rospy.logwarn("Kollision mit Tisch! Bewegung abgebrochen!")
            return False

        roll = math.radians(orientation_deg[0])
        pitch = math.radians(orientation_deg[1])
        yaw = math.radians(orientation_deg[2])
        q = quaternion_from_euler(roll, pitch, yaw)

        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = position[0]
        pose_target.position.y = position[1]
        pose_target.position.z = position[2] + 0.11  # Höhe korrigiert
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        # Toleranzen und Zeitlimits anpassen
        self.arm.set_planning_time(20.0)
        self.arm.set_num_planning_attempts(20)
        self.arm.set_max_velocity_scaling_factor(1)      # Geschwindigkeit halbieren
        self.arm.set_max_acceleration_scaling_factor(1)  # Beschleunigung halbieren
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.allow_replanning(True)

        for attempt in range(max_retries):
            rospy.loginfo(f"Versuch {attempt+1} von {max_retries}")
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(pose_target)
            
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()

            if success:
                rospy.loginfo("Bewegung erfolgreich")
                return True
            else:
                rospy.logwarn("Bewegung fehlgeschlagen – versuche erneut...")

        rospy.logerr("Alle Bewegungsversuche fehlgeschlagen.")
        return False


    def amove_and_save_checkerboard_data(self, start_pos=(0.45, 0.0, 0.50), rows=5, cols=5, spacing_x=0.1, spacing_y=0.15, orientation_deg=(180, 0, -45)):
        for row in range(rows):
            for col in range(cols):
                height_variation = 0.05  # +/- 5cm Variation
                x = start_pos[0] + col * spacing_x
                y = start_pos[1] + row * spacing_y
                z = start_pos[2] + random.uniform(-height_variation, height_variation)    

                rospy.loginfo(f"Bewege zu Checkerboard-Punkt ({row}, {col}) → ({x:.3f}, {y:.3f}, {z:.3f})")
                orientation_deg = (random.uniform(179, 181),random.uniform(-1, 1),random.uniform(-45, 45))
                if not self.move_to_position(position=(x, y, z), orientation_deg=orientation_deg):
                    rospy.logwarn("Bewegung nicht möglich, überspringe Position.")
                    continue

                rospy.sleep(2.0)  # Warten, bis Arm stabil ist
                self.save_picture_and_pose()

        #self.move_to_position(position=(0.25, -0.1, 0.4), orientation_deg=(200, 20, -45))
        #self.save_picture_and_pose()
        #self.move_to_position(position=(0.65, 0.1, 0.4), orientation_deg=(200, 20, 30))
        #self.save_picture_and_pose()


    def move_and_save_checkerboard_data(self, cal_object_pos=(0.45, 0.0, 0.0), rows=6, cols=6, spacing_x=0.02, spacing_y=0.03, z_min=0.50, z_max=0.7):
        # Startposition so berechnen, dass das Gitter mittig zum Objekt liegt
        start_x = cal_object_pos[0] - ((cols - 1) / 2) * spacing_x
        start_y = cal_object_pos[1] - ((rows - 1) / 2) * spacing_y

        for row in range(rows):
            for col in range(cols):
                x = start_x + col * spacing_x
                y = start_y + row * spacing_y
                z = random.uniform(z_min, z_max)
                position = (x, y, z)

                orientation_deg = (random.uniform(170, 190),random.uniform(-10, 10),random.uniform(-45, 0))

                rospy.loginfo(f"Fahre zu Punkt ({row}, {col}): Pos={position}, Ori={orientation_deg}")

                success = self.move_to_position(position=position, orientation_deg=orientation_deg)
                if not success:
                    rospy.logwarn(f"Bewegung zu Punkt {position} fehlgeschlagen.")
                rospy.sleep(0.25)
                self.save_picture_and_pose()

    def wait_for_data(self):
        #self.image_received = False
        rate = rospy.Rate(10)
        while (self.latest_image is None or not self.image_received) and not rospy.is_shutdown():
            rospy.loginfo("Warte auf Bild- und Kameradaten ...")
            rate.sleep()
        return True


    def save_picture_and_pose(self):
        if not self.wait_for_data():
            rospy.logwarn("Kein Bild empfangen, überspringe Position.")
            return

        if not self.detect_and_save_checkerboard():
            rospy.logwarn("Checkerboard nicht erkannt, überspringe Bildaufnahme.")
            return
        
        self.save_robot_pose()
        self.img_count += 1
    

    def detect_and_save_checkerboard(self):
        frame = self.latest_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_cb, corners = cv2.findChessboardCorners(gray, self.checkerboard, None)

        if not ret_cb:
            return False

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        # Ecken zeichnen
        cv2.drawChessboardCorners(frame, self.checkerboard, corners2, ret_cb)




        # Bild skalieren (z. B. auf 50 % der Originalgröße)
        scale_percent = 50  # Prozentwert der Originalgröße
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)

        # Bild verkleinern
        resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        # Verkleinertes Bild anzeigen
        cv2.imshow("Checkerboard Detection", resized_frame)
        cv2.waitKey(500)  # 500 ms warten, damit das Fenster sichtbar ist



        # Checkerboard-Objektpunkte erzeugen
        objp = np.zeros((self.checkerboard[0] * self.checkerboard[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)
        objp *= self.square_size

        # Daten Liste hinzufügen
        self.imgpoints.append(corners2)
        self.objpoints.append(objp)

        # Bild speichern
        image_save_path = "main_task/checkerboard_images"
        os.makedirs(image_save_path, exist_ok=True)
        img_name = os.path.join(image_save_path, f"image_{self.img_count:02d}.png")
        cv2.imwrite(img_name, frame)
        rospy.loginfo(f"Bild {img_name} gespeichert")

        return True


    def save_robot_pose(self):
        T = self.get_robot_pose()
        self.robot_poses.append(T)
        rospy.loginfo(f"Roboterpose #{self.img_count} gespeichert:\n{T}")


    def calibrate_camera(self, objpoints, imgpoints, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        rospy.loginfo(f"Kameramatrix:\n{mtx}")
        rospy.loginfo(f"Verzerrung:\n{dist.ravel()}")
        return rvecs, tvecs


    def compute_camera_poses(self, rvecs, tvecs):
        poses = []
        for rvec, tvec in zip(rvecs, tvecs):
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            poses.append(T)
        return poses


    def decompose_pose(self, T):
        return T[:3, :3], T[:3, 3]


    def prepare_hand_eye_inputs(self, robot_poses, camera_poses):
        R_gripper2base, t_gripper2base = [], []
        R_target2cam, t_target2cam = [], []
        for r_pose, c_pose in zip(robot_poses, camera_poses):
            Rg, tg = self.decompose_pose(r_pose)
            Rt, tt = self.decompose_pose(c_pose)
            R_gripper2base.append(Rg)
            t_gripper2base.append(tg)
            R_target2cam.append(Rt)
            t_target2cam.append(tt)
        return R_gripper2base, t_gripper2base, R_target2cam, t_target2cam


    def calibrate_hand_eye(self, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI):
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=method
        )
        T = np.eye(4)
        T[:3, :3] = R_cam2gripper
        T[:3, 3] = t_cam2gripper.reshape(3)
        return T


    def handeye_tf_calculation(self):
        cv2.destroyAllWindows()
        if len(self.robot_poses) < 5:
            rospy.logerr("Nicht genug Daten für Kalibrierung (mind. 5 Bilder/Paarungen benötigt)")
            return False
        rvecs, tvecs    = self.calibrate_camera(self.objpoints, self.imgpoints, self.latest_image)
        camera_poses    = self.compute_camera_poses(rvecs, tvecs)
        Rg, tg, Rt, tt  = self.prepare_hand_eye_inputs(self.robot_poses, camera_poses)
        methods = {
            "Tsai": cv2.CALIB_HAND_EYE_TSAI,
            "Park": cv2.CALIB_HAND_EYE_PARK,
            "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
            "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS
        }

        for name, method in methods.items():
            T = hand_eye.calibrate_hand_eye(Rg, tg, Rt, tt, method=method)
            print(f"{name}:\n{T}\n")

        T_cam2gripper   = self.calibrate_hand_eye(Rg, tg, Rt, tt)
        rospy.loginfo(f"Hand → Camera Transformationsmatrix (4x4):\n{T_cam2gripper}")
        return T_cam2gripper
    ########################################################################
 

    def quat_to_euler_deg(self, quat):
        euler_rad = tft.euler_from_quaternion(quat)
        euler_deg = tuple(math.degrees(angle) for angle in euler_rad)
        return euler_deg
    

    def move_to_multiple_points_pointing_to_object(self, num_points, 
                                                start_pos=(0.3, 0.3, 0.4), 
                                                object_pos=(0.3, 0.3, 0.0),
                                                x_range=0.1, y_range=0.1, z_range=(0.21, 0.5)):
        for i in range(num_points):
            x = random.uniform(start_pos[0] - x_range/2, start_pos[0] + x_range/2)
            y = random.uniform(start_pos[1] - y_range/2, start_pos[1] + y_range/2)
            z = random.uniform(z_range[0], z_range[1])
            
            position        = (x, y, z)
            direction       = np.array(object_pos) - np.array(position)
            direction_norm  = direction / np.linalg.norm(direction)
            
            yaw   = math.degrees(math.atan2(direction_norm[1], direction_norm[0]))
            pitch = math.degrees(math.atan2(-direction_norm[2], 
                                            math.sqrt(direction_norm[0]**2 + direction_norm[1]**2)))
            roll  = 180.0  # konstante Roll-Orientierung
            
            rospy.loginfo(f"Punkt {i+1}: {position}, Orientierung (Roll, Pitch, Yaw): ({roll:.1f}, {pitch:.1f}, {yaw:.1f})")
            
            success = self.move_to_position(position=position, orientation_deg=(roll, pitch, yaw))
            if not success:
                rospy.logwarn(f"Bewegung zu Punkt {position} fehlgeschlagen.")


    def move_to_points_orthogonal_to_object(self, cal_object_pos=(0.3, 0.3, 0.0), rows=3, cols=3, spacing_x=0.05, spacing_y=0.05, z_min=0.30, z_max=0.55):
        # Startposition so berechnen, dass das Gitter mittig zum Objekt liegt
        start_x = cal_object_pos[0] - ((cols - 1) / 2) * spacing_x
        start_y = cal_object_pos[1] - ((rows - 1) / 2) * spacing_y

        for row in range(rows):
            for col in range(cols):
                x = start_x + col * spacing_x
                y = start_y + row * spacing_y
                z = random.uniform(z_min, z_max)
                position = (x, y, z)

                quat = self.look_at_orientation(position, cal_object_pos, roll=0)
                orientation_deg = self.quat_to_euler_deg(quat)

                rospy.loginfo(f"Fahre zu Punkt ({row}, {col}): Pos={position}, Ori={orientation_deg}")

                success = self.move_to_position(position=position, orientation_deg=orientation_deg)
                if not success:
                    rospy.logwarn(f"Bewegung zu Punkt {position} fehlgeschlagen.")
                rospy.sleep(1.5)
                self.save_picture_and_pose()


    def move_in_arc_around_object(self, cal_object_pos, radius=0.20, z=0.4, num_poses=3):
        for theta in np.linspace(0, 2*np.pi, num_poses, endpoint=False):
            x = cal_object_pos[0] + radius * math.cos(theta)
            y = cal_object_pos[1] + radius * math.sin(theta)
            position = (x, y, z)
            quat = self.look_at_orientation(position, cal_object_pos, roll=0)
            orientation_deg = self.quat_to_euler_deg(quat)
            self.move_to_position(position, orientation_deg)
            rospy.sleep(1.5)
            self.save_picture_and_pose()


    def look_at_orientation(self, position, target, roll=0):
        forward = np.array(target) - np.array(position)
        forward /= np.linalg.norm(forward)

        up = np.array([0, 0, 1])
        if np.allclose(forward, up) or np.allclose(forward, -up):
            up = np.array([0, 1, 0])

        right = np.cross(up, forward)
        right /= np.linalg.norm(right)
        true_up = np.cross(forward, right)

        rot_matrix = np.eye(4)
        rot_matrix[0:3, 0] = right
        rot_matrix[0:3, 1] = true_up
        rot_matrix[0:3, 2] = forward

        # Roll direkt übernehmen (ohne Begrenzung)
        roll_rad = math.radians(roll)
        roll_rot = tft.rotation_matrix(roll_rad, forward)
        rot_matrix = np.dot(rot_matrix, roll_rot)

        quat = tft.quaternion_from_matrix(rot_matrix)

        return quat






if __name__ == "__main__":
    rospy.init_node('handeye_calib_node', anonymous=True)
    rospy.loginfo("Starte automatisierte Hand-Eye Kalibrierung...")

    # Kurze Wartezeit zum Initialisieren
    rospy.sleep(2.0)
    hand_eye = hand_eye_calibration()
    hand_eye.move_and_save_checkerboard_data()
    #hand_eye.handeye_tf_calculation()

    #hand_eye.move_in_arc_around_object(cal_object_pos=(0.45, 0.0, 0.0))
    #hand_eye.move_to_points_orthogonal_to_object(cal_object_pos=(0.45, 0.0, 0.0),rows=5,cols=5,spacing_x=0.05,spacing_y=0.05,z_min=0.4,z_max=0.55)
    #hand_eye.move_in_arc_around_object(cal_object_pos=(0.45, 0.0, 0.0))
    hand_eye.handeye_tf_calculation()
