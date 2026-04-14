#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R


class tf_manager():
    def __init__(self):
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)

    
    def get_tf_hand_to_camera(self):
        # Diese Funktion erstellt die statische Transformation von Endeffektor zur Kamera
        translation = np.array([0.08, 0.0, -0.02])  # in Metern
        rotation    = [0.0, np.pi, 0.0]             # roll, pitch, yaw in Radiant
        rotation    = [-0.000796, 0.0, -1.5708]     # roll, pitch, yaw in Radiant

        # Erstelle Rotationsmatrix #Euler Winkel
        rot_matrix = R.from_euler('xyz', rotation).as_matrix()

        # Gegebene Daten
        translation = np.array([0.06, 0.02, -0.02])
        quaternion  = [0.0002815440309169026, -0.0002815440309167916, 0.7071067251362828, -0.7071067251362828]

        # Rotation berechnen # quaternion Winkel
        rot_matrix = R.from_quat(quaternion).as_matrix()


        # Homogene 4x4-Transformationsmatrix
        T = np.eye(4)
        T[0:3, 0:3] = rot_matrix
        T[0:3, 3]   = translation
        # -------- ab hier relevant für echten Roboter ------------

        # Matrix automatisch erstellt
        T = np.array([[ 0.02909049, -0.9986162,  -0.04381128,  0.05255668],
                      [ 0.99944899,  0.02975965, -0.01469955, -0.0410961 ],
                      [ 0.01598302, -0.04335952,  0.99893168,  0.08306703],
                      [ 0.0,         0.0,         0.0,         1.0       ]])
        T = np.array([[ 0.07877075, -0.99498305, -0.06167573,  0.06713169],
                      [ 0.99688108,  0.07831954,  0.00970326, -0.0443922 ],
                      [-0.00482416, -0.0622477,   0.99804907, -0.08835455],
                      [ 0. ,         0.  ,        0. ,         1.        ]])
        T = np.array([[ 0.03315248, -0.99793308, -0.05504979,  0.05893811],
                    [ 0.99940224,  0.03364073, -0.00796613 ,-0.03426252],
                    [ 0.00980158, -0.05475279,  0.99845183 ,-0.11071556],
                    [ 0.    ,      0.      ,    0.     ,     1.        ]])
        T = np.array([[ 0.13668052, -0.98941507 ,-0.04874692 , 0.04824856],
                    [ 0.99033018,  0.13765597, -0.01723295 ,-0.04502607],
                    [ 0.02376085, -0.04592013 , 0.99866249 , 0.14306657],
                    [ 0.      ,    0.  ,        0.  ,        1.        ]])
        T = np.array([[ 0.09848305, -0.99456245 ,-0.03386179  ,0.04660335],
                    [ 0.99503417,  0.09890892, -0.01113633 ,-0.03589937],
                    [ 0.01442501, -0.03259689,  0.99936448 , 0.02800913],
                    [ 0.    ,      0.    ,      0.      ,    1.        ]])
        
        T = np.array([[ 0.01488588, -0.99893591, -0.04365156 , 0.05737771],
                    [ 0.99966013,  0.0158027 , -0.02073398, -0.03875958],
                    [ 0.02140173 ,-0.04332808 , 0.99883164 , 0.0769408 ],
                    [ 0.    ,      0.      ,    0.      ,    1.        ]])
        
        T = np.array([[ 0.02242771, -0.99849656, -0.05001619 , 0.05423452],
                    [ 0.99943283 , 0.0236496,  -0.02397324, -0.03914838],
                    [ 0.02512006 ,-0.04945015 , 0.99846065 , 0.08899464],
                    [ 0.    ,      0.   ,       0.      ,    1.        ]])
        print("Hand → Camera Transformationsmatrix (4x4):\n", T)
        return T
    
    def get_tf_base_to_hand(self):
        # ermittelt die Transformationsmatrix von der Base zur Robterhand
        # Hole live die Pose des Endeffektors relativ zur Roboterbasis (== world)
        tf_base_to_endeffektor  = self.tf_buffer.lookup_transform("panda_link0", "panda_hand", rospy.Time(0), rospy.Duration(1.0))
        t                       = tf_base_to_endeffektor.transform.translation
        r                       = tf_base_to_endeffektor.transform.rotation
        translation             = np.array([t.x, t.y, t.z])
        rotation                = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T = np.eye(4)
        T[0:3, 0:3] = rotation
        T[0:3, 3]   = translation
        print("World → Hand Transformationsmatrix (4x4):\n",T)
        return T


    def get_tf_base_to_camera(self):
        T_hand_to_camera    = self.get_tf_hand_to_camera()      # Transformation: Hand ↔ Camera
        T_base_to_hand      = self.get_tf_base_to_hand()        # Transformation: Base ↔ Hand
        T_base_to_camera    = T_base_to_hand @ T_hand_to_camera # Matrix-Multiplikation: Welt → Kamera
        print("World → Camera Transformationsmatrix (4x4):\n", np.round(T_base_to_camera, 4))
        return T_base_to_camera
    

    def tf_image_to_base(self, object_camera_coords=(0.0, 0.0, 0.5)):
        # Transformiert Objektkoordinaten aus Kameraraum in die Roboterbasis (bzw. Welt)
        image_coords            = np.array(object_camera_coords + (1.0,))   # tuple + (1.0,) ergibt 4D-Vektor
        base_to_camera          = self.get_tf_base_to_camera()              # aktuelle Pose der Kamera zur Basis
        #camera_to_base         = np.linalg.inv(base_to_camera)             # inverse berechnen
        object_base_coords      = tuple(base_to_camera @ image_coords )             # Nur x, y, z zurückgeben

        # Hier wird der z-Wert immer auf 0 gesetzt, egal was Kamera erkannt/berechnet hat
        object_base_coords = list(object_base_coords)
        object_base_coords[2]   = 0.0
        rospy.loginfo(f"Objekt in Camera coords: {image_coords}")
        rospy.loginfo(f"Objekt in Base   coords: {object_base_coords}")
        return object_base_coords


if __name__ == "__main__":
    rospy.init_node("tf_test_node", anonymous=True)
    rospy.sleep(1.0)  
    a = tf_manager()

    rospy.loginfo("Starte Test tf_claculation:")

    # Testweise berechnen von Koordinaten aus vorhandenen Kamerainformationen
    obj_coords = (0.23478648283459225, -0.05409398514807756, 0.6673333333333333)  # Beispielkoordinaten in Kamera-Frame


    obj_position = a.tf_image_to_base(obj_coords)
