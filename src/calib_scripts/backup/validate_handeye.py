#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf

# Load calibration matrix
T_cam_to_ee = np.load("T_camera_to_ee.npy")

# Checkerboard config
CHECKERBOARD = (8, 6)
SQUARE_SIZE = 0.025  # meters

K = np.array([[615.0, 0, 320],
              [0, 615.0, 240],
              [0, 0, 1]])
D = np.zeros(5)

# Object points in 3D
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

bridge = CvBridge()
listener = None

def get_robot_pose():
    try:
        now = rospy.Time(0)
        listener.waitForTransform("panda_link0", "panda_link8", now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_link8", now)
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        return T
    except Exception as e:
        print("❌ TF lookup failed:", e)
        return None

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        ret, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
        if ret:
            # Convert to 4x4 camera-to-object transform
            R, _ = cv2.Rodrigues(rvec)
            T_cam_to_obj = np.eye(4)
            T_cam_to_obj[:3, :3] = R
            T_cam_to_obj[:3, 3] = np.squeeze(tvec)

            # Get robot's current EE pose
            T_base_to_ee = get_robot_pose()
            if T_base_to_ee is not None:
                # Chain the transformations
                T_base_to_camera = T_base_to_ee @ np.linalg.inv(T_cam_to_ee)
                T_base_to_obj = T_base_to_camera @ T_cam_to_obj

                pos = T_base_to_obj[:3, 3]
                print("📍 Checkerboard in base frame (m):", np.round(pos, 3))
            else:
                print("⚠️ Skipping frame due to missing robot pose.")

    cv2.imshow("Checkerboard Validation", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        rospy.signal_shutdown("User exit")

def main():
    global listener
    rospy.init_node("validate_handeye")
    listener = tf.TransformListener()
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    print("🟢 Move robot, show checkerboard. Press ESC to quit.")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
