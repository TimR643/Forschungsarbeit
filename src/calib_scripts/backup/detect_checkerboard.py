import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Checkerboard parameters
CHECKERBOARD = (8, 6)
SQUARE_SIZE = 0.025  # in meters

bridge = CvBridge()

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        print("Could not convert image:", e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, found)
        print("✅ Checkerboard detected.")
    else:
        print("❌ Checkerboard NOT found.")

    cv2.imshow("Checkerboard Detection", frame)
    cv2.waitKey(1)

def main():
    rospy.init_node('checkerboard_detector')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    print("🟢 Subscribed to /camera/color/image_raw")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
