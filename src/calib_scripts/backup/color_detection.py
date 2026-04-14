#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RedObjectDetector:
    def __init__(self):
        rospy.init_node("red_object_detector")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        print("🟥 Looking for red objects... Press ESC to exit.")
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range (lower and upper bounds)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Ignore small noise
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), (0, 0, 255), 2)
                cv2.putText(frame, f"Red Object", (center[0]-40, center[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                print(f"🟥 Detected red object at pixel: {center}")

        # Show the image
        cv2.imshow("Red Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            rospy.signal_shutdown("User exited")

if __name__ == "__main__":
    RedObjectDetector()
    cv2.destroyAllWindows()
