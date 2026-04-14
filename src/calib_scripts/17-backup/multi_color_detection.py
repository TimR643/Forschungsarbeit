#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class ColorDetector:
    def __init__(self):
        rospy.init_node("color_detector_node")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.latest_image = None
        self.output_dir = "detected_objects"
        os.makedirs(self.output_dir, exist_ok=True)
        rospy.loginfo("[OBJECT DETECTOR] Node initialized and waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image.copy()
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"[ERROR] Converting image: {e}")

    def process_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        cv2.imshow("Camera View", image)
        cv2.imshow("HSV View", hsv)
        cv2.setMouseCallback("HSV View", self.click_event, hsv)

        # Only red detection
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        cv2.imshow("Red Mask", mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        timestamp = rospy.Time.now().to_sec()

        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < 300:
                continue  # ignore small objects

            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Save detection
            obj_data = {
                "color": "red",
                "shape": "unknown",
                "pixel": [cx, cy],
                "timestamp": timestamp
            }

            filename = f"red_object_{int(timestamp)}.json"
            path = os.path.join(self.output_dir, filename)
            with open(path, "w") as f:
                json.dump(obj_data, f, indent=2)
            rospy.loginfo(f"💾 Saved red object to {path}")
            break  # only one object per frame

        cv2.waitKey(1)

    def click_event(self, event, x, y, flags, hsv_img):
        if event == cv2.EVENT_LBUTTONDOWN:
            h, s, v = hsv_img[y, x]
            print(f"[HSV DEBUG] Clicked at ({x},{y}) -> H={h}, S={s}, V={v}")

if __name__ == "__main__":
    try:
        ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

