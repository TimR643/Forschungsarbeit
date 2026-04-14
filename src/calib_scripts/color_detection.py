#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorAndDepthDetector:
    def __init__(self):
        rospy.init_node("color_depth_detector_node")

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_image = None

        self.output_dir = "detected_objects"
        os.makedirs(self.output_dir, exist_ok=True)

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        rospy.loginfo("[OBJECT DETECTOR] Node initialized and waiting for images...")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[ERROR] Converting depth image: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image.copy()
            if self.latest_depth is not None:
                self.process_image(cv_image, self.latest_depth)
        except Exception as e:
            rospy.logerr(f"[ERROR] Converting color image: {e}")

    def process_image(self, image, depth_image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Red mask
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Step 2: Morphological operations to clean the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        timestamp = rospy.Time.now().to_sec()

        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < 300:
                continue

            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Step 1: Use median depth over 5x5 window
            kernel_size = 5
            half_k = kernel_size // 2

            if (cy - half_k >= 0 and cy + half_k < depth_image.shape[0] and
                cx - half_k >= 0 and cx + half_k < depth_image.shape[1]):

                depth_patch = depth_image[cy - half_k:cy + half_k + 1, cx - half_k:cx + half_k + 1]
                valid_depths = depth_patch[depth_patch > 0]

                if valid_depths.size > 0:
                    depth_value = float(np.median(valid_depths)) * 0.001
                else:
                    depth_value = float(depth_image[cy, cx]) * 0.001
            else:
                depth_value = float(depth_image[cy, cx]) * 0.001

            # Draw centroid for visual debugging
            cv2.circle(image, (cx, cy), 5, (255, 255, 0), -1)

            obj_data = {
                "color": "red",
                "shape": "unknown",
                "pixel": [cx, cy],
                "depth": depth_value,
                "timestamp": timestamp
            }

            filename = f"red_object_{int(timestamp)}.json"
            path = os.path.join(self.output_dir, filename)
            with open(path, "w") as f:
                json.dump(obj_data, f, indent=2)
            rospy.loginfo(f"📂 Saved red object to {path}")
            break  # Only one object per frame

        cv2.imshow("Camera", image)
        cv2.imshow("Red Mask", mask)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ColorAndDepthDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

