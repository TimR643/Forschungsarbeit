#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rospy
import numpy as np
import os
from image_geometry import PinholeCameraModel
import datetime


class take_picture:
    def __init__(self):
        self.bridge             = CvBridge()
        self.image_sub          = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.latest_image       = None
        self.color_camera_model = PinholeCameraModel()

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV bridge Error (RGB): %s", e)


    def collect_images(self):
        now         = datetime.datetime.now()
        timestamp   = now.strftime("%Y-%m-%d_%H-%M-%S")  # z.B. 2025-07-02_14-30-59
        filename    = f"{timestamp}_stein.png"
        script_dir  = os.path.dirname(os.path.abspath(__file__))
        images_dir  = os.path.join(script_dir, "../images")
        image_path  = os.path.join(images_dir, filename)

        if self.latest_image is not None:
            cv2.imwrite(image_path, self.latest_image)
            rospy.loginfo(f"Raw_Picture saved: {image_path}")
        else:
            rospy.logwarn("No image to save!")

        return self.latest_image
    
    def collect_images_on_keypress(self):
        rospy.loginfo("Drücke Enter, um ein Bild zu speichern, Ctrl+C zum Beenden.")
        try:
            while not rospy.is_shutdown():
                input_str = input(">>> ")
                if input_str == "":
                    self.collect_images()
                else:
                    rospy.loginfo("Nur Enter drücken, um Bilder zu speichern.")
        except KeyboardInterrupt:
            rospy.loginfo("Bildersammlung beendet.")

if __name__ == "__main__":
    rospy.init_node("Take_Picture_Node", anonymous=True)
    pic = take_picture()
    pic.collect_images_on_keypress()



