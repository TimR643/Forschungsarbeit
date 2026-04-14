import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

def save_depth(msg):
    bridge = CvBridge()
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    np.save("test_depth.npy", depth)
    rospy.signal_shutdown("Saved")

rospy.init_node("depth_saver")
rospy.Subscriber("/camera/depth/image_rect_raw", Image, save_depth)
rospy.spin()
