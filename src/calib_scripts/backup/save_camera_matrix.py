import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo

def save_matrix(msg):
    K = np.array(msg.K).reshape(3, 3)
    np.save("camera_matrix.npy", K)
    print("[✓] Saved to camera_matrix.npy")
    rospy.signal_shutdown("done")

rospy.init_node("save_camera_matrix")
rospy.Subscriber("/camera/color/camera_info", CameraInfo, save_matrix)
rospy.spin()
