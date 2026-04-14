import numpy as np
import cv2
import json

def load_sample(path):
    with open(path, 'r') as f:
        return json.load(f)

def rvec_tvec_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(np.array(rvec))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).flatten()
    return T

# 🚨 Change this to pick your sample
sample = load_sample("sample_01.json")

# Load your calibration result
T_camera2ee = np.load("T_camera_to_ee.npy")

# Build transforms
T_checkerboard2cam = rvec_tvec_to_matrix(sample['rvec'], sample['tvec'])
T_gripper2base = np.array(sample['robot_pose'])

# Chain the transforms:
T_obj2base = T_gripper2base @ T_camera2ee @ T_checkerboard2cam

# Final position of checkerboard in robot base frame:
position_in_base = T_obj2base[:3, 3]

print("\n📍 Estimated Checkerboard Position (robot base frame):")
print("X (meters):", position_in_base[0])
print("Y (meters):", position_in_base[1])
print("Z (meters):", position_in_base[2])
