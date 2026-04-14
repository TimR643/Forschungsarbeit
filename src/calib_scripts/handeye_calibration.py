#!/usr/bin/env python3
import os
import json
import numpy as np
import cv2
import glob

# === PATHS ===
npz_folder = "/home/roslab/catkin_ws/src/calib_scripts/samples/npz"
intrinsics_path = "/home/roslab/catkin_ws/src/calib_scripts/calibration_data/camera_intrinsics.json"

# === CHESSBOARD SETTINGS ===
pattern_size = (8, 6)   # inner corners (columns, rows)
square_size = 0.025     # meters

# === Load camera intrinsics from JSON ===
with open(intrinsics_path, "r") as f:
    intrinsics = json.load(f)
K = np.array(intrinsics["camera_matrix"])
D = np.array(intrinsics["dist_coeff"]).reshape(-1, 1)

# === Generate 3D points for checkerboard target (Z=0) ===
objp = np.zeros((np.prod(pattern_size), 3), np.float32)
objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
objp *= square_size

# === Initialize motion pair lists ===
A_rot, A_trans = [], []  # Gripper-to-base (relative motion)
B_rot, B_trans = [], []  # Target-to-camera (relative motion)

files = sorted(glob.glob(os.path.join(npz_folder, "sample_*.npz")))
print(f"📂 Found {len(files)} .npz samples")

if len(files) < 2:
    print("❌ Need at least 2 samples.")
    exit(1)

def get_target_to_cam(image, i):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not found:
        print(f"[!] Checkerboard NOT found in sample {i}")
        return None, None
    _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)

# === Loop through image pairs ===
for i in range(len(files) - 1):
    data1 = np.load(files[i])
    data2 = np.load(files[i + 1])

    T_robot1 = np.array(data1["robot_pose"])
    T_robot2 = np.array(data2["robot_pose"])
    img1 = data1["image"]
    img2 = data2["image"]

    # A = gripper-to-base relative motion
    A = np.linalg.inv(T_robot1) @ T_robot2
    Ra, ta = A[:3, :3], A[:3, 3]

    # B = target-to-camera relative motion
    R1, t1 = get_target_to_cam(img1, i)
    R2, t2 = get_target_to_cam(img2, i+1)
    if R1 is None or R2 is None:
        continue

    B = R1 @ R2.T
    tb = t1 - B @ t2

    A_rot.append(Ra)
    A_trans.append(ta)
    B_rot.append(B)
    B_trans.append(tb)

print(f"\n🧠 Collected {len(A_rot)} valid motion pairs.")

if len(A_rot) < 2:
    print("❌ Not enough valid motion pairs for calibration.")
    exit(1)

# === Hand-eye calibration ===
R_cam2ee, t_cam2ee = cv2.calibrateHandEye(
    R_gripper2base=A_rot,
    t_gripper2base=A_trans,
    R_target2cam=B_rot,
    t_target2cam=B_trans,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# === Compose 4x4 transform
T = np.eye(4)
T[:3, :3] = R_cam2ee
T[:3, 3] = t_cam2ee.reshape(3)

print("\n🎯 Final Camera-to-End-Effector Transform (T_cam_to_ee):")
print(np.round(T, 4))

# === Save to file
np.save("T_camera_to_ee.npy", T)
print("💾 Saved as T_camera_to_ee.npy")

