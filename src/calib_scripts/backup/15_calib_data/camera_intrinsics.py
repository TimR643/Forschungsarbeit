#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os

# --- SETTINGS ---
image_folder = "/home/roslab/catkin_ws/src/calib_scripts/15_calib_data/images"  # Full path to image folder
pattern_size = (8, 6)                               # Inner corners (not squares!)
square_size  = 0.025                                # meters

# Prepare known 3D object points in checkerboard frame
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points in world
imgpoints = []  # 2D points in image
image_shape = None

# Load image paths
images = sorted(glob.glob(os.path.join(image_folder, "image_*.png")))
print(f"\n📷 Found {len(images)} images in {image_folder}")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"⚠️ Could not read image: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if image_shape is None:
        image_shape = gray.shape[::-1]  # (width, height)

    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if found:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners)
        objpoints.append(objp)
        print(f"✓ Found corners in {os.path.basename(fname)}")
    else:
        print(f"✗ Chessboard not found in {os.path.basename(fname)}")

# Error check
if len(objpoints) == 0:
    print("\n❌ No valid chessboard detections. Check pattern size and images.")
    exit()

# Calibrate
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_shape, None, None
)

# Results
print("\n=== 🎯 Calibration Results ===")
print("Camera matrix (K):\n", K)
print("\nDistortion coefficients (k1, k2, p1, p2, k3):\n", dist.ravel())
print(f"\nRMS reprojection error: {ret:.4f}")

# Define and use the path:
save_path = "/home/roslab/catkin_ws/src/calib_scripts/15_calib_data/camera_intrinsics.npz"
np.savez(save_path, K=K, dist=dist)
print(f"\n📁 Saved intrinsics to: {save_path}")
