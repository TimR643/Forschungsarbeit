import cv2
import numpy as np
import os
import json

# SETTINGS
IMAGE_FOLDER = "samples/images"
OUTPUT_FOLDER = "samples/checkerboard_poses"
CAMERA_INTRINSICS_FILE = "calibration_data/camera_intrinsics.json"
CHECKERBOARD = (6, 8)
SQUARE_SIZE = 0.025  # meters

# PREPARE OUTPUT FOLDER
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# LOAD CAMERA INTRINSICS
with open(CAMERA_INTRINSICS_FILE, 'r') as f:
    calib = json.load(f)
camera_matrix = np.array(calib['camera_matrix'])
dist_coeffs = np.array(calib['dist_coeff'])

# PREPARE 3D POINTS OF CHECKERBOARD
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# PROCESS EACH IMAGE
image_files = sorted(os.listdir(IMAGE_FOLDER))
success_count = 0

for idx, filename in enumerate(image_files):
    image_path = os.path.join(IMAGE_FOLDER, filename)
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        # REFINE CORNERS
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # SOLVE PNP
        success, rvec, tvec = cv2.solvePnP(objp, corners_subpix, camera_matrix, dist_coeffs)

        if success:
            # SAVE POSE
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()

            pose_file = os.path.join(OUTPUT_FOLDER, f"pose_{idx:02d}.json")
            with open(pose_file, 'w') as f:
                json.dump({"T_checkerboard_in_camera": T.tolist()}, f, indent=2)

            success_count += 1
            print(f"[{idx+1}/{len(image_files)}] Saved pose: {pose_file}")

            # VISUAL FEEDBACK
            cv2.drawChessboardCorners(image, CHECKERBOARD, corners_subpix, ret)
            cv2.imshow('Checkerboard Detection', image)
            cv2.waitKey(100)

    else:
        print(f"[{idx+1}/{len(image_files)}] Checkerboard NOT detected: {filename}")

cv2.destroyAllWindows()

print(f"\n✅ Generated {success_count}/{len(image_files)} checkerboard poses.")
print(f"Saved in folder: {OUTPUT_FOLDER}")
