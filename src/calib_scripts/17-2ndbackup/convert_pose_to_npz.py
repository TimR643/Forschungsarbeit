import os
import json
import cv2
import numpy as np

# Set folders
root_dir = "/home/roslab/catkin_ws/src/calib_scripts/samples"
image_dir = os.path.join(root_dir, "images")
pose_dir = os.path.join(root_dir, "poses")
npz_dir = os.path.join(root_dir, "npz")
os.makedirs(npz_dir, exist_ok=True)

# List sorted files
image_files = sorted(f for f in os.listdir(image_dir) if f.endswith(".png"))
pose_files = sorted(f for f in os.listdir(pose_dir) if f.endswith(".json"))

assert len(image_files) == len(pose_files), "Mismatch between images and poses!"

for img_file, pose_file in zip(image_files, pose_files):
    idx = img_file.split("_")[-1].split(".")[0]

    # Load image
    img_path = os.path.join(image_dir, img_file)
    image = cv2.imread(img_path)

    # Load pose
    pose_path = os.path.join(pose_dir, pose_file)
    with open(pose_path, "r") as f:
        data = json.load(f)

    T = np.array(data["robot_pose"])
    camera_frame = data.get("camera_frame", "unknown")

    # Save .npz
    out_path = os.path.join(npz_dir, f"sample_{idx}.npz")
    np.savez_compressed(out_path, image=image, robot_pose=T, camera_frame=camera_frame)
    print(f"✅ Saved {out_path}")

