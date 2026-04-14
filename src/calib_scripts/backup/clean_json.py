import os
import json
import argparse
import numpy as np
from glob import glob
from numpy.linalg import norm

def load_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    robot_T = np.array(data["robot_pose"])
    tvec = np.array(data["tvec"]).reshape(3)
    return robot_T, tvec, data

def angle_between_rotations(R1, R2):
    dR = R1.T @ R2
    angle = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0))
    return np.degrees(angle)

def should_keep_sample(last_robot_T, last_tvec, new_robot_T, new_tvec,
                       robot_pos_thresh=0.03, robot_angle_thresh=5.0,
                       tvec_thresh=0.03):
    if last_robot_T is None or last_tvec is None:
        return True

    # Robot position diff
    dp_robot = norm(last_robot_T[:3, 3] - new_robot_T[:3, 3])
    angle_robot = angle_between_rotations(last_robot_T[:3, :3], new_robot_T[:3, :3])

    # Checkerboard tvec (camera space) diff
    dp_tvec = norm(last_tvec - new_tvec)

    return (dp_robot > robot_pos_thresh or
            angle_robot > robot_angle_thresh or
            dp_tvec > tvec_thresh)

def main(input_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    json_files = sorted(glob(os.path.join(input_dir, "*.json")))

    last_robot_T = None
    last_tvec = None
    kept = 0

    print(f"📂 Checking {len(json_files)} files...")
    for path in json_files:
        robot_T, tvec, data = load_json(path)
        if should_keep_sample(last_robot_T, last_tvec, robot_T, tvec):
            out_path = os.path.join(output_dir, os.path.basename(path))
            with open(out_path, 'w') as f:
                json.dump(data, f, indent=4)
            last_robot_T = robot_T
            last_tvec = tvec
            kept += 1
            print(f"✅ Kept: {os.path.basename(path)}")
        else:
            print(f"⏩ Skipped: {os.path.basename(path)}")

    print(f"\n🎉 Final result: {kept} samples saved to {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Smarter filter for robot calibration JSONs.")
    parser.add_argument("--input_dir", required=True, help="Path to original JSONs")
    parser.add_argument("--output_dir", required=True, help="Where to save filtered JSONs")
    args = parser.parse_args()

    main(args.input_dir, args.output_dir)
