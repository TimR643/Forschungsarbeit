import os
import json
import numpy as np

# Folder containing your JSON files
json_dir = "/home/roslab/catkin_ws/src/calib_scripts/samples/poses"
output_dir = json_dir  # You can change if needed

for filename in sorted(os.listdir(json_dir)):
    if filename.endswith(".json"):
        json_path = os.path.join(json_dir, filename)

        with open(json_path, "r") as f:
            matrix_list = json.load(f)

        T = np.array(matrix_list)  # Convert to NumPy array

        # Generate output filename: TBase2EE_XXX.npz
        index = filename.split("_")[1].split(".")[0]  # from pose_01.json
        out_filename = f"TBase2EE_{int(index):03}.npz"

        # Save as .npz file
        np.savez(os.path.join(output_dir, out_filename), T)
        print(f"✅ Converted {filename} -> {out_filename}")

