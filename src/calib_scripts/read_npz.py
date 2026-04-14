import numpy as np

# Path to your NPZ file
file_path = '~/catkin_ws/src/calib_scripts/samples/poses/TBase2EE_000.npz'  # <-- change this to your file path

# Load the NPZ file
data = np.load(file_path, allow_pickle=True)

# Print keys
print(f"[INFO] Keys found: {data.files}\n")

# Print contents
for key in data.files:
    print(f"[INFO] Key: {key}")
    value = data[key]
    if isinstance(value.item(), dict):
        for sub_key, sub_val in value.item().items():
            print(f"  {sub_key}:")
            print(f"    {sub_val}\n")
    else:
        print(value)
    print("-" * 40)

