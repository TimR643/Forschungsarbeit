import numpy as np

# Step 1: Load original hand-eye matrix
T = np.load("T_camera_to_ee.npy")  # shape (4, 4)

# Step 3: Rotate both rotation and translation
T_new = np.eye(4)
T_new[:3, :3] = R_x_90 @ T[:3, :3]
T_new[:3, 3]  = R_x_90 @ T[:3, 3]  # rotate translation vector too

# ✅ Step 4: Add small position correction (adjust as needed)
T_new[:3, 3] += np.array([0.02, 0.05, 0.0])  # Try small values and tweak

# Step 5: Save and reload this new matrix
np.save("T_camera_to_ee_rotated.npy", T_new)
