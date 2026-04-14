import numpy as np

T = np.load("T_camera_to_ee.npy")
print("T_camera_to_ee =\n", T)
print("\nRotation matrix:\n", T[:3, :3])
print("Translation vector:\n", T[:3, 3])
