import os
import json
import numpy as np
import cv2
import glob

def load_samples(folder):
    files = sorted(glob.glob(os.path.join(folder, "sample_*.json")))
    samples = []
    for f in files:
        with open(f, 'r') as file:
            data = json.load(file)
            samples.append(data)
    return samples

def rvec_tvec_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(np.array(rvec))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).reshape(3)
    return T

def matrix_to_rt(T):
    R = T[:3, :3]
    t = T[:3, 3]
    return R, t

def compute_handeye(samples):
    A_rot, A_trans = [], []
    B_rot, B_trans = [], []

    for i in range(len(samples) - 1):
        T_robot_prev = np.array(samples[i]['robot_pose'])
        T_robot_curr = np.array(samples[i+1]['robot_pose'])
        T_cam_prev = rvec_tvec_to_matrix(samples[i]['rvec'], samples[i]['tvec'])
        T_cam_curr = rvec_tvec_to_matrix(samples[i+1]['rvec'], samples[i+1]['tvec'])

        A = np.linalg.inv(T_robot_prev) @ T_robot_curr
        B = T_cam_prev @ np.linalg.inv(T_cam_curr)

        R_a, t_a = matrix_to_rt(A)
        R_b, t_b = matrix_to_rt(B)

        A_rot.append(R_a)
        A_trans.append(t_a)
        B_rot.append(R_b)
        B_trans.append(t_b)

    print(f"🧠 Using {len(A_rot)} motion pairs...")

    R_cam2ee, t_cam2ee = cv2.calibrateHandEye(
        R_gripper2base=A_rot,
        t_gripper2base=A_trans,
        R_target2cam=B_rot,
        t_target2cam=B_trans,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    T = np.eye(4)
    T[:3, :3] = R_cam2ee
    T[:3, 3] = t_cam2ee.reshape(3)
    return T

if __name__ == "__main__":
    samples = load_samples("calib_data")
    if len(samples) < 2:
        print("❌ Need at least 2 samples to compute relative motions.")
        exit(1)

    T_cam2ee = compute_handeye(samples)

    print("\n🎯 Final Camera-to-End-Effector Transform:")
    print(np.round(T_cam2ee, 4))

    np.save("T_camera_to_ee.npy", T_cam2ee)
    print("💾 Saved as T_camera_to_ee.npy")
