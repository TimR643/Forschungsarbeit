import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import os

class CameraCalibration:
    def __init__(self, image_folder, Transforms_folder, pattern_size=(6, 8), square_size=25/1000, ShowProjectError=True, ShowCorners=False):
        self.pattern_size = pattern_size
        self.square_size = square_size

        self.image_files = sorted(glob.glob(f'{image_folder}/*.png'))
        self.transform_files = sorted(glob.glob(f'{Transforms_folder}/*.npz'))
        self.images = [cv2.imread(f) for f in self.image_files]
        self.images = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) for img in self.images]
        self.All_T_base2EE_list = []

        for f in self.transform_files:
            try:
                data = np.load(f, allow_pickle=True)
                keys = data.files
                if 'arr_0' in keys:
                    T = np.array(data['arr_0'])
                else:
                    raise ValueError(f"Expected 'arr_0' in {f}, found keys: {keys}")
                if T.shape != (4, 4):
                    raise ValueError(f"Matrix in {f} is not 4x4: shape is {T.shape}")
                self.All_T_base2EE_list.append(T)
            except Exception as e:
                print(f"[!] Failed to load {f}: {e}")

        self.chessboard_corners, self.IndexWithImg = self.find_chessboard_corners(self.images, self.pattern_size, ShowCorners=ShowCorners)
        self.intrinsic_matrix = self.calculate_intrinsics(self.chessboard_corners, self.IndexWithImg,
                                                           self.pattern_size, self.square_size,
                                                           self.images[0].shape[:2], ShowProjectError=ShowProjectError)

        self.T_base2EE_list = [self.All_T_base2EE_list[i] for i in self.IndexWithImg]

        np.savez("IntrinsicMatrix.npz", self.intrinsic_matrix)

        self.RTarget2Cam, self.TTarget2Cam = self.compute_camera_poses(self.chessboard_corners,
                                                                       self.pattern_size, self.square_size,
                                                                       self.intrinsic_matrix)

        self.T_target2cam = [np.concatenate((R, T), axis=1) for R, T in zip(self.RTarget2Cam, self.TTarget2Cam)]
        for i in range(len(self.T_target2cam)):
            self.T_target2cam[i] = np.concatenate((self.T_target2cam[i], np.array([[0, 0, 0, 1]])), axis=0)

        self.T_cam2target = [np.linalg.inv(T) for T in self.T_target2cam]
        self.R_cam2target = [T[:3, :3] for T in self.T_cam2target]
        self.R_vec_cam2target = [cv2.Rodrigues(R)[0] for R in self.R_cam2target]
        self.T_cam2target = [T[:3, 3] for T in self.T_cam2target]

        self.TEE2Base = [np.linalg.inv(T) for T in self.T_base2EE_list]
        self.REE2Base = [T[:3, :3] for T in self.TEE2Base]
        self.R_vecEE2Base = [cv2.Rodrigues(R)[0] for R in self.REE2Base]
        self.tEE2Base = [T[:3, 3] for T in self.TEE2Base]

        if not os.path.exists("FinalTransforms"):
            os.mkdir("FinalTransforms")

        for i in range(0, 5):
            print("Method:", i)
            self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
                self.R_cam2target,
                self.T_cam2target,
                self.R_vecEE2Base,
                self.tEE2Base,
                method=i
            )
            print("The results for method", i, "are:")
            print("R_cam2gripper:", self.R_cam2gripper)
            print("t_cam2gripper:", self.t_cam2gripper)
            self.T_cam2gripper = np.concatenate((self.R_cam2gripper, self.t_cam2gripper), axis=1)
            self.T_cam2gripper = np.concatenate((self.T_cam2gripper, np.array([[0, 0, 0, 1]])), axis=0)
            np.savez(f"FinalTransforms/T_cam2gripper_Method_{i}.npz", self.T_cam2gripper)
            self.T_gripper2cam = np.linalg.inv(self.T_cam2gripper)
            np.savez(f"FinalTransforms/T_gripper2cam_Method_{i}.npz", self.T_gripper2cam)

        for i in range(0, 2):
            self.R_base2world, self.t_base2world, self.R_gripper2cam, self.t_gripper2cam = cv2.calibrateRobotWorldHandEye(
                self.RTarget2Cam, self.TTarget2Cam, self.REE2Base, self.tEE2Base, method=i)
            print("The results for method using calibrateRobotWorldHandEye", i+4, "are:")
            print("R_cam2gripper:", self.R_gripper2cam)
            print("t_cam2gripper:", self.t_gripper2cam)
            self.T_gripper2cam = np.concatenate((self.R_gripper2cam, self.t_gripper2cam), axis=1)
            self.T_gripper2cam = np.concatenate((self.T_gripper2cam, np.array([[0, 0, 0, 1]])), axis=0)
            np.savez(f"FinalTransforms/T_gripper2cam_Method_{i+4}.npz", self.T_gripper2cam)
            self.T_cam2gripper = np.linalg.inv(self.T_gripper2cam)
            np.savez(f"FinalTransforms/T_cam2gripper_Method_{i+4}.npz", self.T_cam2gripper)

    def find_chessboard_corners(self, images, pattern_size, ShowCorners=False):
        chessboard_corners = []
        IndexWithImg = []
        i = 0
        print("Finding corners...")
        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size)
            if ret:
                chessboard_corners.append(corners)
                cv2.drawChessboardCorners(image, pattern_size, corners, ret)
                if ShowCorners:
                    plt.imshow(image)
                    plt.title("Detected corner in image: " + str(i))
                    plt.show()
                if not os.path.exists("DetectedCorners"):
                    os.makedirs("DetectedCorners")
                cv2.imwrite("DetectedCorners/DetectedCorners" + str(i) + ".png", image)
                IndexWithImg.append(i)
            else:
                print("No chessboard found in image: ", i)
            i += 1
        return chessboard_corners, IndexWithImg

    def compute_camera_poses(self, chessboard_corners, pattern_size, square_size, intrinsic_matrix, Testing=False):
        object_points = np.zeros((pattern_size[0] * pattern_size[1], 3), dtype=np.float32)
        object_points[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
        RTarget2Cam = []
        TTarget2Cam = []
        i = 1
        for corners in chessboard_corners:
            _, rvec, tvec = cv2.solvePnP(object_points, corners, intrinsic_matrix, None)
            if Testing:
                print("Current iteration:", i)
                print("rvec:", rvec)
            i += 1
            R, _ = cv2.Rodrigues(rvec)
            RTarget2Cam.append(R)
            TTarget2Cam.append(tvec)
        return RTarget2Cam, TTarget2Cam

    def calculate_intrinsics(self, chessboard_corners, IndexWithImg, pattern_size, square_size, ImgSize, ShowProjectError=True):
        imgpoints = chessboard_corners
        objpoints = []
        for i in range(len(IndexWithImg)):
            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
            objpoints.append(objp)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, ImgSize, None, None)
        print("The projection error from the calibration is:",
              self.calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist, ShowProjectError))
        return mtx

    def calculate_reprojection_error(self, objpoints, imgpoints, rvecs, tvecs, mtx, dist, ShowPlot=False):
        total_error = 0
        num_points = 0
        errors = []
        for i in range(len(objpoints)):
            imgpoints_projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            imgpoints_projected = imgpoints_projected.reshape(-1, 1, 2)
            error = cv2.norm(imgpoints[i], imgpoints_projected, cv2.NORM_L2) / len(imgpoints_projected)
            errors.append(error)
            total_error += error
            num_points += 1
        mean_error = total_error / num_points
        if ShowPlot:
            fig, ax = plt.subplots()
            img_indices = range(1, len(errors) + 1)
            ax.bar(img_indices, errors)
            ax.set_xlabel('Image Index')
            ax.set_ylabel('Reprojection Error')
            ax.set_title('Reprojection Error for Each Image')
            plt.show()
            fig.savefig('ReprojectionError.png')
        return mean_error

if __name__ == "__main__":
    image_folder = "/home/roslab/catkin_ws/src/calib_scripts/samples/images"
    PoseFolder = "/home/roslab/catkin_ws/src/calib_scripts/samples/poses"
    calib = CameraCalibration(image_folder, PoseFolder, ShowProjectError=True)

