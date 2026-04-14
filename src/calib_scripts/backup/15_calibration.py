import glob, cv2, numpy as np, os

# --- SETTINGS ---
folder      = "/path/to/calib_images"
pattern_sz  = (9, 6)
square_size = 0.025    # meters

# prepare object points, like (0,0,0),(s,0,0),… in the checkerboard frame
objp = np.zeros((pattern_sz[0]*pattern_sz[1],3), np.float32)
objp[:,:2] = np.indices(pattern_sz).T.reshape(-1,2)
objp *= square_size

objpoints, imgpoints = [], []
imgs = sorted(glob.glob(os.path.join(folder, "calib_*.png")))

for fn in imgs:
    img = cv2.imread(fn)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern_sz)
    if not found: continue
    corners = cv2.cornerSubPix(
        gray, corners, (11,11), (-1,-1),
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    objpoints.append(objp)
    imgpoints.append(corners)

# calibrate
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("RMS error:", ret)
print("Camera matrix K:\n", K)
print("Distortion coeffs:\n", dist.ravel())
