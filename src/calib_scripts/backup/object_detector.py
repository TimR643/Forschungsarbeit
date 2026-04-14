import cv2
import numpy as np
import csv
import os

# -------- Paths --------
COLOR_IMAGE_PATH = "test_image.png"
DEPTH_IMAGE_PATH = "test_depth.npy"
CAMERA_MATRIX_PATH = "camera_matrix.npy"
HAND_EYE_TRANSFORM_PATH = "camera_to_gripper_named.npz"
CSV_OUTPUT_PATH = "sorted_objects.csv"

# -------- Settings --------
COLOR_RANGES = {
    "red":    ((0, 100, 100), (10, 255, 255)),
    "green":  ((50, 100, 100), (70, 255, 255)),
    "blue":   ((100, 100, 100), (130, 255, 255)),
    "yellow": ((20, 100, 100), (35, 255, 255)),
}

MIN_AREA = 5
VISUALIZE = True
SAVE_TO_CSV = True

# -------- Load Inputs --------
color_img = cv2.imread(COLOR_IMAGE_PATH)
depth_img = np.load(DEPTH_IMAGE_PATH)
K = np.load(CAMERA_MATRIX_PATH)
T_cam2gripper = np.load(HAND_EYE_TRANSFORM_PATH)['T_cam2gripper']

if color_img is None:
    raise FileNotFoundError(f"[!] Color image not found at {COLOR_IMAGE_PATH}")

if depth_img.shape != color_img.shape[:2]:
    print(f"[!] Depth image shape {depth_img.shape} does not match color {color_img.shape[:2]}")
    print("[!] Resizing depth image to match RGB size...")
    depth_img = cv2.resize(depth_img, (color_img.shape[1], color_img.shape[0]), interpolation=cv2.INTER_NEAREST)

# -------- Helper Functions --------
def pixel_to_camera_coords(u, v, z, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])

def classify_shape(approx):
    if len(approx) == 3:
        return "triangle"
    elif len(approx) == 4:
        return "rectangle"
    elif len(approx) > 6:
        return "circle"
    else:
        return "unknown"

def assign_box(color, shape):
    if color == "red":
        return "Box A"
    elif color == "green":
        return "Box B"
    elif color == "blue":
        return "Box C"
    else:
        return "Unknown Box"

# -------- Detection + Sorting --------
hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
sorted_objects = []

print("\n[INFO] Detected and Sorted Objects:")

for color_name, (lower, upper) in COLOR_RANGES.items():
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    cv2.imshow(f"{color_name} mask", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"[DEBUG] {color_name}: {len(contours)} contours found")

    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(f"[DEBUG] Contour area: {area}")
        if area < MIN_AREA:
            continue

        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        shape = classify_shape(approx)

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        z_raw = depth_img[cy, cx]
        z = z_raw / 1000.0
        if z == 0 or np.isnan(z):
            print(f"[!] Skipping object at ({cx}, {cy}) — invalid depth.")
            continue

        point_cam = pixel_to_camera_coords(cx, cy, z, K)
        point_cam_h = np.append(point_cam, 1.0)
        point_gripper = T_cam2gripper @ point_cam_h
        box = assign_box(color_name, shape)

        print(f" - {color_name} {shape} → {box}")
        print(f"   • Image: ({cx}, {cy})  Depth: {z:.3f} m")
        print(f"   • 3D (camera): {point_cam}")
        print(f"   • 3D (gripper): {point_gripper[:3]}\n")

        sorted_objects.append({
            "color": color_name,
            "shape": shape,
            "box": box,
            "cx": cx,
            "cy": cy,
            "depth": z,
            "camera_x": point_cam[0],
            "camera_y": point_cam[1],
            "camera_z": point_cam[2],
            "gripper_x": point_gripper[0],
            "gripper_y": point_gripper[1],
            "gripper_z": point_gripper[2],
        })

        if VISUALIZE:
            label = f"{color_name} {shape} → {box}"
            cv2.drawContours(color_img, [cnt], -1, (255, 255, 255), 2)
            cv2.putText(color_img, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)

# -------- CSV Export --------
if SAVE_TO_CSV and sorted_objects:
    with open(CSV_OUTPUT_PATH, "w", newline='') as csvfile:
        fieldnames = list(sorted_objects[0].keys())
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sorted_objects)
    print(f"[✓] Exported {len(sorted_objects)} objects to {CSV_OUTPUT_PATH}")

# -------- Display --------
if VISUALIZE:
    cv2.imshow("Object Sorting", color_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

