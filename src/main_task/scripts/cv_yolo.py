#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import os
from scipy.optimize import minimize_scalar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from ultralytics import YOLO
import pytesseract
import easyocr
pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'


class detection_and_recognition:
    def __init__(self):
        self.bridge = CvBridge()

        # Bildpuffer
        self.latest_image       = None
        self.latest_depth       = None

        # Kameramodelle
        self.camera_model               = PinholeCameraModel()
        self.depth_camera_model         = PinholeCameraModel()
        self.cam_info_received          = False
        self.cam_info_depth_received    = False

        # ROS Subscriber
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_depth_callback)

        # YOLO Modell laden
        self.model = YOLO("/home/roslab/catkin_ws/src/main_task/weights/best_14.pt")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr("Fehler beim Konvertieren des BRG Farbbildes: %s", e)


    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            rospy.logerr("Fehler beim Konvertieren des Tiefenbildes: %s", e)


    def camera_info_callback(self, msg):
        if not self.cam_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.cam_info_received = True
            #rospy.loginfo("Farbkamera-Info empfangen.")


    def camera_info_depth_callback(self, msg):
        if not self.cam_info_depth_received:
            self.depth_camera_model.fromCameraInfo(msg)
            self.cam_info_depth_received = True
            #rospy.loginfo("Tiefenkamera-Info empfangen.")


    def wait_for_data(self):
        rate = rospy.Rate(10)
        while (self.latest_image is None or self.latest_depth is None or not self.cam_info_received) and not rospy.is_shutdown():
            rospy.loginfo("Warte auf Bild- und Kameradaten ...")
            rate.sleep()


    # Steine erkennen
    def get_all_detections(self):
        self.wait_for_data()
        detections = []

        yolo_results = self.model(self.latest_image, conf=0.8)

        for r in yolo_results:
            if hasattr(r, "obb"):
                for i, polygon in enumerate(r.obb.xyxyxyxy):
                    # polygon = [x1, y1, x2, y2, x3, y3, x4, y4]
                    points = polygon.view(4, 2).cpu().numpy().astype(np.float32)

                    # Mittelpunkt berechnen (optional)
                    xs, ys = points[:, 0], points[:, 1]
                    cx, cy = int(xs.mean()), int(ys.mean())

                    # cv2.minAreaRect erwartet Punkte als float32 und Form (N,1,2)
                    pts    = points.reshape((-1, 1, 2))

                    # Rotated Rectangle berechnen
                    rect   = cv2.minAreaRect(pts)  # ((cx, cy), (w, h), angle)

                    width, height = rect[1]
                    angle_cv = rect[2]

                    # Korrigiere Winkel, falls Breite < Höhe
                    if width < height:
                        angle_cv += 90  # Winkel jetzt im Bereich [0, 90)

                    angle_deg = 90 - angle_cv

                    # Winkel auf [-90, 90] normieren
                    if angle_deg > 90:
                        angle_deg -= 180
                    elif angle_deg < -90:
                        angle_deg += 180

                    # Runde auf ganze Grad
                    angle_deg = int(round(angle_deg))

                    # Tiefe aus ROI auslesen
                    cx_d, cy_d = cx, cy
                    roi_z = self.latest_depth[cy_d - 3:cy_d + 3, cx_d - 3:cx_d + 3]

                    valid_z = roi_z[np.isfinite(roi_z) & (roi_z > 0.0)]
                    if len(valid_z) == 0:
                        continue

                    depth_value = np.mean(valid_z) * 0.001  # in Meter

                    ray = self.camera_model.projectPixelTo3dRay((cx, cy))

                    label_id = int(r.obb.cls[i].item())
                    label = self.model.names[label_id]
                    confidence = float(r.obb.conf[i].item())

                    detection = {
                        "label": label,
                        "position_camera": (ray[0] * depth_value, ray[1] * depth_value, 0.5),
                        "pixel": (cx, cy),
                        "confidence": confidence,
                        "angle": angle_deg,
                        "polygon": points.tolist(),
                    }
                    detections.append(detection)
            else:
                rospy.logwarn("Keine OBB-Ergebnisse vorhanden.")

        # Bild mit Bounding Boxes anzeigen
        output_path = os.path.join(os.path.dirname(__file__), "../images/detected_output.png")
        cv2.imwrite(output_path, yolo_results[0].plot())

        rospy.loginfo(f"verwertbare Objekte YoloV8: {detections}")
        return detections


    # Schrift erkennen
    def get_drop_off_position(self):
        self.wait_for_data()
        detections = []

        reader = easyocr.Reader(['de', 'en'])

        # EasyOCR ausführen
        results = reader.readtext(self.latest_image)

        for (bbox, text, confidence) in results:
            text = text.strip()
            if text == '':
                continue

            # Bounding Box: 4 Punkte (x, y), wir berechnen Rechteck
            pts = np.array(bbox).astype(int)
            x_min, y_min = np.min(pts, axis=0)
            x_max, y_max = np.max(pts, axis=0)
            w, h = x_max - x_min, y_max - y_min
            cx, cy = x_min + w // 2, y_min + h // 2

            cx_d, cy_d = int(cx * 1), int(cy * 1)
            roi_z = self.latest_depth[cy_d - 3:cy_d + 3, cx_d - 3:cx_d + 3]
            roi_xy = self.latest_depth[cy - 3:cy + 3, cx - 3:cx + 3]

            valid_z = roi_z[np.isfinite(roi_z) & (roi_z > 0.0)]
            valid_xy = roi_xy[np.isfinite(roi_xy) & (roi_xy > 0.0)]
            if len(valid_z) == 0 or len(valid_xy) == 0:
                rospy.logwarn(f"OCR: '{text}' hat ungültige Tiefenwerte.")
                continue

            depth_z = np.mean(valid_z) * 0.001
            depth_xy = np.mean(valid_xy) * 0.001
            ray = self.camera_model.projectPixelTo3dRay((cx, cy))

            #rospy.loginfo(f"OCR: '{text}' bei (x={x_min}, y={y_min}, w={w}, h={h}): Pixel-Zentrum: ({cx}, {cy})")
            cv2.rectangle(self.latest_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            detections.append({
                "label": text.lower(),
                "position_camera": (ray[0] * depth_xy, ray[1] * depth_xy, 0.5),
                "pixel": (cx, cy),
                "confidence": confidence,
                "angle": 0,
            })

        # Bild mit Bounding Boxes anzeigen
        output_path = os.path.join(os.path.dirname(__file__), "../images/ocr_detected.png")
        cv2.imwrite(output_path, self.latest_image)

        rospy.loginfo(f"verwertbare Objekte EasyOCR: {detections}")
        return detections


# Test aus Terminal
if __name__ == "__main__":
    rospy.init_node("cv_node", anonymous=True)
    detector = detection_and_recognition()
    rospy.sleep(1.0)
    rospy.loginfo("Starte einmalige Objekterkennung...")

    #detector.get_drop_off_position()
    

    all_objects = detector.get_drop_off_position()
    print(all_objects)
