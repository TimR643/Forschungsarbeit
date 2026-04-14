#!/usr/bin/env python3
import cv2
import rospy
import torch
import numpy as np
import os
import math
from scipy.optimize import minimize_scalar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import pytesseract
import easyocr
pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'


class detection_and_recognition:
    def __init__(self):
        self.bridge = CvBridge()

        # Bildpuffer
        self.latest_image       = None
        self.latest_image_rgb   = None
        self.latest_depth       = None
        self.last_results       = None  # Ergebnis von YOLO-Inferenz zwischenspeichern
        self.last_ocr_result    = None

        # Kameramodelle
        self.camera_model               = PinholeCameraModel()
        self.depth_camera_model         = PinholeCameraModel()
        self.cam_info_received          = False
        self.cam_info_depth_received    = False

        # Detektion
        self.object_camera_coords       = (0, 0, 0)
        self.detected_class             = {"color": None, "shape": None}

        # ROS Subscriber
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_depth_callback)

        # YOLO Modell laden
        self.model      = torch.hub.load('yolov5', 'custom', path='main_task/weights/best_11.pt', source='local')
        self.model.conf = 0.8

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
            rospy.loginfo("Farbkamera-Info empfangen.")


    def camera_info_depth_callback(self, msg):
        if not self.cam_info_depth_received:
            self.depth_camera_model.fromCameraInfo(msg)
            self.cam_info_depth_received = True
            rospy.loginfo("Tiefenkamera-Info empfangen.")


    def wait_for_data(self):
        rate = rospy.Rate(10)
        while (self.latest_image is None or self.latest_depth is None or not self.cam_info_received) and not rospy.is_shutdown():
            rospy.loginfo("Warte auf Bild- und Kameradaten ...")
            rate.sleep()


    def run_yolo_once(self):
        """YOLO nur einmal pro Schleifendurchlauf aufrufen und Ergebnisse zwischenspeichern"""
        self.wait_for_data()
        self.latest_image_rgb   = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB)
        self.last_results       = self.model(self.latest_image_rgb)

        # === Anzeige des Ergebnisbilds aktualisieren ===
        rendered_rgb = self.last_results.render()[0]            # YOLO-Rendering (RGB)
        result_bgr   = cv2.cvtColor(rendered_rgb, cv2.COLOR_RGB2BGR)
        #cv2.imshow("YOLO Ergebnis", result_bgr)
        #cv2.waitKey(1)  # erforderlich für Anzeige (1 ms)



    def detect_object(self):
        self.run_yolo_once()
        df = self.last_results.pandas().xyxy[0]
        if df.empty:
            rospy.logwarn("Kein Objekt erkannt.")
            return False
        rospy.loginfo(f"{len(df)} Objekt(e) erkannt.")
        return True


    def save_images(self):
        script_dir  = os.path.dirname(os.path.abspath(__file__))
        rgb_path    = os.path.join(script_dir, "../images/raw_color.png")
        depth_path  = os.path.join(script_dir, "../images/raw_depth.png")
        output_path = os.path.join(script_dir, '../images/detected_output.png')

        # Yolo Ausgabe speichern und in BRG wandeln
        rendered_rgb        = self.last_results.render()[0]
        self.last_result    = cv2.cvtColor(rendered_rgb, cv2.COLOR_RGB2BGR)

        cv2.imwrite(output_path, self.last_result)
        cv2.imwrite(rgb_path, self.latest_image)
        cv2.imwrite(depth_path, self.latest_depth)
        rospy.loginfo("Raw_Pictures_Saved")

    def solve_rotation(self, y, aspect_ratioX, aspect_ratioY):
        def error(x):
            return (((aspect_ratioY*math.sin(x) + aspect_ratioX * math.cos(x)) / (aspect_ratioX * math.sin(x) + aspect_ratioY*math.cos(x))) - y)**2

        result = minimize_scalar(error, bounds=(0, math.pi/2), method='bounded')
        
        if result.success:
            return math.degrees(result.x)
        else:
            return None

    def get_drop_off_position(self):
        self.wait_for_data()
        reader = easyocr.Reader(['de', 'en'])  # Sprachmodelle anpassen

        detections = []

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

            rospy.loginfo(f"OCR: '{text}' bei (x={x_min}, y={y_min}, w={w}, h={h}): Pixel-Zentrum: ({cx}, {cy})")
            cv2.rectangle(self.latest_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            detections.append({
                "label": text.lower(),  # ggf. dynamisch aus OCR-Text ableiten
                "position_camera": (ray[0] * depth_xy, ray[1] * depth_xy, 0.5),
                "pixel": (cx, cy),
                "confidence": confidence,
                "angle": 0,
            })

        # Bild mit Bounding Boxes anzeigen
        output_path = os.path.join(os.path.dirname(__file__), "../images/ocr_detected.png")
        cv2.imwrite(output_path, self.latest_image)
        rospy.loginfo(f"OCR-Bild gespeichert unter: {output_path}")

        rospy.loginfo(f"[EASYOCR] Anzahl verwertbarer Objekte: {len(detections)}")
        rospy.loginfo(detections)

        return detections


    def get_all_detections_masked(self):
        df = self.last_results.pandas().xyxy[0]
        detections = []
        hsv_image = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "red": [
                [np.array([0, 20, 20]), np.array([10, 255, 255])],
                [np.array([160, 20, 20]), np.array([180, 255, 255])]
            ],
            "green": [
                [np.array([30, 20, 20]), np.array([90, 255, 255])]
            ],
            "blue": [
                [np.array([85, 10, 10]), np.array([150, 255, 255])]
            ],
            "yellow": [
                [np.array([20, 20, 20]), np.array([35, 255, 255])]              
            ]
        }

        for _, row in df.iterrows():
            #print(row)
            label = row['name']
            if any(c in label for c in color_ranges):
                color_key = next(c for c in color_ranges if c in label)
            else:
                continue

            xmin, xmax = int(row['xmin']), int(row['xmax'])
            ymin, ymax = int(row['ymin']), int(row['ymax'])

            # aspect_ratio = (xmax - xmin)/(ymax - ymin)
            # angle = self.solve_rotation(aspect_ratio, 1, 2)
            # rospy.loginfo(f"angle: {angle}")

            roi = hsv_image[ymin:ymax, xmin:xmax]
            #print(roi)

            ranges = color_ranges[color_key]
            mask = cv2.inRange(roi, ranges[0][0], ranges[0][1])
            for r in ranges[1:]:
                mask = cv2.bitwise_or(mask, cv2.inRange(roi, r[0], r[1]))

            # Orientierung aus Konturen bestimmen
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            angle = None
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(largest_contour)
                angle = rect[-1]
                if rect[1][0] < rect[1][1]:
                    angle = 90 + angle
                #Winkel normalisieren
                if angle < -45:
                    angle += 90
            rospy.loginfo(angle)
            M = cv2.moments(mask)
            if M["m00"] == 0:
                print("Farbe")
                continue

            cx          = int(M["m10"] / M["m00"]) + xmin
            cy          = int(M["m01"] / M["m00"]) + ymin
            cx_d, cy_d  = int(cx * 1), int(cy * 1)

            roi_z       = self.latest_depth[cy_d-3:cy_d+3, cx_d-3:cx_d+3]
            roi_xy      = self.latest_depth[cy-3:cy+3, cx-3:cx+3]
            valid_z     = roi_z[np.isfinite(roi_z) & (roi_z > 0.0)]
            valid_xy    = roi_xy[np.isfinite(roi_xy) & (roi_xy > 0.0)]
            if len(valid_z) == 0 or len(valid_xy) == 0:
                print("Test")

            depth_z     = np.mean(valid_z) * 0.001
            depth_xy    = np.mean(valid_xy) * 0.001
            ray         = self.camera_model.projectPixelTo3dRay((cx, cy))

            detections.append({
                "label":            label,
                "position_camera":  (ray[0] * depth_xy, ray[1] * depth_xy, 0.5),
                "pixel":            (cx, cy),
                "confidence":       row['confidence'],
                "angle":            angle,
            })
        self.save_images()
        rospy.loginfo(f"[MASKED] Anzahl verwertbarer Objekte: {len(detections)}")
        return detections


# Test aus Terminal
if __name__ == "__main__":
    rospy.init_node("cv_node", anonymous=True)
    detector = detection_and_recognition()
    rospy.sleep(1.0)
    rospy.loginfo("Starte einmalige Objekterkennung...")

    #detector.get_drop_off_position()
    

    if detector.detect_object():
        all_objects = detector.get_all_detections_masked()
        print(all_objects)
        detector.save_images()
        for i, obj in enumerate(all_objects):
            rospy.loginfo(f"[{i+1}] Klasse: {obj['label']}, 3D-Position (HSV-Maske): {obj['position_camera']}")
    else:
        rospy.logwarn("Kein Objekt erkannt.")