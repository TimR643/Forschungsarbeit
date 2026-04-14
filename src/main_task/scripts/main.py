#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg

from statemachine import StateMachine, State

# Import of the other files and classes in this package
from cv import detection_and_recognition
from tf_calculate import tf_manager
from controller import controller
from brick_manager import BrickManager

class Process_Manager(StateMachine):
    # States
    start                           = State("Start", initial=True)
    calibration                     = State("Calibration")
    detection_and_classification    = State("Detection_and_Classification")
    pick_object                     = State("Pick_Object")
    place_object                    = State("Place_Object")
    finished                        = State("Finished", final=True)

    # Transitionen
    start_to_calibration            = start.to(calibration)
    calibration_to_detection        = calibration.to(detection_and_classification)
    detection_to_pick               = detection_and_classification.to(pick_object)
    detection_to_detection          = detection_and_classification.to(detection_and_classification)
    pick_to_place                   = pick_object.to(place_object)
    place_to_detection              = place_object.to(detection_and_classification)
    place_to_finished               = place_object.to(finished)

    def __init__(self):
        # Instanze anlegen
        self.camera_tasks       = detection_and_recognition()
        self.controller_tasks   = controller()
        self.tf_tasks           = tf_manager()
        self.brick_manager      = BrickManager()

        # StateMachine initialisieren
        super().__init__()


    # All actions which in the state start happens
    def on_enter_start(self):
        rospy.loginfo("------------------------------------------ Starting ------------------------------------------")
        self.controller_tasks.home_position()   # Move Home Position
        self.controller_tasks.init_gripper()    # Init Gripper
        self.start_to_calibration()             # jump to next State in StateMachine 

    def on_enter_calibration(self):
        rospy.loginfo("------------------------------------------ Camera Calibration ------------------------------------------") 

        # jump to next State in StateMachine
        self.calibration_to_detection()

    def on_enter_detection_and_classification(self):
        rospy.loginfo("------------------------------------------ Detect Object ------------------------------------------") 
        self.controller_tasks.detection_position()                      # moves robot arm to camera detection position
        while self.camera_tasks.detect_object() == False:               # camera detect objects in loop until sucess
            rospy.loginfo("No object detected. Retry")
        all_objects = self.camera_tasks.get_all_detections_masked()     # Get 3d camera coordinates from the object 
        if len(all_objects)>= 1:
            # Hier Funktion zum Entscheiden welcher gepickt werden soll
            self.object_camera_coords = all_objects[0]['position_camera']
            self.label = all_objects[0]['label']
            self.angle = all_objects[0]['angle']
            rospy.loginfo(f"Object anfahren: {self.label}")
            rospy.loginfo(f"Object an Kamera koordinaten: {self.object_camera_coords}")
            self.detection_to_pick()                                        # jump to next State in StateMachine
        else:
            rospy.loginfo(f"Kein verwertbares Object erkannt wiederholen")
            self.detection_to_detection()


    def on_enter_pick_object(self):
        rospy.loginfo("------------------------------------------ Pick object ------------------------------------------") 
        object_base_coords      = self.tf_tasks.tf_image_to_base(self.object_camera_coords)# From camera coords to base coords                              
        object_base_rotation    = (180,0,-45 + self.angle)                   
        self.controller_tasks.pick_object(object_base_coords, object_base_rotation)         # Pick object
        self.pick_to_place()                                                                # jump to next State in StateMachine


    def on_enter_place_object(self):
        rospy.loginfo("------------------------------------------ Place object ------------------------------------------") 
        
        if self.label in ["red1x1", "green1x1", "blue1x1", "yellow1x1"]:
            self.controller_tasks.camera_position_1x1()
            all_objects = self.camera_tasks.get_drop_off_position()

            color = ''.join([c for c in self.label if not c.isdigit() and c != 'x'])            # Farbe extrahieren (z.B. 'red' aus 'red1x1')

            # Filtere alle Objekte nach Farbe
            matching_objects = [obj for obj in all_objects if obj['label'] == color]

            if not matching_objects:
                rospy.logwarn(f"Kein Objekt der Farbe '{color}' gefunden!")
                return

            # Nimm z. B. das Objekt mit der höchsten Confidence
            best_object = max(matching_objects, key=lambda x: x['confidence'])

            coords = best_object["position_camera"]
            object_base_coords = self.tf_tasks.tf_image_to_base(coords)
            object_base_rotation = (180, 0, -45)

            self.controller_tasks.place_object(object_base_coords, object_base_rotation)
        
        else:
            object_target_coords, object_target_rotation = self.brick_manager.get_next_position(self.label)
            if object_target_coords == (0, 0.5, 0.1):
                self.controller_tasks.camera_position_1x1()
                all_objects = self.camera_tasks.get_drop_off_position()
                color = ''.join([c for c in self.label if not c.isdigit() and c != 'x'])
                matching_objects = [obj for obj in all_objects if obj['label'] == color]
                best_object = max(matching_objects, key=lambda x: x['confidence'])
                coords = best_object["position_camera"]
                object_target_coords = self.tf_tasks.tf_image_to_base(coords)
                object_target_rotation = (180, 0, -45)
                
            self.controller_tasks.place_object(object_target_coords, object_target_rotation)

        self.place_to_detection()



    def on_enter_finished(self):
        rospy.loginfo("Finished!") 
        rospy.loginfo("Alle Objekte wurden verarbeitet, Process abgeschlossen")



#def main():
#    moveit_commander.roscpp_initialize([])          # nur einmal aufrufen, auch wenn mehrer klassen, deshalb zwingend an dieser stelle
#    rospy.init_node('main_task', anonymous=True)
#
#    state_machine = Process_Manager()
#    rospy.spin()


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node('main_task', anonymous=True)

    state_machine = Process_Manager()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass