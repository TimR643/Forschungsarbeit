# Package for the Main Task
This package is designed for the main task in the ROS Lab: sorting various objects, big and small bricks, using camera detection and the Franka Emika Panda robot arm.

## 📌 Status
### 🔧 Core Modules (Latest Version)
- `main_yolo.py`  
  → State machine that manages the overall process flow.
- `cv_yolo.py`  
  → Handles all camera-related tasks, including YOLO detection.
- `tf_calculate.py`  
  → Transforms image coordinates to robot base frame coordinates.
- `brick_manager.py`  
  → Manages placement positions for the bricks.
- `controller.py`  
  → Class to control all robot movements (e.g., picking, placing, motion control).

---

### 🛠️ Tools & Calibration
- `create_images.py`  
  → Captures images for training the YOLO model.
- `tf_camera_calibration.py`  
  → Performs hand-eye calibration between the robot and the camera.

---
### 🚀 Launch Files
- `start.launch`  
  → Launches the system in the **Gazebo simulation** environment.
- `start3.launch`  
  → Launches the system on the **real robot**.
---


## Todo to start
1. Download or clone this complete folder `main_task` into your `catkin_ws/src`.
2. Build the workspace with:
```bash
catkin_make
```
3. Start the launch file `launch/start3.launch` with the command:
```bash
roslaunch main_task start3.launch
```
It will start all necessary packages

### Optional: Start with SpaceMouse teleop integrated
If `panda_spacemouse_teleop` is in the same workspace, you can start the existing
real-robot launch and automatically bring up SpaceMouse teleop + controller spawner:
```bash
roslaunch main_task start3.launch enable_spacemouse:=true
```
The SpaceMouse output is remapped to `/cartesian_velocity_example_controller/command`.

If your RealSense node crashes during teleop bring-up, you can temporarily disable
camera startup while validating robot control:
```bash
roslaunch main_task start3.launch enable_spacemouse:=true enable_camera:=false
```
4. Wait until everything is started.
5. Make sure that `main_yolo.py` is executable:
```bash
chmod +x ~/catkin_ws/src/main_task/scripts/main.py
```
6. Start the Task process with:
```bash
rosrun main_task main.py
```

## Hand-eye calibration
1. Start the launch file `launch/start3.launch` with the command:
```bash
roslaunch main_task start3.launch
```
2. Run the hand-eye-calibration script
```bash
roslaunch main_task tf_camera_callibration.py
```
3. The Final calculated 4x4 Matrix will be printed on the terminal
4. To use the new matrix you can copy it to the tf_calulate.py script
