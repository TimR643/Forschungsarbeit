# panda_spacemouse_teleop (ROS1)

ROS1 package to read a 3Dconnexion SpaceMouse via `pyspacemouse` and publish a safety-limited `geometry_msgs/Twist` for Panda teleoperation.

## 1) Prerequisites

- Ubuntu with ROS1 (`noetic` expected)
- Working `franka_ros` setup for your Panda
- SpaceMouse connected via USB

### System packages

```bash
sudo apt update
sudo apt install -y python3-pip libhidapi-dev
```

### Python package

```bash
python3 -m pip install --user "pyspacemouse<2.0"
```


> ROS Noetic uses Python 3.8 by default. `pyspacemouse` 2.x requires newer Python features (`dataclass(slots=...)`).
> Therefore pin `pyspacemouse<2.0` on Noetic/Python 3.8.

### Optional: udev rule for hidraw access (recommended)

```bash
cat <<'RULE' | sudo tee /etc/udev/rules.d/99-spacemouse.rules
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"
RULE

sudo usermod -aG plugdev $USER
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then log out and in again.

## 2) Build the catkin workspace

Run from your catkin workspace root (example: `~/catkin_ws`):

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 3) Verify SpaceMouse input outside ROS

```bash
python3 -m pyspacemouse --list-connected
```

If your device is listed, input should be available.

## 4) Run teleop publisher

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch panda_spacemouse_teleop spacemouse_teleop.launch
```

Published topics:
- `/spacemouse/twist` (`geometry_msgs/Twist`)
- `/spacemouse/enabled` (`std_msgs/Bool`)

## 5) Connect to your Panda controller topic

If your controller expects a different topic, remap at launch:

```bash
roslaunch panda_spacemouse_teleop spacemouse_teleop.launch \
  twist_topic:=/franka_controller/target_cartesian_velocity
```

## 6) Safety behavior

Defaults in `config/teleop.yaml` are conservative:

- `max_linear: 0.03` m/s
- `max_angular: 0.20` rad/s
- `require_deadman: true`
- `deadman_button_index: 0` (hold left button while moving)
- `start_enabled: false` (press enable button index 1 to arm)

Edit `config/teleop.yaml` for axis inversion and scales.

## 7) Quick checks

In another terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /spacemouse/enabled
rostopic echo /spacemouse/twist
```

## Notes

- This package only publishes `Twist`; your Panda motion depends on the controller consuming that topic.
- Start in a safe workspace, with E-stop available, low speeds, and collision-free conditions.


### Troubleshooting: `TypeError: dataclass() got an unexpected keyword argument "slots"`

You installed an incompatible `pyspacemouse` major version on Python 3.8. Reinstall pinned version:

```bash
python3 -m pip uninstall -y pyspacemouse
python3 -m pip install --user "pyspacemouse<2.0"
```

Then verify:

```bash
python3 -m pyspacemouse --list-connected
```
