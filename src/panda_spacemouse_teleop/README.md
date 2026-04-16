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

Ensure user scripts are on PATH (so `pyspacemouse` command is found):

```bash
export PATH="$HOME/.local/bin:$PATH"
```


> ROS Noetic uses Python 3.8 by default. `pyspacemouse` 2.x requires newer Python features (`dataclass(slots=...)`).
> Therefore pin `pyspacemouse<2.0` on Noetic/Python 3.8.


If `pyspacemouse` was already installed, force reinstall a compatible version:

```bash
python3 -m pip uninstall -y pyspacemouse
python3 -m pip install --user "pyspacemouse<2.0"
```

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
catkin_make --pkg panda_spacemouse_teleop
source devel/setup.bash
```

## 3) Verify SpaceMouse input outside ROS

```bash
~/.local/bin/pyspacemouse --list-spacemouse
```

If your device is listed, input should be available.

If `catkin_make` fails in unrelated packages (e.g. `realsense2_camera`), build only this package:

```bash
cd ~/catkin_ws
catkin_make --pkg panda_spacemouse_teleop
source devel/setup.bash
```

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



### One-command startup with Franka example velocity controller

If `franka_control.launch` is already running, you can auto-load the
`cartesian_velocity_example_controller` and start SpaceMouse teleop with one launch:

```bash
roslaunch panda_spacemouse_teleop spacemouse_franka_cartesian_velocity.launch
```

This launch sets the required controller params (`type`, `arm_id`, `joint_names`),
spawns the controller, and remaps teleop output to
`/cartesian_velocity_example_controller/command`.

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
- Set optional button indices to `-1` (disabled). Do not use `null` in ROS1 params.

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
~/.local/bin/pyspacemouse --list-spacemouse
```


### Troubleshooting: `cannot marshal None unless allow_none is enabled`

ROS1 parameter server (XMLRPC) cannot handle `null`/`None` values in launch-loaded params.
Use `-1` for disabled optional button indices (e.g. `disable_button_index: -1`).


### Troubleshooting: `No module named pyspacemouse.__main__`

For `pyspacemouse<2.0`, use the CLI executable instead of `python -m` (and use `--list-spacemouse`):

```bash
~/.local/bin/pyspacemouse --list-spacemouse
```


### Troubleshooting: `AttributeError: __enter__`

`pyspacemouse` 1.x does not expose the same context-manager API as 2.x.
This package supports both APIs now; rebuild your workspace after pulling updates:

```bash
cd ~/catkin_ws
catkin_make --pkg panda_spacemouse_teleop
source devel/setup.bash
```
