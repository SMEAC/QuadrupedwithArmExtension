# Go2 with Arm Teleoperation Extension

An Isaac Sim extension for teleoperating a **Unitree Go2** quadruped robot equipped with an **OpenManipulator-X** arm using keyboard commands and a trained RL locomotion policy.

## Features

- RL-based flat-terrain locomotion policy (trained in Isaac Lab) on a Go2 quadruped
- OpenManipulator-X arm rigidly attached to the Go2 base via a fixed joint
- Two cameras: one mounted on the arm wrist, one on the Go2 head
- RTX LiDAR with ROS2 point cloud output
- Full ROS2 bridge integration: `cmd_vel`, `joint_states`, odometry, TF, camera feeds, clock
- Keyboard teleoperation of base velocity and body pitch/roll

## Prerequisites

- **Isaac Sim** 5.1+ installed
- **Isaac Sim ROS2 Bridge extension** enabled (`isaacsim.ros2.bridge`)
- A trained Go2 policy file (`.pt` + `.yaml`) — see **Configuring the Policy** below
- **OpenManipulator-X** USD file (`open_manipulator_x.usd`) — see **Configuring the Arm** below

## Installation

There are two ways to install this extension:

### Option A: Place in Isaac Sim's user extensions directory (simplest)

```bash
mkdir -p ~/go2withArmExtension

# Copy this extension into Isaac Sim's user extension directory
cp -r ~/go2withArmExtension \
  /home/gavin/IsaacSim/source/isaacsim-user/exts/
```

> **Tip:** If your Isaac Sim is installed elsewhere, find the correct path by going to **Edit > Preferences > Extensions > Extension Search Path** in Isaac Sim. Each extension must live in its own subdirectory.

### Option B: Specify a custom extension search path

```bash
# Point Isaac Sim at your extension directory on launch
./isaac-sim.sh -v \
  --ext-folder /home/gavin/IsaacSim/source/isaacsim-user/exts/go2withArmExtension
```

## Activating the Extension

After installation, restart Isaac Sim and:

1. Open the **Extension Manager** window:
   - Menu bar: **Isaac Sim > Windows > Extensions** (or press `Shift+Ctrl+E`)
2. Search for **"Go2 with Arm Teleoperation"** in the extension list
3. Check the box to **enable** the extension
4. The sample will register in the **Examples Browser** under category **A**
   - Menu: **Isaac Sim > Windows > Examples** (or press `Shift+Ctrl+X`)
   - Find **"Go2 Arm Teleop"** in the list and click **Run**

### Alternative: Command-line activation

Start Isaac Sim with the extension enabled:

```bash
./isaac-sim.sh -v \
  --ext-folder /home/gavin/go2withArmExtension \
  --exts/go2_with_arm.enable=1
```

Or via environment variable:

```bash
export OMNI_KIT_EXTENSIONS_PATH=/home/gavin/go2withArmExtension
./isaac-sim.sh -v
```

Then enable it in the Extension Manager as described above.

## Configuring the Policy

Edit `go2_with_arm/policy/go2withpitch.py` and update the default policy paths in `__init__`:

```python
self.load_policy(
    "/path/to/your/policy.pt",       # Trained policy weights
    "/path/to/your/env.yaml",        # Policy environment config
)
```

## Configuring the Arm

Edit `go2_with_arm/sample/go2armteleop_example.py` in the `__init__` method:

```python
self.arm_usd_path = "/path/to/open_manipulator_x.usd"   # Path to your OpenManipulator-X USD
self.arm_position = np.array([1.5, 0.0, 0.0])           # Position of arm on the Go2 base
```

## Keyboard Controls

| Key / Numpad | Action          |
|-------------|-----------------|
| Numpad 8 / Up    | Move Forward    |
| Numpad 2 / Down  | Move Reverse    |
| Numpad 4 / Left  | Strafe Left     |
| Numpad 6 / Right | Strafe Right    |
| Numpad 7 / N     | Spin Counterclockwise |
| Numpad 9 / M     | Spin Clockwise      |
| A / a            | Pitch Up            |
| Z / z            | Pitch Down          |
| C / c            | Roll Left           |
| X / x            | Roll Right          |

Hold a key to increase the command velocity; release to stop. Commands accumulate while keys are held.

## Extension Structure

```
go2withArmExtension/
├── extension.toml          # Extension manifest (metadata, dependencies, modules)
├── data/
│   └── preview.png         # Preview image for the extension catalog
├── docs/
│   └── README.md           # This file
├── go2_with_arm/
│   ├── __init__.py         # Package entry point
│   ├── policy/
│   │   ├── __init__.py
│   │   └── go2withpitch.py # Go2 RL policy controller
│   └── sample/
│       ├── __init__.py
│       ├── go2armteleop_example.py   # Interactive sample (scene setup, cameras, LiDAR, graphs)
│       └── go2armteleop_extension.py # Extension wrapper (registration with Examples Browser)
└── LICENSE
```

## License

Apache 2.0 — see [LICENSE](../LICENSE) file.
