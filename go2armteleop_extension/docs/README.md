# Go2 with Arm Teleoperation Extension

An Isaac Sim 5.1+ extension for teleoperating a **Unitree Go2** quadruped robot equipped with an **OpenManipulator-X** arm using keyboard commands and a trained RL locomotion policy.

![Extension Preview](data/preview.png)

## Features

- RL-based flat-terrain locomotion policy (trained in Isaac Lab) on a Go2 quadruped
- OpenManipulator-X arm rigidly attached to the Go2 base via a fixed joint
- Tennis ball asset (bundled USD) placed in the scene
- Autopilot ball-following mode with ``[-180, 180]``-wrapped yaw error and smoothstep roll blending
- Custom gripper asset (``Gripper_Ball_v3.usda``) mounted on both arm gripper links
- Dual cameras: arm wrist camera and Go2 head camera, streaming RGB via ROS2
- RTX LiDAR with ROS2 point cloud output
- Full ROS2 bridge integration: ``cmd_vel``, ``joint_states``, odometry, TF, camera feeds, clock
- Keyboard teleoperation of base velocity and body pitch/roll
- Modular architecture — scene, cameras, omnigraphs, and policy each in separate modules
- Refactored policy with dedicated controllers, core prims, and robot classes

## Prerequisites

- **Isaac Sim** 5.1+ installed
- **Isaac Sim ROS2 Bridge extension** enabled (`isaacsim.ros2.bridge`)
- **OpenManipulator-X** USD file (`open_manipulator_x.usd`) — see **Configuring the Arm** below

> **Policy weights** are bundled with the extension in `policy/data/policy/`. To use a custom policy, see **Configuring the Policy** below.

## Installation

There are two ways to install this extension:

### Option A: Place in Isaac Sim's user extensions directory (simplest)

```bash
mkdir -p ~/QuadrupedwithArmExtension

# Copy this extension into Isaac Sim's user extension directory
cp -r ~/QuadrupedwithArmExtension \
  ~/IsaacSim/source/isaacsim-user/exts/
```

> **Tip:** If your Isaac Sim is installed elsewhere, find the correct path by going to **Edit > Preferences > Extensions > Extension Search Path** in Isaac Sim. Each extension must live in its own subdirectory.

### Option B: Specify a custom extension search path

```bash
# Point Isaac Sim at your extension directory on launch
./isaac-sim.sh -v \
  --ext-folder ~/IsaacSim/source/isaacsim-user/exts/QuadrupedwithArmExtension
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
  --ext-folder /home/gavin/QuadrupedwithArmExtension \
  --exts/go2armteleop_extension.enable=1
```

Or via environment variable:

```bash
export OMNI_KIT_EXTENSIONS_PATH=~/QuadrupedwithArmExtension
./isaac-sim.sh -v
```

Then enable it in the Extension Manager as described above.

## Configuration

### Policy weights

Policy weights (`.pt`) and environment config (`.yaml`) are bundled with the extension in the `policy/data/policy/` directory. To use a custom policy, set the paths in `go2armteleop.py` (`Go2ArmExample.__init__`):

```python
self.policy_path = "/path/to/your/policy.pt"       # Trained policy weights
self.policy_config_path = "/path/to/your/env.yaml"  # Policy environment config
```

### Arm USD and mount position

Edit `go2armteleop.py` (`Go2ArmExample.__init__`, ~line 110):

```python
self.arm_usd_path = "/path/to/open_manipulator_x.usda"   # Path to your OpenManipulator-X USD
self.arm_position = np.array([0.2, 0.0, 0.07])           # Position of arm on the Go2 base
```

### Ball follow autopilot parameters

In `Go2ArmExample.__init__`:

```python
self._ball_follow_dist = 0.6      # stop distance (m)
self._ball_follow_start = 1.5     # activate at this distance (m)
self._yaw_gain = 0.01             # proportional yaw rate gain
self._vx_gain = 0.8               # approach velocity gain
self._roll_gain = 1.0             # lean toward ball gain
self._max_yaw_rate = 1.5          # max yaw rate (rad/s)
self._max_vx = 0.4                # max forward velocity (m/s)
```

## Keyboard Controls

| Key / Numpad | Action          |
|------|-------|
| Numpad 8 / Up    | Move Forward    |
| Numpad 2 / Down  | Move Reverse    |
| Numpad 4 / Left  | Strafe Left     |
| Numpad 6 / Right | Strafe Right    |
| Numpad 7 / N     | Spin Counterclockwise |
| Numpad 9 / M     | Spin Clockwise      |
| A / a            | Pitch Up            |
| Z / z            | Pitch Down          |
| C / c            | Roll Left         |
| X / x            | Roll Right        |

Hold a key to increase the command velocity; release to stop. Commands accumulate while keys are held.

## Extension Structure

```
go2withArmExtension/
├── config/
│   └── extension.toml        # Extension manifest (metadata, dependencies, modules)
├── docs/
│   ├── Overview.md           # Detailed documentation
│   ├── README.md             # Mirror of Overview.md
│   └── CHANGELOG.md          # Version history
├── go2armteleop_extension/
│   ├── __init__.py           # Package entry point (exports Go2withPitchFlatTerrainPolicy, Go2ArmExample)
│   ├── go2armteleop.py       # Interactive sample (orchestration, keyboard, lifecycle, autopilot)
│   ├── go2armteleop_extension.py   # Extension wrapper (Examples Browser registration)
│   ├── ui_extension.py   # UI panel (autopilot toggle, telemetry, debug, joint inspector)
│   ├── scene.py              # Scene objects: ground, Go2 robot, arm, ball, basket, mat, cameras, LiDAR, gripper
│   ├── cameras.py            # CameraConfig dataclass, camera creation, viewport assignment
│   ├── omnigraphs.py         # ROS2 bridge OmniGraph creation
│   └── policy/
│       ├── __init__.py
│       ├── robots/
│       │   ├── __init__.py
│       │   └── go2withpitch.py   # Go2withPitchFlatTerrainPolicy (RL policy, extends PolicyControllerGo2)
│       ├── controllers/
│       │   ├── __init__.py
│       │   ├── config_loader.py    # parse_env_config, get_robot_joint_properties, etc.
│       │   └── policy_controller.py  # PolicyControllerGo2 (base controller)
│       ├── core/
│       │   ├── __init__.py
│       │   ├── articulation.py       # Articulation wrapper
│       │   ├── single_articulation.py  # SingleArticulation prim wrapper
│       │   ├── single_prim_wrapper.py  # _SinglePrimWrapper base class
│       │   └── xform_prim.py         # XFormPrim prim view class
│       └── data/
│           ├── policy/
│           │   ├── policy.pt     # Bundled Go2 locomotion policy weights
│           │   └── env.yaml      # Bundled policy environment config
│           └── preview.png       # Extension catalog preview
├── resource/
│   ├── Tennis_ball_01.usda       # Tennis ball asset
│   ├── Gripper_Ball_v3.usda      # Gripper asset (v3)
│   ├── Gripper_Ball.usda         # Gripper asset (v1)
│   ├── Gripper_Ball_v2.usda      # Gripper asset (v2)
│   ├── Gripper_Ball_v2.STL       # Gripper mesh source
│   └── openManipulator/          # Arm configs + meshes
└── LICENSE
```

## Differences from Original Isaac Lab / Isaac Sim Repository

This extension diverges from the original Isaac Lab `go2withArm` repository in several key ways:

### Policy Architecture

- **Refactored into sub-packages**: The monolithic `go2withpitch.py` has been split into a modular package:
  - `policy/controllers/` — `PolicyControllerGo2` base controller with policy loading, env config parsing, joint property resolution
  - `policy/robots/` — `Go2withPitchFlatTerrainPolicy` (extends `PolicyControllerGo2`) with 50-dim observation computation and 12-dim action application
  - `policy/core/` — Isaac Sim prim wrappers (`SingleArticulation`, `XFormPrim`, `_SinglePrimWrapper`, `Articulation`)
  - `policy/data/policy/` — nested policy weights and config (vs. flat `data/policy/`)

### Scene Objects

- **Basket**: Added `create_basket()` — loads a KLT bin from NVIDIA OCP URL at `[2.0, -1.0, 0.5]`
- **Mat**: Added `create_mat()` — loads a plane from NVIDIA OCP URL with configurable OmniPBR material color
- **Ball position**: Changed from `[0.0, 1.0, 0.5]` to `[2.0, 1.0, 1.5]`
- **Arm position**: Changed from `[0.2, 0.0, 0.07]` to `[1.5, 0.0, 0.0]` (arm USD mount point); fixed joint offset remains `[0.2, 0.0, 0.07]`
- **Go2 USD**: Now resolved via `isaacsim.storage.native.get_assets_root_path()` (NVIDIA OCP URL) instead of hardcoded local path
- **Gripper**: Uses `Gripper_Ball_v3.usda` (was `Gripper_Ball.usda`); wrapped with `RigidPrim` for physics

### Autopilot

- **Yaw error wrapping**: Added `[-180, 180]` modulo wrap to avoid ±180° seam reversal
- **Yaw command scaling**: Doubled yaw output (`2 * Rz`) for more responsive turning
- **Roll blending**: Added smoothstep-based roll blending that ramps as the robot approaches the ball, with Z-axis sign awareness
- **Command vector**: Autopilot publishes `[Vx, 0.0, 2*Rz, -Rx, 0.0]` (was `[Vx, 0.0, Rz, -Rx, 0.0]`)

### Camera System

- **CameraConfig dataclass**: Camera configs now defined as structured `CameraConfig` objects in `cameras.py` (was inline dicts)
- **Viewport assignment**: Cameras now explicitly assigned to both Viewport and Viewport 2 windows
- **Resolution**: Both cameras set to 512×512 (was default)

### Fixed Joint Transforms

- **Quaternion calculation**: Switched from manual `scipy.spatial.transform.Rotation` quaternion extraction to consistent `Gf.Quatf` conversion

### UI Panel

- **Joint inspector**: Added "Show Config" button that displays joint patterns from `UNITREE_GO2_CFG`
- **Hip finder**: Added "Find Hips" button that traverses the Go2 USD to find hip joint indices
- **Telemetry**: Live-updated labels for ball offset, distance XY, robot/target/yaw error, merged command

## License

Apache 2.0 — see [LICENSE](LICENSE) file.
