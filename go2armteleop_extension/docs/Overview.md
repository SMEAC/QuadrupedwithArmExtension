# Go2 with Arm Teleoperation Extension

An Isaac Sim 5.1+ extension for teleoperating a **Unitree Go2** quadruped robot with an **OpenManipulator-X** arm using keyboard commands and a trained RL locomotion policy.

## Features

- **RL-based locomotion** — flat-terrain policy trained in Isaac Lab, running inference every decimation step
- **OpenManipulator-X arm** — rigidly attached to the Go2 base via a fixed joint
- **Dual cameras** — arm wrist camera and Go2 head camera, streaming RGB via ROS2
- **RTX LiDAR** — 3D point cloud output via ROS2
- **Full ROS2 bridge** — `cmd_vel`, `joint_states`, odometry, TF, camera feeds, simulation clock
- **Keyboard teleoperation** — base velocity (forward/back/strafe/spin) and body pitch/roll control
- **ROS2 `cmd_vel` fusion** — keyboard commands merge with `/cmd_vel` input so external systems can also drive the robot

## Prerequisites

- Isaac Sim 5.1+
- `isaacsim.ros2.bridge` extension enabled
- A trained Go2 policy file (`.pt` weights + `.yaml` config)
- `open_manipulator_x.usd` asset

## Installation

### Option A: Isaac Sim user extensions directory

```bash
cp -r ~/go2withArmExtension \
  /home/gavin/IsaacSim/source/isaacsim-user/exts/
```

### Option B: Custom search path

In Isaac Sim go to **Edit → Preferences → Extensions → Extension Search Path** and add the parent directory containing `go2armteleop_extension/`. Always restart Isaac Sim after changing search paths.

## Activation

1. Restart Isaac Sim.
2. Open **Isaac Sim → Windows → Extensions** (`Shift+Ctrl+E`).
3. Search for **"Go2 with Arm Teleoperation"** and enable it.
4. Open **Isaac Sim → Windows → Examples** (`Shift+Ctrl+X`) and find **"Go2 Arm Teleop"** in category **A** → click **Run**.

**CLI launch:**

```bash
./isaac-sim.sh -v --ext-folder /home/gavin/go2withArmExtension --exts/go2_with_arm.enable=1
```

## Configuration

### Policy weights

In `go2armteleop_extension/policy/go2withpitch.py` (`__init__`, ~line 64):

```python
policy_path = "/path/to/your/policy.pt"
policy_config_path = "/path/to/your/env.yaml"
```

### Arm USD and mount position

In `go2armteleop_extension/go2armteleop_example.py` (`Go2ArmExample.__init__`, ~line 98):

```python
self.arm_usd_path = "/path/to/open_manipulator_x.usd"
self.arm_position = np.array([1.5, 0.0, 0.0])   # position on Go2 base
```

## Keyboard Controls

| Key / Numpad | Action          |
|--------------|-----------------|
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

Hold a key to accumulate command velocity; release to stop. Commands accumulate while keys are held.

## Architecture

```
go2armteleop_extension/
+-- config/
|   +-- extension.toml         # Isaac Sim extension manifest
+-- data/
|   +-- preview.png            # Extension catalog preview
+-- __init__.py                # Package entry point
+-- go2armteleop_example.py    # Interactive sample (scene, cameras, LiDAR, graphs)
+-- go2armteleop_extension.py  # Extension wrapper (Examples Browser registration)
+-- policy/
    +-- __init__.py
    +-- go2withpitch.py        # Go2withPitchFlatTerrainPolicy (RL policy)
```

### Data flow

1. Keyboard press → `_base_command` accumulates a 5-dim vector `[v_x, v_y, yaw_rate, pitch_rate, roll_rate]`.
2. ROS2 `cmd_vel` → read from the `/CMDVELGraph` OmniGraph.
3. **Merge** → `keyboard + cmd_vel` → passed to `go2.forward()`.
4. `go2.forward()` → every `_decimation` steps: observation → policy inference → `ArticulationAction` (12 joint position targets, scaled by 0.25) → applied to Go2 articulation.

### OmniGraphs

| Graph | Purpose |
|-------|---------|
| `/CMDVELGraph` | Subscribe to `cmd_vel` (`geometry_msgs/Twist`) |
| `/ArmGraph` | Subscribe to `joint_states`, drive arm articulation |
| `/LIDARGraph3D` | RTX LiDAR point cloud → ROS2 |
| `/OdometryGraph` | Publish odometry, TF trees, simulation time |
| `/CameraArmGraph` | Arm wrist camera → ROS2 `wrist_camera` (RGB) |
| `/CameraQuadrupedGraph` | Go2 head camera → ROS2 `rgb` (RGB) |
| `/SimTimeGraph` | Publish simulation clock to ROS2 |

### Policy internals

- **Observation** — 50-dim vector: base linear/ang velocity, gravity direction, command, joint positions (relative to default), joint velocities, previous action.
- **Action** — 12-dim joint position targets for Go2 leg joints, scaled by `0.25`.

## License

Apache 2.0.
