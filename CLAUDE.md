# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

An Isaac Sim (Omniverse) extension for teleoperating a **Unitree Go2** quadruped robot with an **OpenManipulator-X** arm using a trained RL locomotion policy and keyboard input. Runs only inside Isaac Sim 5.1+ — no standalone build, test, or lint system.

## Isaac Sim Extension Structure (important)

Isaac Sim discovers extensions by scanning subdirectories for `extension.toml`, not at the immediate root of the extension search path. This directory layout must be preserved:

```
/home/gavin/go2withArmExtension/            ← Isaac Sim search path
├── config/
│   └── extension.toml                      ← Isaac Sim discovers this
├── go2_with_arm/                           ← [[python.module]] go2_with_arm
├── data/
├── LICENSE
└── README.md
```

Key Isaac Sim configuration:

1. **Search path**: Add `/home/gavin/go2withArmExtension` in Isaac Sim via **Edit → Preferences → Extensions → Extension Search Path**. This points to the **parent** directory (not config/).
2. **Always restart Isaac Sim** after adding/removing search paths — Isaac Sim does not hot-reload extension search paths at runtime.
3. After restarting, the extension will appear in the Extension Manager. Enable it, then open **Isaac Sim → Windows → Examples (Shift+Ctrl+X)** to find "Go2 Arm Teleop" in category **"A"**.

## Key Configuration (hardcoded paths to update for your environment)

- **Policy weights**: `go2_with_arm/policy/go2withpitch.py:64-67` — default `policy_path` and `policy_config_path` point to `/home/gavin/isaacSimData/go2Policy/...`
- **Arm USD**: `go2_with_arm/sample/go2armteleop_example.py:98` — `arm_usd_path` points to `/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd`
- **Arm position on Go2 base**: `go2_with_arm/sample/go2armteleop_example.py:100` — `arm_position = [1.5, 0.0, 0.0]`

## Architecture

```
go2_with_arm/
  __init__.py               # Package entry: exports Go2withPitchFlatTerrainPolicy, Go2ArmExample
  policy/
    go2withpitch.py         # Go2withPitchFlatTerrainPolicy — RL policy controller
                            # Extends isaacsim.robot.policy.examples.controllers.PolicyController
                            # - _compute_observation(): builds 50-dim obs vector (vel, ang vel, gravity, command, joint states, prev action)
                            # - forward(dt, command): runs policy inference, applies action as joint position targets (scaled by 0.25)
                            # Command vector: [v_x, v_y, yaw_rate, pitch_rate, roll_rate]
  sample/
    go2armteleop_extension.py   # Go2ArmExampleExtension — omni.ext.IExt wrapper
                                # Registers sample in Isaac Sim Examples Browser (category A)
                                # Matches the pattern of isaacsim.examples.interactive/quadruped/
    go2armteleop_example.py   # Go2ArmExample — the interactive sample (extends BaseSample)
                            # - setup_scene(): creates world (ground, Go2 robot, OpenManipulator-X arm, fixed joint, 2 cameras, RTX LiDAR, OmniGraphs)
                            # - setup_post_load/setup_post_reset(): subscribes to keyboard, registers physics callback
                            # - on_physics_step(): merges ROS2 cmd_vel with keyboard input, forwards to policy
                            # - _sub_keyboard_event(): accumulates commands on key-press, subtracts on key-release
                            # OmniGraphs created: /CMDVELGraph, /ArmGraph, /LIDARGraph3D, /OdometryGraph, /CameraArmGraph, /CameraQuadrupedGraph, /SimTimeGraph
                            # All graphs wire ROS2 bridge nodes (publishers/subscribers for cmd_vel, joint_states, odometry, TF, cameras, clock, LiDAR)
```

### Data flow

1. Keyboard press → `_base_command` accumulates 5-dim vector
2. ROS2 `cmd_vel` → read from `/CMDVELGraph` outputs
3. **Merge** → `_merged_command = keyboard + ros2` → passed to `go2.forward()`
4. `go2.forward()` → policy inference (every `_decimation` steps) → `ArticulationAction` with joint position targets → applied to Go2 articulation

### OmniGraph nodes summary

| Graph | Purpose |
|-------|-------|
| `/CMDVELGraph` | Subscribe to `cmd_vel` (geometry_msgs/Twist) |
| `/ArmGraph` | Subscribe to `joint_states`, drive arm articulation |
| `/LIDARGraph3D` | RTX LiDAR point cloud output via ROS2 |
| `/OdometryGraph` | Publish odometry, TF trees (raw + standard), simulation time |
| `/CameraArmGraph` | Arm wrist camera → ROS2 `wrist_camera` (RGB) |
| `/CameraQuadrupedGraph` | Go2 head camera → ROS2 `rgb` (RGB) |
| `/SimTimeGraph` | Publish simulation clock to ROS2 |

## Developing

- **Run**: Start Isaac Sim with the extension folder, enable in Extension Manager, run "Go2 Arm Teleop" from Examples Browser (Shift+Ctrl+X)
- **CLI launch**: `./isaac-sim.sh -v --ext-folder /home/gavin/go2withArmExtension --exts/go2_with_arm.enable=1`
- **No tests or linting** — validation is through simulation run
- To modify the policy: edit `go2withpitch.py` (observation space is 50-dim, action is 12-dim joint positions)
- To modify the sample scene: edit `go2armteleop_example.py` `setup_scene()` method
- To add new ROS2 topics: create new OmniGraphs in `setup_scene()` using `isaacsim.ros2.bridge` node types
