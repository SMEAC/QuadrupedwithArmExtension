# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

An Isaac Sim (Omniverse) extension for teleoperating a **Unitree Go2** quadruped robot with an **OpenManipulator-X** arm using a trained RL locomotion policy and keyboard input. Runs only inside Isaac Sim 5.1+ — no standalone build, test, or lint system.

## Isaac Sim Extension Structure (important)

Isaac Sim discovers extensions by scanning subdirectories for `extension.toml`, not at the immediate root of the extension search path. This directory layout must be preserved:

```
/home/gavin/go2withArmExtension/            ← Isaac Sim search path
└── go2armteleop_extension/
    ├── config/
    │   └── extension.toml                  ← Isaac Sim discovers this
    ├── go2armteleop_extension.py           ← [[python.module]] go2armteleop_extension
    ├── go2armteleop.py                     ← Go2ArmExample sample
    ├── scene.py                            ← Scene object creators
    ├── cameras.py                          ← Camera setup and viewports
    ├── omnigraphs.py                       ← ROS2 bridge OmniGraphs
    ├── policy/                             ← RL policy
    │   └── go2withpitch.py
    ├── data/
    │   └── policy/
    │       ├── policy.pt
    │       └── env.yaml
    ├── docs/
    ├── resource/
    └── __init__.py
```

Key Isaac Sim configuration:

1. **Search path**: Add `/home/gavin/go2withArmExtension` in Isaac Sim via **Edit → Preferences → Extensions → Extension Search Path**. This points to the **parent** directory (not config/).
2. **Always restart Isaac Sim** after adding/removing search paths — Isaac Sim does not hot-reload extension search paths at runtime.
3. After restarting, the extension will appear in the Extension Manager. Enable it, then open **Isaac Sim → Windows → Examples (Shift+Ctrl+X)** to find **"Go2 Arm Teleop v1"** in category **"A"**.

## Key Configuration (hardcoded paths to update for your environment)

- **Policy weights**: `go2armteleop_extension/policy/go2withpitch.py:72-77` — default `policy_path` and `policy_config_path` resolve relative to the extension's `data/policy/` directory. Override by passing arguments to `Go2withPitchFlatTerrainPolicy.__init__()`.
- **Arm USD**: `go2armteleop_extension/scene.py:330` — `arm_usd_path` defaults to `/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd` (pass as argument to `setup_scene()` to override).
- **Arm position on Go2 base**: `go2armteleop_extension/scene.py:332` — `arm_position` defaults to `[1.5, 0.0, 0.0]` (pass as argument to `setup_scene()` to override).

## Architecture

```
go2armteleop_extension/
  __init__.py                    # Package entry: exports Go2withPitchFlatTerrainPolicy, Go2ArmExample
  go2armteleop_extension.py      # Go2ArmExampleExtension — omni.ext.IExt wrapper
                                # Registers sample in Isaac Sim Examples Browser (category A)
                                # Extension name: "Go2 Arm Teleop v1"
  go2armteleop.py                # Go2ArmExample — the interactive sample (extends BaseSample)
                                # - setup_scene(): delegates to scene.py, cameras.py, omnigraphs.py
                                # - setup_post_load/setup_post_reset(): subscribes to keyboard,
                                #   registers physics callback
                                # - on_physics_step(): merges ROS2 cmd_vel with keyboard input,
                                #   forwards to policy
                                # - _sub_keyboard_event(): accumulates commands on key-press,
                                #   subtracts on key-release
  scene.py                       # Scene object creation (ground, Go2 robot, arm, LiDAR, cameras)
                                # - setup_scene() orchestrates all object creators
                                # - create_go2_robot(), create_open_manipulator_x(),
                                #   create_fixed_joint(), create_rtx_lidar(), create_camera_obj()
  cameras.py                     # Camera setup and viewport assignment
                                # - CameraConfig dataclass
                                # - create_camera(), setup_viewports()
                                # - CAMERA_ARM, CAMERA_QUADRUPED constants
  omnigraphs.py                  # ROS2 bridge OmniGraph creation
                                # - _create_cmdvel_graph()
                                # - _create_arm_graph()
                                # - _create_lidar_graph()
                                # - _create_odometry_graph()
                                # - _create_camera_arm_graph()
                                # - _create_camera_quadruped_graph()
                                # - _create_clock_graph()
                                # - setup_omnigraphs() orchestrates all graphs
  policy/
    __init__.py                  # Policy package entry
    go2withpitch.py              # Go2withPitchFlatTerrainPolicy — RL policy controller
                                # - _compute_observation(): builds 50-dim obs vector
                                #   (vel, ang vel, gravity, command, joint states, prev action)
                                # - forward(dt, command): runs policy inference, applies action as
                                #   joint position targets (scaled by 0.25)
                                # Command vector: [v_x, v_y, yaw_rate, pitch_rate, roll_rate]
                                # - Policy weights default to data/policy/policy.pt (bundled)
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

- **Run**: Start Isaac Sim with the extension folder, enable in Extension Manager, run "Go2 Arm Teleop v1" from Examples Browser (Shift+Ctrl+X)
- **CLI launch**: `./isaac-sim.sh -v --ext-folder /home/gavin/go2withArmExtension --exts/go2armteleop_extension.enable=1`
- **No tests or linting** — validation is through simulation run
- To modify the policy: edit `go2armteleop_extension/policy/go2withpitch.py` (observation space is 50-dim, action is 12-dim joint positions)
- To modify the sample scene: edit `go2armteleop_extension/scene.py` (`setup_scene()` function)
- To modify camera setup: edit `go2armteleop_extension/cameras.py` (`CameraConfig` dataclass and `setup_viewports()`)
- To modify OmniGraphs: edit `go2armteleop_extension/omnigraphs.py` (`setup_omnigraphs()` function)
- To add new ROS2 topics: create new OmniGraphs in `setup_omnigraphs()` using `isaacsim.ros2.bridge` node types
