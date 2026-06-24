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
    ├── data/
    │   ├── policy/
    │   │   ├── policy.pt                   ← Bundled Go2 locomotion policy
    │   │   └── env.yaml                    ← Policy env config
    │   └── preview.png                     ← Extension catalog preview
    ├── docs/
    │   ├── Overview.md                     ← Main documentation
    │   ├── README.md                       ← Mirror of Overview.md
    │   └── CHANGELOG.md
    ├── policy/
    │   ├── __init__.py                     ← Package entry
    │   └── go2withpitch.py                 ← RL policy
    ├── resource/
    │   ├── Tennis_ball_01.usda             ← Tennis ball asset
    │   ├── Gripper_Ball*.usda              ← Gripper variants
    │   └── openManipulator/                ← Arm configs + meshes
    ├── __init__.py
    ├── go2armteleop_extension.py           ← [[python.module]] go2armteleop_extension
    │                                       ← Go2ArmExampleExtension (omni.ext.IExt)
    │                                       ← Registers sample in Examples Browser (category A)
    │                                       ← Extension name: "Go2 Arm Teleop v1"
    ├── go2armteleop.py                     ← Go2ArmExample (BaseSample)
    │                                       ← setup_scene(), setup_post_load/reset()
    │                                       ← on_physics_step(): merge ROS2 + keyboard
    │                                       ← _sub_keyboard_event(): key press/release
    │                                       ← autopilot: _compute_ball_follow_command()
    │                                       ← autopilot: _publish_cmd_vel_autopilot()
    ├── ui_extension_example.py             ← UI panel (autopilot toggle, telemetry)
    ├── scene.py                            ← Scene object creators
    │                                       ← setup_scene() orchestrator
    │                                       ← create_go2_robot(), create_open_manipulator_x()
    │                                       ← create_ball(), create_gripper(), create_fixed_joint()
    │                                       ← create_rtx_lidar(), create_range_sensor()
    │                                       ← create_camera_obj()
    ├── cameras.py                          ← Camera setup and viewports
    │                                       ← CameraConfig dataclass, create_camera()
    │                                       ← setup_viewports(), CAMERA_ARM/QUADRUPED
    ├── omnigraphs.py                       ← ROS2 bridge OmniGraphs
    │                                       ← _create_cmdvel_graph()
    │                                       ← _create_cmdvel_autopilot_pub_graph()
    │                                       ← _create_arm_graph()
    │                                       ← _create_lidar_graph()
    │                                       ← _create_odometry_graph()
    │                                       ← _create_camera_arm_graph()
    │                                       ← _create_camera_quadruped_graph()
    │                                       ← _create_clock_graph()
    │                                       ← setup_omnigraphs() orchestrator
```

Key Isaac Sim configuration:

1. **Search path**: Add `/home/gavin/go2withArmExtension` in Isaac Sim via **Edit → Preferences → Extensions → Extension Search Path**. This points to the **parent** directory (not config/).
2. **Always restart Isaac Sim** after adding/removing search paths — Isaac Sim does not hot-reload extension search paths at runtime.
3. After restarting, the extension will appear in the Extension Manager. Enable it, then open **Isaac Sim → Windows → Examples (Shift+Ctrl+X)** to find **"Go2 Arm Teleop v1"** in category **"A"**.

## Key Configuration (hardcoded paths to update for your environment)

- **Policy weights**: `go2armteleop_extension/policy/go2withpitch.py` — default `policy_path` and `policy_config_path` resolve relative to the extension's `data/policy/` directory. Override by passing arguments to `Go2withPitchFlatTerrainPolicy.__init__()`.
- **Go2 USD**: `go2armteleop_extension/scene.py:467` — Go2 robot USD defaults to NVIDIA OCP URL (can override via `usd_path` argument to `create_go2_robot()`).
- **Arm USD**: `go2armteleop_extension/scene.py:477` — `arm_usd_path` defaults to `/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd` (pass as argument to `setup_scene()` to override).
- **Arm position on Go2 base**: `go2armteleop_extension/scene.py:479` — `arm_position` defaults to `[0.2, 0.0, 0.07]` (pass as argument to `setup_scene()` to override).
- **Arm fixed joint**: `go2armteleop_extension/scene.py:483-486` — fixed joint connects `/World/Go2/base` → `/World/open_manipulator_x/world` with offset `[0.2, 0.0, 0.07]`, no rotation.
- **Tennis ball**: `go2armteleop_extension/scene.py:501` — `create_ball()` resolves relative path `resource/Tennis_ball_01.usda`; position defaults to `[2.0, 1.0, 1.5]`.
- **Ball follow autopilot**: `go2armteleop_extension/go2armteleop.py:68-75` — parameters `_ball_follow_dist` (0.6 m), `_ball_follow_start` (1.5 m), `_yaw_gain` (0.01), `_vx_gain` (0.8), `_roll_gain` (1.0), `_max_yaw_rate` (1.5), `_max_vx` (0.4).
- **Gripper**: `go2armteleop_extension/scene.py:488-499` — two gripper prims (`/World/gripper_left`, `/World/gripper_right`) use `resource/Gripper_Ball_v3.usda`, scaled to 1.5, each connected to the arm gripper links via fixed joints.

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
  scene.py                       # Scene object creation (ground, Go2 robot, arm, tennis ball, LiDAR, cameras)
                                # - setup_scene() orchestrates all object creators
                                # - create_go2_robot(), create_open_manipulator_x(),
                                #   create_ball(), create_gripper(), create_fixed_joint(), create_rtx_lidar()
       #   create_range_sensor(), create_camera_obj()
  cameras.py                     # Camera setup and viewport assignment
                                # - CameraConfig dataclass
                                # - create_camera(), setup_viewports()
                                # - CAMERA_ARM, CAMERA_QUADRUPED constants
  omnigraphs.py                  # ROS2 bridge OmniGraph creation
                                # - _create_cmdvel_graph()
                                # - _create_cmdvel_autopilot_pub_graph()
                                # - _create_arm_graph()
                                # - _create_lidar_graph()
                                # - _create_odometry_graph()
                                # - _create_camera_arm_graph()
                                # - _create_camera_quadruped_graph()
                                # - _create_clock_graph()
                                # - setup_omnigraphs() orchestrates all graphs
  ui_extension_example.py      # UI panel: autopilot toggle, telemetry (ball offset,
                               # yaw/error, command vector), debug buttons
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

### Autopilot (ball-following)

When `autopilot_enabled` is True (toggled in the UI panel):

1. `_compute_ball_follow_command()` reads ball/robot transforms from USD stage
2. Computes yaw_error with `[-180, 180]` wrap to avoid ±180° seam reversal
3. Publishes `[Vx, 0.0, Rz, -Rx, 0.0]` on `/CMDVELAutopilotGraph` (impulse-triggered ROS2 publisher)
4. Merged with keyboard in `on_physics_step()` via `_autopilot_command`

### OmniGraph nodes summary

| Graph | Purpose |
|-------|-------|
| `/CMDVELGraph` | Subscribe to `cmd_vel` (geometry_msgs/Twist) |
| `/CMDVELAutopilotGraph` | Publish autopilot commands (impulse-triggered) |
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
- To modify the UI panel: edit `go2armteleop_extension/ui_extension_example.py` (autopilot toggle, telemetry labels)
- To modify autopilot ball-following: edit `go2armteleop_extension/go2armteleop.py` (`_compute_ball_follow_command()`, `_publish_cmd_vel_autopilot()`)
