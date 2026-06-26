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
    │   ├── robots/
    │   │   ├── __init__.py
    │   │   └── go2withpitch.py             ← Go2withPitchFlatTerrainPolicy (RL policy)
    │   ├── controllers/
    │   │   ├── __init__.py
    │   │   ├── config_loader.py            ← parse_env_config, get_robot_joint_properties, etc.
    │   │   └── policy_controller.py        ← PolicyControllerGo2 (base controller)
    │   ├── core/
    │   │   ├── __init__.py
    │   │   ├── articulation.py             ← Articulation wrapper
    │   │   ├── single_articulation.py      ← SingleArticulation prim wrapper
    │   │   ├── single_prim_wrapper.py      ← _SinglePrimWrapper base class
    │   │   └── xform_prim.py               ← XFormPrim prim view class
    │   └── data/
    │       ├── policy/
    │       │   ├── policy.pt               ← Bundled policy weights
    │       │   └── env.yaml                ← Bundled policy config
    │       └── preview.png                 ← Extension catalog preview
    ├── resource/
    │   ├── Tennis_ball_01.usda             ← Tennis ball asset
    │   ├── Gripper_Ball_v3.usda            ← Gripper asset (v3)
    │   ├── Gripper_Ball.usda               ← Gripper asset (v1)
    │   ├── Gripper_Ball_v2.usda            ← Gripper asset (v2)
    │   ├── Gripper_Ball_v2.STL             ← Gripper mesh source
    │   ├── gripperTest.usd                 ← Gripper test asset
    │   └── openManipulator/                ← Arm configs + meshes
    ├── __init__.py
    ├── go2armteleop_extension.py           ← [[python.module]] go2armteleop_extension
    │                                       ← Go2ArmExampleExtension (omni.ext.IExt)
    │                                       ← Registers sample in Examples Browser (category A)
    │                                       ← Extension name: "Go2 Arm Teleop v1"
    ├── go2armteleop.py                     ← Go2ArmExample (BaseSample)
    │                                       ← setup_scene(): delegates to scene.py, cameras.py, omnigraphs.py
    │                                       ← setup_post_load/reset(): subscribes to keyboard,
    │                                         registers physics callback
    │                                       ← on_physics_step(): merge ROS2 + keyboard + autopilot
    │                                       ← _sub_keyboard_event(): key press/release
    │                                       ← autopilot: _compute_ball_follow_command()
    │                                         — yaw_error wrapped to [-180, 180]
    │                                         — yaw command scaled 2*Rz
    │                                         — smoothstep roll blending
    │                                       ← autopilot: _publish_cmd_vel_autopilot()
    ├── ui_extension.py             ← UI panel (autopilot toggle, telemetry,
    │                                       ←   joint inspector, hip finder)
    ├── scene.py                            ← Scene object creators
    │                                       ← setup_scene() orchestrator
    │                                       ← create_go2_robot(), create_open_manipulator_x()
    │                                       ← create_ball(), create_gripper(), create_basket()
    │                                       ← create_mat(), create_fixed_joint()
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

- **Policy weights**: `go2armteleop_extension/policy/robots/go2withpitch.py` — default `policy_path` and `policy_config_path` resolve relative to the extension's `policy/data/policy/` directory. Override by passing arguments to `Go2withPitchFlatTerrainPolicy.__init__()`.
- **Go2 USD**: `go2armteleop_extension/policy/robots/go2withpitch.py:66-67` — Go2 robot USD defaults to Isaac Sim asset root (`get_assets_root_path() + "/Isaac/Robots/Unitree/Go2/go2.usd"`). Can override via `usd_path` argument.
- **Arm USD**: `go2armteleop_extension/scene.py:619` — `arm_usd_path` defaults to `/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd` (pass as argument to `setup_scene()` to override).
- **Arm position on Go2 base**: `go2armteleop_extension/scene.py:621` — `arm_position` defaults to `[1.5, 0.0, 0.0]` (pass as argument to `setup_scene()` to override).
- **Arm fixed joint**: `go2armteleop_extension/scene.py:625-628` — fixed joint connects `/World/Go2/base` → `/World/open_manipulator_x/world` with offset `[0.2, 0.0, 0.07]`, no rotation.
- **Tennis ball**: `go2armteleop_extension/scene.py:643` — `create_ball()` resolves relative path `resource/Tennis_ball_01.usda`; position defaults to `[2.0, 1.0, 1.5]`.
- **Basket**: `go2armteleop_extension/scene.py:646` — `create_basket()` loads from NVIDIA OCP URL (`.../Props/KLT_Bin/small_KLT.usd`); position defaults to `[2.0, -1.0, 0.5]`.
- **Ball follow autopilot**: `go2armteleop_extension/go2armteleop.py:73-79` — parameters `_ball_follow_dist` (0.6 m), `_ball_follow_start` (1.5 m), `_yaw_gain` (0.01), `_vx_gain` (0.8), `_roll_gain` (1.0), `_max_yaw_rate` (1.5), `_max_vx` (0.4). Yaw error wrapped to `[-180, 180]`; yaw command doubled (`2*Rz`); roll uses smoothstep blending.
- **Gripper**: `go2armteleop_extension/scene.py:630-631` — two gripper prims (`/World/gripper_left`, `/World/gripper_right`) use `resource/Gripper_Ball_v3.usda`, scaled to 1.5, each connected to the arm gripper links via fixed joints with scipy-based quaternion transform.

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
                               # - on_physics_step(): merges ROS2 cmd_vel + keyboard + autopilot
                               # - _sub_keyboard_event(): accumulates commands on key-press,
                               #   subtracts on key-release
                               # - _compute_ball_follow_command(): yaw-wrapped, smoothstep roll, 2*Rz
  scene.py                       # Scene object creation (ground, Go2, arm, ball, basket, mat,
                               #   LiDAR, cameras, gripper, fixed joints)
                               # - setup_scene() orchestrates all object creators
                               # - create_go2_robot(), create_open_manipulator_x(),
                               #   create_ball(), create_gripper(), create_basket(), create_mat()
       #   create_fixed_joint(), create_rtx_lidar(), create_range_sensor()
       #   create_camera_obj()
  cameras.py                     # Camera setup and viewport assignment
                               # - CameraConfig dataclass
                               # - create_camera(), setup_viewports()
                               # - CAMERA_ARM, CAMERA_QUADRUPED constants (512x512)
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
  ui_extension.py      # UI panel: autopilot toggle, telemetry (ball offset,
                               #   dist XY, robot/target/yaw, command), joint inspector, debug
  policy/
    __init__.py                  # Policy package entry
    robots/
      __init__.py
      go2withpitch.py            # Go2withPitchFlatTerrainPolicy — RL policy controller
                               # - extends PolicyControllerGo2
                               # - _compute_observation(): builds 50-dim obs vector
                               #   (vel, ang vel, gravity, command, joint states, prev action)
                               # - forward(dt, command): runs policy inference, applies action as
                               #   joint position targets (scaled by 0.25)
                               # Command vector: [v_x, v_y, yaw_rate, pitch_rate, roll_rate]
    controllers/
      __init__.py
      config_loader.py           # parse_env_config, get_robot_joint_properties, etc.
      policy_controller.py       # PolicyControllerGo2 (base controller)
                               # - load_policy(): loads Torch JIT model + env config
                               # - initialize(): sets up articulation, gains, limits
                               # - _compute_action(): runs policy inference
    core/
      __init__.py
      articulation.py            # Articulation wrapper (physx tensor API)
      single_articulation.py     # SingleArticulation (single-robot wrapper)
      single_prim_wrapper.py     # _SinglePrimWrapper (base class)
      xform_prim.py              # XFormPrim (prim view class)
    data/
      policy/
        policy.pt                # Bundled Go2 locomotion policy weights
        env.yaml                 # Bundled policy environment config (Isaac Lab format)
      preview.png                # Extension catalog preview
```

### Data flow

1. Keyboard press → `_base_command` accumulates 5-dim vector `[v_x, v_y, yaw_rate, pitch_rate, roll_rate]`
2. ROS2 `cmd_vel` → read from `/CMDVELGraph` outputs
3. **Merge** → `_merged_command = keyboard + ros2 + autopilot` → passed to `go2.forward()`
4. `go2.forward()` → policy inference (every `_decimation` steps) → `ArticulationAction` with joint position targets → applied to Go2 articulation

### Autopilot (ball-following)

When `autopilot_enabled` is True (toggled in the UI panel):

1. `_compute_ball_follow_command()` reads ball/robot transforms from USD stage
2. Computes yaw_error with `[-180, 180]` wrap to avoid ±180° seam reversal
3. Applies smoothstep roll blending that ramps as robot approaches the ball
4. Publishes `[Vx, 0.0, 2*Rz, -Rx, 0.0]` on `/CMDVELAutopilotGraph` (impulse-triggered ROS2 publisher)
5. Merged with keyboard in `on_physics_step()` via `_autopilot_command`

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

## Differences from Original Isaac Lab Repository

This extension diverges from the original Isaac Lab `go2withArm` repository in several key ways:

### Policy Architecture

- **Refactored into sub-packages**: The original monolithic `go2withpitch.py` has been split into `policy/robots/`, `policy/controllers/`, and `policy/core/` sub-packages for modularity
- **Policy data nested**: Policy weights and env config moved to `policy/data/policy/` (nested under the policy package)
- **Base controller**: `PolicyControllerGo2` extracted as a reusable base class with policy loading, env config parsing, and joint property resolution

### Scene Objects

- **Basket**: New `create_basket()` function — loads KLT bin from NVIDIA OCP URL at `[2.0, -1.0, 0.5]`
- **Mat**: New `create_mat()` function — loads plane from NVIDIA OCP URL with configurable OmniPBR material color
- **Ball position**: Changed from `[0.0, 1.0, 0.5]` to `[2.0, 1.0, 1.5]`
- **Arm mount**: Position changed from `[0.2, 0.0, 0.07]` to `[1.5, 0.0, 0.0]`
- **Go2 USD**: Now resolved via `get_assets_root_path()` (NVIDIA OCP) instead of hardcoded local path
- **Gripper**: Uses `Gripper_Ball_v3.usda` (was v1), wrapped with `RigidPrim` for physics
- **Fixed joint**: Quaternion transform via scipy instead of manual construction

### Autopilot

- **Yaw error wrapping**: Added `[-180, 180]` modulo wrap to avoid ±180° seam reversal
- **Yaw command scaling**: Doubled yaw output (`2*Rz`)
- **Roll blending**: Added smoothstep-based roll blending with Z-axis sign awareness
- **Command vector**: `[Vx, 0.0, 2*Rz, -Rx, 0.0]` (was `[Vx, 0.0, Rz, -Rx, 0.0]`)

### Camera System

- **CameraConfig dataclass**: Structured camera config objects in `cameras.py`
- **Viewport assignment**: Explicit dual-viewport camera assignment
- **Resolution**: Both cameras at 512×512

### UI Panel

- **Joint inspector**: Displays joint patterns from `UNITREE_GO2_CFG`
- **Hip finder**: Traverses Go2 USD to find hip joint indices
- **Live telemetry**: Ball offset, distance XY, robot/target/yaw error, merged command

## Developing

- **Run**: Start Isaac Sim with the extension folder, enable in Extension Manager, run "Go2 Arm Teleop v1" from Examples Browser (Shift+Ctrl+X)
- **CLI launch**: `./isaac-sim.sh -v --ext-folder /home/gavin/go2withArmExtension --exts/go2armteleop_extension.enable=1`
- **No tests or linting** — validation is through simulation run
- To modify the policy: edit `go2armteleop_extension/policy/robots/go2withpitch.py` (observation space is 50-dim, action is 12-dim joint positions)
- To modify the base controller: edit `go2armteleop_extension/policy/controllers/policy_controller.py`
- To modify config loading: edit `go2armteleop_extension/policy/controllers/config_loader.py`
- To modify core prims: edit `go2armteleop_extension/policy/core/` modules
- To modify the sample scene: edit `go2armteleop_extension/scene.py` (`setup_scene()` function)
- To modify camera setup: edit `go2armteleop_extension/cameras.py` (`CameraConfig` dataclass and `setup_viewports()`)
- To modify OmniGraphs: edit `go2armteleop_extension/omnigraphs.py` (`setup_omnigraphs()` function)
- To add new ROS2 topics: create new OmniGraphs in `setup_omnigraphs()` using `isaacsim.ros2.bridge` node types
- To modify the UI panel: edit `go2armteleop_extension/ui_extension.py` (autopilot toggle, telemetry labels, joint inspector)
- To modify autopilot ball-following: edit `go2armteleop_extension/go2armteleop.py` (`_compute_ball_follow_command()`, `_publish_cmd_vel_autopilot()`)
