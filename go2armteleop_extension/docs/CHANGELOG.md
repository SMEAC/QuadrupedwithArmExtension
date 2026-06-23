# Changelog

## [1.1.0] - 2026-06-10

### Changed

- **Refactored `go2armteleop.py` into submodules**: Split the monolithic ~500-line sample file into four focused modules:
  - `scene.py` — scene object creation (ground plane, Go2 robot, OpenManipulator-X arm, LiDAR, cameras)
  - `cameras.py` — camera setup and viewport assignment
  - `omnigraphs.py` — ROS2 bridge OmniGraph creation (cmd_vel, arm, LiDAR, odometry, cameras, clock)
  - `go2armteleop.py` — orchestration, keyboard input, physics callbacks

- **Policy paths are now relative to the extension's data directory**: Policy weights (`.pt`) and config (`.yaml`) are bundled in `data/policy/` instead of hardcoded to `/home/gavin/isaacSimData/go2Policy/gavin6Jun26/`.

### Added

- `data/policy/policy.pt` — bundled Go2 locomotion policy weights
- `data/policy/env.yaml` — bundled policy environment configuration

### Added

- **Tennis ball** — bundled USD asset (`resource/Tennis_ball_01.usda`) loaded into the scene at ``[0.0, 1.0, 0.5]`` via a new ``create_ball()`` function in ``scene.py``

### Updated


- `config/extension.toml` — revision bumped to 1.1.0, new changelog entry
- `README.md` — updated architecture diagram and configuration docs
- `docs/Overview.md` — updated architecture diagram and configuration docs
