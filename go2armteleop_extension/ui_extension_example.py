# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import omni.ext
from pxr import Usd
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort: skip
import numpy as np
from pxr import UsdGeom, Sdf, Gf, UsdPhysics

# Module-level flag shared with Go2ArmExample for autopilot toggle
autopilot_enabled = False

# Module-level telemetry shared with Go2ArmExample
_telemetry_offset = "N/A"
_telemetry_command = "N/A"
_robot_yaw = "N/A"
_target_yaw = "N/A"
_yaw_error = "N/A"
_dist_xy = "N/A"


def find_joint_indices(usd_path: str) -> tuple[list[int], list[tuple[int, str]]]:
    """Find joint indices by traversing the USD file hierarchy.

    Isaac Lab resolves joint indices by depth-first traversal of the USD
    prim hierarchy. Every Joint / SkelRootJoint prim gets a sequential
    integer index starting at 0. This function mirrors that resolution
    so it can be called from the UI without a running simulation.

    Args:
        usd_path: Path to the GO2 USD file.

    Returns:
        (hip_indices, all_joints) -- hip_indices are joint indices where the
        prim name contains "hip"; all_joints is a list of (index, name) pairs.
    """
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"Cannot open USD file: {usd_path}")

    root = stage.GetPrimAtPath("/go2_description")
    if root is None or not root.IsValid():
        raise RuntimeError(f"Prim /go2_description not found in {usd_path}")

    joint_type_names = ("Joint", "PhysicsRevoluteJoint")
    hip_indices: list[int] = []
    all_joints: list[tuple[int, str]] = []

    idx = 0

    def traverse(prim):
        nonlocal idx
        kind = prim.GetTypeName()
        if kind in joint_type_names:
            name_lower = prim.GetName().lower()
            all_joints.append((idx, prim.GetName()))
            if "hip" in name_lower:
                hip_indices.append(idx)
            idx += 1
        for child in prim.GetAllChildren():
            traverse(child)

    for prim in root.GetAllChildren():
        traverse(prim)

    return hip_indices, all_joints


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[quadruped_go2_locomotion] some_public_function was called with x: ", x)
    return x**x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class ExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[quadruped_go2_locomotion] startup")

        self._count = 0

        # Hardcoded S3 path for the Unitree GO2 robot
        self._usd_path = (
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
            "/Assets/Isaac/5.1/Isaac/Robots/Unitree/Go2/go2.usd"
        )

        self._window = omni.ui.Window("My Window", width=500, height=550)
        with self._window.frame:
            with omni.ui.Frame():
                with omni.ui.VStack():
                    label = omni.ui.Label("")

                    def on_rotx():
                        joint_patterns = getattr(UNITREE_GO2_CFG.init_state, "joint_pos", {})
                        lines = ["Config joint patterns (regex):"]
                        for pat, default_val in joint_patterns.items():
                            lines.append(f"  {pat}: {default_val:.3f}")
                        label.text = "\n".join(lines)
                    def on_show_cfg():
                        joint_patterns = getattr(UNITREE_GO2_CFG.init_state, "joint_pos", {})
                        lines = ["Config joint patterns (regex):"]
                        for pat, default_val in joint_patterns.items():
                            lines.append(f"  {pat}: {default_val:.3f}")
                        label.text = "\n".join(lines)

                    def on_find_hips():
                        if self._usd_path:
                            hip_idx, all_joints = find_joint_indices(self._usd_path)
                            lines = [f"Hip joint indices: {hip_idx}", "All joints in order:"]
                            for idx_val, name in all_joints:
                                marker = " <-- hip" if idx_val in hip_idx else ""
                                lines.append(f"  [{idx_val}] {name}{marker}")
                            label.text = "\n".join(lines)
                        else:
                            label.text = "USD path not found in config."

                    def on_debug_hierarchy():
                        """Print and display the full USD hierarchy to find correct joint types."""
                        stage = omni.usd.get_context().get_stage()
                        loaded_joint = stage.GetPrimAtPath("/World/gripper_left/FixedJoint")
                        physics_joint = UsdPhysics.Joint(loaded_joint)
                        from scipy.spatial.transform import Rotation as R

                        quat_scipy = R.from_euler("xyz", np.array([0.0, 0.0, 0.0]), degrees=True).as_quat()
                        orientation = np.array([quat_scipy[0], quat_scipy[1], quat_scipy[2], quat_scipy[3]])
                        #orientation = np.array([0.0, 1.0, 0.0, 0.0])
                        print(f"Orientation quat: {orientation}")
                        physics_joint.GetLocalRot0Attr().Set(Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3]))
                        '''
                        if self._usd_path:
                            stage = Usd.Stage.Open(self._usd_path)
                            if stage is None:
                                label.text = "Failed to open USD file."
                                return

                            root = stage.GetPrimAtPath("/go2_description")
                            if root is None or not root.IsValid():
                                label.text = "Prim /go2_description not found in USD file."
                                return

                            all_prims = []

                            def traverse(prim, depth=0):
                                prefix = "  " * depth
                                kind = prim.GetTypeName()
                                name_lower = prim.GetName().lower()
                                all_prims.append(f"{prefix}type={kind:20s} name={prim.GetName()}")
                                for child in prim.GetAllChildren():
                                    traverse(child, depth + 1)

                            for prim in root.GetAllChildren():
                                traverse(prim)

                            # Show all unique types and a count of prims per type
                            type_counts: dict[str, int] = {}
                            for line in all_prims:
                                # Extract type from "type=Xxx"
                                start = line.index("type=") + 5
                                kind = line[start:start + 20].strip()
                                type_counts[kind] = type_counts.get(kind, 0) + 1

                            summary_lines = ["Type counts in USD:", str(type_counts), "Full hierarchy:"]
                            summary_lines += all_prims
                            label.text = "\n".join(summary_lines)
                        else:
                            label.text = "USD path not found in config."
                        '''

                    def on_reset():
                        self._count = 0
                        label.text = "Press a button to start"

                    on_reset()

                    autopilot_label = omni.ui.Label("Autopilot: OFF")

                    def toggle_autopilot():
                        global autopilot_enabled
                        autopilot_enabled = not autopilot_enabled
                        autopilot_label.text = f"Autopilot: {'ON' if autopilot_enabled else 'OFF'}"

                    with omni.ui.HStack():
                        omni.ui.Button("Show Config", clicked_fn=on_debug_hierarchy)
                        with omni.ui.HStack():
                            omni.ui.Button("Autopilot", clicked_fn=toggle_autopilot)

                    # Telemetry labels — live updated via update event stream
                    self._telemetry_offset_label = omni.ui.Label("Ball Offset: N/A")
                    self._telemetry_command_label = omni.ui.Label("Command: N/A")
                    self._robot_yaw_label = omni.ui.Label("Robot Yaw: N/A")
                    self._target_yaw_label = omni.ui.Label("Target Yaw: N/A")
                    self._yaw_error_label = omni.ui.Label("Yaw Error: N/A")
                    self._dist_xy_label = omni.ui.Label("Distance XY: N/A")

                    def _update_telemetry_labels():
                        self._telemetry_offset_label.text = f"Ball Offset: {_telemetry_offset}"
                        self._telemetry_command_label.text = f"Command: {_telemetry_command}"
                        self._robot_yaw_label.text = f"Robot Yaw: {_robot_yaw}"
                        self._target_yaw_label.text = f"Target Yaw: {_target_yaw}"
                        self._yaw_error_label.text = f"Yaw Error: {_yaw_error}"
                        self._dist_xy_label.text = f"Distance XY: {_dist_xy}"

                    # Subscribe to app update event for live refresh
                    _app = omni.kit.app.get_app()
                    _event_stream = _app.get_update_event_stream()
                    self._telemetry_subscription = _event_stream.create_subscription_to_pop(
                        lambda event: _update_telemetry_labels(),
                        name="telemetry_update",
                    )

    def on_shutdown(self):
        print("[quadruped_go2_locomotion] shutdown")
