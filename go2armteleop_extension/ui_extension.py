# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import sys

import omni.ext
import omni.graph.core as og
import omni.ui as ui
from pxr import Usd
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort: skip
import numpy as np
from pxr import UsdGeom, Sdf, Gf, UsdPhysics
import carb

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
        self._vla_command_value = "go to green mat"
        self._vla_graph_attr_path = "/VLACommandGraph/ros2_publisher_VLA.inputs:data"
        self._vla_graph_attr_candidates = [
            "/VLACommandGraph/ros2_publisher_VLA.inputs:data",
            "/VLACommandGraph/ros2_publisher_VLA.inputs:text",
            "/VLACommandGraph/ros2_publisher_VLA.inputs:message",
            "/VLACommandGraph/ros2_publisher_VLA.inputs:string",
        ]
        self._vla_graph_attr = None
        self._vla_graph_warned = False
        self._vla_field_model = ui.SimpleStringModel(self._vla_command_value)

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
                    def on_set_ball_vel():
                        x = self._ball_vel_x.model.get_value_as_float()
                        y = self._ball_vel_y.model.get_value_as_float()
                        z = self._ball_vel_z.model.get_value_as_float()
                        scene_mod = sys.modules.get("scene")
                        if scene_mod and hasattr(scene_mod, "set_ball_velocity"):
                            scene_mod.set_ball_velocity(x, y, z)
                            self._ball_vel_label.text = f"Ball Vel: {x}, {y}, {z}"
                        else:
                            self._ball_vel_label.text = "Ball vel: scene not ready"


                    def on_viewport_mouse_click(self, x: float, y: float, button: int):
                            # We only care about the Left Mouse Button (typically button code 0)
                            if button != 0:
                                return

                            # 3. Get the active viewport API object
                            viewport_api = omni.kit.viewport.utility.get_active_viewport()
                            if not viewport_api:
                                return

                            # 4. Convert the 2D window coordinates (x, y) into a 3D ray
                            # This returns an (origin, direction) tuple of carb.Float3 objects
                            ray_origin, ray_dir = viewport_api.get_ray_from_screen_x_y(x, y)

                            # 5. Perform the PhysX Raycast into the stage
                            max_distance = 10000.0  # Adjust based on scene scale
                            hit = get_physx_scene_query_interface().raycast_closest(ray_origin, ray_dir, max_distance)

                            if hit["hit"]:
                                # 6. Retrieve the exact surface coordinate!
                                surface_xyz = hit["position"]  # Tuple of (X, Y, Z)
                                prim_path = hit["rigid_body"]   # The path of the hit object
                                
                                print(f"Clicked on Object: {prim_path}")
                                print(f"Surface Coordinate: {surface_xyz}")
                                
                                # Forward the coordinates to your ball placement function
                                self.on_place_ball(surface_xyz)
                            else:
                                print("Clicked, but the ray did not hit any colliders.")

                    def setup_mouse_listener():
                        # 1. Get the active viewport window and its underlying UI frame
                        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
                        if not viewport_window:
                            print("No active viewport window found!")
                            return
                            
                        self._viewport_frame = viewport_window.get_frame("MyViewportOverlay")
                        
                        # 2. Register a mouse pressed event listener on the viewport frame
                        # This will trigger 'on_viewport_mouse_click' whenever the user clicks
                        self._mouse_sub = self._viewport_frame.set_mouse_pressed_fn(on_viewport_mouse_click)

                    def on_place_ball():
                        """Raycast from camera center and place ball at surface + [0,0,0.5].

                        Uses the modern viewport → camera-path → USD-transform pattern:
                        1. get_active_viewport_camera_path()  →  Sdf.Path
                        2. stage.GetPrimAtPath(camera_path)    →  camera prim
                        3. Xformable(camera_prim).ComputeLocalToWorldTransform()  →  Gf.Matrix4d
                        4. ExtractTranslation / ExtractRotation  →  position + direction
                        """
                        import omni.kit.viewport.utility
                        from omni.physx import get_physx_interface
                        from omni.physx import get_physx_scene_query_interface

                        camera_path = omni.kit.viewport.utility.get_active_viewport_camera_path()
                        if not camera_path:
                            self._ball_vel_label.text = "Place Ball: no camera path"
                            return

                        stage = omni.usd.get_context().get_stage()
                        camera_prim = stage.GetPrimAtPath(camera_path)
                        if not camera_prim or not camera_prim.IsValid():
                            self._ball_vel_label.text = "Place Ball: camera prim not found"
                            return

                        # Camera world-space position and direction
                        xformable = UsdGeom.Xformable(camera_prim)
                        world_mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        camera_pos = world_mat.ExtractTranslation()          # Gf.Vec3d

                        # Extract the upper-left 3x3 rotation matrix from the 4x4 transform
                        rot_mat = Gf.Matrix3d(
                            world_mat[0][0], world_mat[0][1], world_mat[0][2],
                            world_mat[1][0], world_mat[1][1], world_mat[1][2],
                            world_mat[2][0], world_mat[2][1], world_mat[2][2],
                        )

                        # Transform camera's forward axis (-Z in camera local space) into world space
                        forward = Gf.Vec3d(0, 0, -1)
                        camera_dir = rot_mat * forward                        # Gf.Vec3d

                        #physx_interface = get_physx_interface()
                        #hit = physx_interface.raycast(camera_pos, camera_dir, max_dist=1000.0)
                        origin = carb.Float3(camera_pos[0], camera_pos[1], camera_pos[2])
                        rayDir = carb.Float3(camera_dir[0], camera_dir[1], camera_dir[2])
                        distance = 1000.0
                        hit = get_physx_scene_query_interface().raycast_closest(origin, rayDir, distance)
                        #hit = get_physx_scene_query_interface().raycast_closest(camera_pos, camera_dir, max_dist=1000.0)
                        if hit["hit"]:
                            print(f"[Go2Arm] Raycast hit at {hit['position']}, normal {hit['normal']}")
                            surface_xyz = hit["position"]
                            new_pos = [surface_xyz.x, surface_xyz.y, surface_xyz.z + 0.5]
                            import scene
                            scene.set_ball_position(new_pos[0], new_pos[1], new_pos[2])
                            self._ball_vel_label.text = f"Ball pos: {new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f}"
                        else:
                            self._ball_vel_label.text = "Place Ball: no hit"
                            print("[Go2Arm] Raycast did not hit any surface.")

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
                            #omni.ui.Button("Place Ball", clicked_fn=setup_mouse_listener)
                            omni.ui.Button("Set Ball Vel", clicked_fn=on_set_ball_vel)



                    # Telemetry labels — live updated via update event stream
                    with omni.ui.CollapsableFrame("Stats", style={"header_background_color": (0.2, 0.6, 0.8)}):
                        with omni.ui.VStack():
                            self._telemetry_offset_label = omni.ui.Label("Ball Offset: N/A")
                            self._telemetry_command_label = omni.ui.Label("Command: N/A")
                            self._robot_yaw_label = omni.ui.Label("Robot Yaw: N/A")
                            self._target_yaw_label = omni.ui.Label("Target Yaw: N/A")
                            self._yaw_error_label = omni.ui.Label("Yaw Error: N/A")
                            self._dist_xy_label = omni.ui.Label("Distance XY: N/A")
                            # Ball velocity controls (collapsible to save space)
                            self._ball_vel_label = omni.ui.Label("")



                    with omni.ui.CollapsableFrame("Ball Velocity", style={"header_background_color": (0.2, 0.6, 0.8)}):
                        with omni.ui.VStack():
                            with omni.ui.HStack():
                                omni.ui.Label("X:", width=16)
                                self._ball_vel_x = ui.FloatField(ui.SimpleFloatModel(0.0), width=50)
                                omni.ui.Label("Y:", width=16)
                                self._ball_vel_y = ui.FloatField(ui.SimpleFloatModel(0.0), width=50)
                                omni.ui.Label("Z:", width=16)
                                self._ball_vel_z = ui.FloatField(ui.SimpleFloatModel(0.0), width=50)

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

                    def _publish_vla_command():
                        self._vla_command_value = self._vla_field_model.get_value_as_string()
                        try:
                            if self._vla_graph_attr is None:
                                for attr_path in self._vla_graph_attr_candidates:
                                    try:
                                        self._vla_graph_attr = og.Controller().attribute(attr_path)
                                        self._vla_graph_attr_path = attr_path
                                        self._vla_graph_warned = False
                                        break
                                    except Exception:
                                        self._vla_graph_attr = None

                            # Graph may not exist yet (e.g. before scene/omnigraph setup).
                            if self._vla_graph_attr is None:
                                return

                            self._vla_graph_attr.set(self._vla_command_value)
                        except Exception as exc:
                            self._vla_graph_attr = None
                            if not self._vla_graph_warned:
                                print(f"[quadruped_go2_locomotion] Warning: unable to set VLA graph input: {exc}")
                                self._vla_graph_warned = True

                    self._vla_update_subscription = _event_stream.create_subscription_to_pop(
                        lambda event: _publish_vla_command(),
                        name="vla_command_publish",
                    )

                    with omni.ui.CollapsableFrame("VLA Command", style={"header_background_color": (0.6, 0.4, 0.2)}):
                        with omni.ui.VStack():
                            omni.ui.Label("ROS2 topic: VLA_Command")
                            self._vla_command_field = ui.StringField(self._vla_field_model, width=360)

    def on_shutdown(self):
        if getattr(self, "_vla_update_subscription", None) is not None:
            self._vla_update_subscription = None
        if getattr(self, "_telemetry_subscription", None) is not None:
            self._telemetry_subscription = None
        self._vla_graph_attr = None
        print("[quadruped_go2_locomotion] shutdown")
