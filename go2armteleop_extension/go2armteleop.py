# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Interactive sample: Go2 quadruped with OpenManipulator-X arm, teleoperated via keyboard.

This sample demonstrates:
  - A Unitree Go2 robot running a flat-terrain RL locomotion policy
  - An OpenManipulator-X arm rigidly attached to the Go2 base
  - Custom gripper assets mounted on both arm gripper links
  - Tennis ball asset in the scene for ball-following autopilot
  - Two cameras (on the arm and on the Go2 head) streaming via ROS2
  - An RTX LiDAR with point cloud output via ROS2
  - Keyboard teleoperation of the Go2 base (forward/back/strafe/spin/pitch/roll)
  - Autopilot ball-following mode (toggle via UI panel)
  - ROS2 bridge integration for cmd_vel, joint_states, odometry, transforms, and camera feeds

Architecture:
  Scene setup    → scene.py
  Camera/view    → cameras.py
  OmniGraphs     → omnigraphs.py
  UI panel       → ui_extension.py
  This file      → orchestration, keyboard input, physics callbacks, autopilot
"""

import os
import sys

# Ensure the extension package directory is on sys.path so sibling subpackages
# (e.g. policy/, cameras/, scene/, omnigraphs.py) are importable.
_package_root = os.path.dirname(os.path.abspath(__file__))
if _package_root not in sys.path:
    sys.path.insert(0, _package_root)

import carb
import numpy as np
import math

import omni
from pxr import UsdGeom, Usd, Gf
from isaacsim.examples.interactive.base_sample import BaseSample
from scene import setup_scene
from cameras import setup_viewports
from omnigraphs import setup_omnigraphs



class Go2ArmExample(BaseSample):
    """Interactive Go2 + OpenManipulator-X sample with keyboard teleoperation and ROS2 bridge."""

    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 200.0
        self._world_settings["rendering_dt"] = 4.0 / 200.0
        self._base_command = [0.0, 0.0, 0.0, 0.1, 0.0]
        self._merged_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._autopilot_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._max_command = 0.5

        # Autopilot ball-following parameters
        self._ball_follow_dist = 0.6
        self._ball_follow_start = 1.5
        self._yaw_gain = 0.01
        self._vx_gain = 0.8
        self._roll_gain = 1.0
        self._max_yaw_rate = 1.5
        self._max_vx = 0.4

        # Keyboard-to-command bindings
        self._input_keyboard_mapping = {
            # Forward
            "NUMPAD_8": [self._max_command, 0.0, 0.0, 0.0, 0.0],
            "UP":     [self._max_command, 0.0, 0.0, 0.0, 0.0],
            # Backward
            "NUMPAD_2": [-self._max_command, 0.0, 0.0, 0.0, 0.0],
            "DOWN":   [-self._max_command, 0.0, 0.0, 0.0, 0.0],
            # Strafe left
            "NUMPAD_6": [0.0, -self._max_command, 0.0, 0.0, 0.0],
            "RIGHT":  [0.0, -self._max_command, 0.0, 0.0, 0.0],
            # Strafe right
            "NUMPAD_4": [0.0, self._max_command, 0.0, 0.0, 0.0],
            "LEFT":   [0.0, self._max_command, 0.0, 0.0, 0.0],
            # Yaw (positive = counterclockwise)
            "NUMPAD_7": [0.0, 0.0, 0.0, self._max_command, 0.0],
            "N":      [0.0, 0.0, 0.0, self._max_command, 0.0],
            # Yaw (negative = clockwise)
            "NUMPAD_9": [0.0, 0.0, 0.0, -self._max_command, 0.0],
            "M":      [0.0, 0.0, 0.0, -self._max_command, 0.0],
            # Pitch up
            "A": [0.0, 0.0, 0.0, 0.0, self._max_command],
            "a": [0.0, 0.0, 0.0, 0.0, self._max_command],
            # Pitch down
            "Z": [0.0, 0.0, 0.0, 0.0, -self._max_command],
            "z": [0.0, 0.0, 0.0, 0.0, -self._max_command],
            # Roll (right)
            "X": [0.0, 0.0, 0.0, 0.0, -self._max_command],
            "x": [0.0, 0.0, 0.0, 0.0, -self._max_command],
            # Roll (left)
            "C": [0.0, 0.0, 0.0, 0.0, self._max_command],
            "c": [0.0, 0.0, 0.0, 0.0, self._max_command],
        }

    # --------------- scene setup ---------------
    def setup_scene(self) -> None:
        # Create all scene objects (ground, Go2, arm, cameras, LiDAR)
        scene_data = setup_scene(world=self._world)
        self._scene = scene_data  # store for later access

        # Assign cameras to viewport windows
        setup_viewports(
        )

        # Create all ROS2 bridge OmniGraphs
        self._omnigraphs_meta = setup_omnigraphs(
            controller=omni.graph.core.Controller(),
            camera_arm=scene_data["camera_arm"],
            camera_quadruped=scene_data["camera_quadruped"],
        )

        # Store timeline event subscription (callback set in setup_post_load)
        self._event_stream = scene_data["event_stream"]

    # --------------- lifecycle ---------------
    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )
        self._physics_ready = False

        # Subscribe to timeline PLAY events to reset physics on replay
        self._event_timer_callback = self._event_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size: float) -> None:
        if self._physics_ready:
            meta = self._omnigraphs_meta
            ros2_cmd = [
                omni.graph.core.Controller().attribute(meta["cmdvel_linear_x"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_linear_y"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_z"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_y"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_x"]).get(),
            ]
            #print(f"[Go2Arm] ROS2 cmd_vel: ({ros2_cmd[0]:.3f}, {ros2_cmd[1]:.3f}, {ros2_cmd[2]:.3f}, {ros2_cmd[3]:.3f}, {ros2_cmd[4]:.3f})")
            if self.autopilot_enabled:
                self._compute_ball_follow_command()

            offset = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._merged_command = [x + y + z for x, y, z in zip(offset, ros2_cmd, self._base_command)]
            

            # Write telemetry for UI display
            import ui_extension
            from pxr import UsdGeom, Usd
            ball_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/ball/Tennis_ball_01/ball")
            robot_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Go2/base")
            if ball_prim and ball_prim.IsValid() and robot_prim and robot_prim.IsValid():

               # 1. Cast the prim to an Xformable schema
                xformable = UsdGeom.Xformable(ball_prim)
                world_transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                ball_tf = world_transform_matrix.ExtractTranslation()
                #print(f"[Go2Arm] Ball position Global: ({ball_tf[0]:.3f}, {ball_tf[1]:.3f}, {ball_tf[2]:.3f})")

                xformable = UsdGeom.Xformable(robot_prim)
                world_transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                robot_tf = world_transform_matrix.ExtractTranslation()

                R = world_transform_matrix.ExtractRotation()
                from isaacsim.core.utils.rotations import matrix_to_euler_angles
                euler_angles = matrix_to_euler_angles(Gf.Matrix3d(R),degrees=True, extrinsic=False)

                dx = ball_tf[0] - robot_tf[0]
                dy = ball_tf[1] - robot_tf[1]
                dz = ball_tf[2] - robot_tf[2]
                dist_xy = math.hypot(dx, dy)

                # --- Rz: yaw toward ball relative to robot's current heading ---
                robot_yaw   = euler_angles[2]
                #print(f"[Go2Arm] Robot yaw: {robot_yaw:.1f} deg, angle to ball: {math.degrees(math.atan2(dy, dx)):.1f} deg")

                #robot_yaw = math.atan2(robot_rot_mat[1][0], robot_rot_mat[0][0])
                target_yaw = -math.degrees(math.atan2(dy, dx))
                #print(f"[Go2Arm] Target yaw: {target_yaw:.1f} deg")
                yaw_error = robot_yaw - target_yaw
                #print(f"[Go2Arm] Yaw error: {yaw_error:.1f} deg")


                #print(f"[Go2Arm] Robot position: ({robot_tf[0]:.3f}, {robot_tf[1]:.3f}, {robot_tf[2]:.3f})")
                dx = ball_tf[0] - robot_tf[0]
                dy = ball_tf[1] - robot_tf[1]
                dz = ball_tf[2] - robot_tf[2]
                ui_extension._telemetry_offset = f"({dx:.3f}, {dy:.3f}, {dz:.3f})"
                ui_extension._dist_xy = f"({dist_xy:.1f} m)"

                ui_extension._robot_yaw = f"({robot_yaw:.1f} deg)"
                ui_extension._target_yaw = f"({target_yaw:.1f} deg)"
                ui_extension._yaw_error = f"({yaw_error:.1f} deg)"

                #print(f"[Go2Arm] Ball offset: {ui_extension._telemetry_offset}")
                ui_extension._telemetry_command = (
                    f"[{self._merged_command[0]:.3f}, "
                    f"{self._merged_command[1]:.3f}, "
                    f"{self._merged_command[2]:.3f}, "
                    f"{self._merged_command[3]:.3f}, "
                    f"{self._merged_command[4]:.3f}]"
                )


 



            self.go2.forward(step_size, self._merged_command)
        else:
            self.go2.initialize(physics_sim_view="/World/go2")
            self.go2.post_reset()
            self.go2.robot.set_joints_default_state(self.go2.default_pos)
            # Arm is controlled via OmniGraph subscriptions, so a tensor articulation
            # handle is not required here and can fail for non-homogeneous roots.
            self._physics_ready = True

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """Keyboard event subscriber: accumulates on press, releases on key-up."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True

    def _on_timeline_play(self, event) -> None:
        """Reset physics state when the timeline is played."""
        if hasattr(self, "go2") and self.go2:
            self._physics_ready = False
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def world_cleanup(self) -> None:
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")

    def _publish_cmd_vel_autopilot(self, cmd: list[float]) -> None:
        """Publish [vx, vy, yaw_rate, pitch_rate, roll_rate] on cmd_vel_autopilot via OmniGraph."""
        meta = self._omnigraphs_meta
        try:
            ctrl = omni.graph.core.Controller()
            ctrl.attribute(meta["cmdvel_autopilot_linear_x"]).set(float(cmd[0]))
            ctrl.attribute(meta["cmdvel_autopilot_linear_y"]).set(float(cmd[1]))
            ctrl.attribute(meta["cmdvel_autopilot_angular_z"]).set(float(cmd[2]))
            ctrl.attribute(meta["cmdvel_autopilot_angular_y"]).set(float(cmd[3]))
            ctrl.attribute(meta["cmdvel_autopilot_angular_x"]).set(float(cmd[4]))
            # Emit exactly one message publish event when autopilot updates.
            ctrl.attribute(meta["cmdvel_autopilot_impulse"]).set(True)
        except Exception as e:
            print(f"[Go2Arm] Warning: unable to publish cmd_vel_autopilot via OmniGraph: {e}")

    # --------------- autopilot toggle ---------------
    @property
    def autopilot_enabled(self):
        from ui_extension import autopilot_enabled
        return autopilot_enabled

    def _compute_ball_follow_command(self) -> None:
        """Compute and publish [v_x, v_y, yaw_rate, pitch_rate, roll_rate] for ball follow.

        All three commands are computed simultaneously with smooth blending:
          Rz (yaw_rate)  — proportional to angle error to ball
          Vx (forward)   — proportional to distance above target
          Rx (roll/lean) — ramps up as robot approaches the ball
        """

        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()

        ball_prim = stage.GetPrimAtPath("/World/ball/Tennis_ball_01/ball")
        if not ball_prim or not ball_prim.IsValid():
            self._autopilot_command = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._publish_cmd_vel_autopilot(self._autopilot_command)
            return

        robot_prim = stage.GetPrimAtPath("/World/Go2/base")
        if not robot_prim or not robot_prim.IsValid():
            self._autopilot_command = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._publish_cmd_vel_autopilot(self._autopilot_command)
            return

        xformable = UsdGeom.Xformable(ball_prim)
        world_transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        ball_tf = world_transform_matrix.ExtractTranslation()
        #print(f"[Go2Arm] Ball position Global: ({ball_tf[0]:.3f}, {ball_tf[1]:.3f}, {ball_tf[2]:.3f})")

        xformable = UsdGeom.Xformable(robot_prim)
        world_transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        robot_tf = world_transform_matrix.ExtractTranslation()
        R = world_transform_matrix.ExtractRotation()
        from isaacsim.core.utils.rotations import matrix_to_euler_angles
        euler_angles = matrix_to_euler_angles(Gf.Matrix3d(R),degrees=True, extrinsic=False)
        #euler_angles = robot_rot_mat.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())

        dx = ball_tf[0] - robot_tf[0]
        dy = ball_tf[1] - robot_tf[1]
        dz = ball_tf[2] - robot_tf[2]
        dist_xy = math.hypot(dx, dy)

        # --- Rz: yaw toward ball relative to robot's current heading ---
        robot_yaw   = euler_angles[2]
        
        target_yaw = -math.degrees(math.atan2(dy, dx))

        yaw_error = robot_yaw - target_yaw
        # Wrap to [-180, 180] to avoid crossing the ±180° seam the long way
        yaw_error = (yaw_error + 180.0) % 360.0 - 180.0

        #Rz = max(-self._max_yaw_rate, min(self._max_yaw_rate, self._yaw_gain * yaw_error))
        Rz = yaw_error * self._yaw_gain
        # --- Vx: approach velocity (zero when at / inside target distance) ---
        if dist_xy > self._ball_follow_dist:
            span = self._ball_follow_start - self._ball_follow_dist
            Vx = min(self._vx_gain * (dist_xy - self._ball_follow_dist) / span,
                     self._max_vx)
        else:
            Vx = 0.0

        # --- Rx: lean (roll) toward ball, ramps between start and target distance ---
        if dist_xy < self._ball_follow_start:
            span = self._ball_follow_start - self._ball_follow_dist
            t = max(0.0, 1.0 - (dist_xy - self._ball_follow_dist) / span)
            t = t * t  # smoothstep for extra smoothness
            Rx = max(-1.0, min(1.0,
                               self._roll_gain * t * math.copysign(1, dz)))
        else:
            Rx = 0.0

        self._autopilot_command = [Vx, 0.0, 2*Rz, -Rx, 0.0]
        self._publish_cmd_vel_autopilot(self._autopilot_command)

    # --------------- property accessors ---------------
    @property
    def go2(self):
        return self._scene["go2"]

    @property
    def arm(self):
        return self._scene["arm"]
