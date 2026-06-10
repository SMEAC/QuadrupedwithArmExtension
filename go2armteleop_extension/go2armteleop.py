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
  - Two cameras (on the arm and on the Go2 head) streaming via ROS2
  - An RTX LiDAR with point cloud output via ROS2
  - Keyboard teleoperation of the Go2 base (forward/back/strafe/spin/pitch)
  - ROS2 bridge integration for cmd_vel, joint_states, odometry, transforms, and camera feeds

Architecture:
  Scene setup    → scene.py
  Camera/view    → cameras.py
  OmniGraphs     → omnigraphs.py
  This file      → orchestration, keyboard input, physics callbacks
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
import omni
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
        self._max_command = 0.5

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
            # Merge ROS2 cmd_vel with keyboard base_command
            meta = self._omnigraphs_meta
            new_cmd = [
                omni.graph.core.Controller().attribute(meta["cmdvel_linear_x"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_linear_y"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_z"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_y"]).get(),
                omni.graph.core.Controller().attribute(meta["cmdvel_angular_x"]).get(),
            ]
            self._merged_command = [x + y for x, y in zip(new_cmd, self._base_command)]
            self.go2.forward(step_size, self._merged_command)
        else:
            self._physics_ready = True
            self.go2.initialize(physics_sim_view="/World/go2")
            print(f"[Go2Arm] Go2 articulation bodies: {self.go2.robot.get_articulation_body_count()}")
            self.go2.post_reset()
            self.go2.robot.set_joints_default_state(self.go2.default_pos)
            self.arm.initialize()

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

    # --------------- property accessors ---------------
    @property
    def go2(self):
        return self._scene["go2"]

    @property
    def arm(self):
        return self._scene["arm"]
