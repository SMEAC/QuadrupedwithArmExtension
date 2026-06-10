# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Policy controller for the Go2 quadruped with flat terrain locomotion."""

import os
from typing import Optional

import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path


def _get_extension_root() -> str:
    """Return the root directory of the go2armteleop_extension package."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Go2withPitchFlatTerrainPolicy(PolicyController):
    """Go2 quadruped policy controller with base pitch and yaw control.

    This controller runs a learned locomotion policy on a Unitree Go2 robot.
    It accepts a 5-element command vector: [v_x, v_y, yaw_rate, pitch_rate, roll_rate].
    """

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "Go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        policy_path: Optional[str] = None,
        policy_config_path: Optional[str] = None,
    ) -> None:
        """Initialize robot and load RL policy.

        Args:
            prim_path: USD prim path of the robot on the stage.
            root_path: Path to the articulation root of the robot.
            name: Name of the quadruped.
            usd_path: Robot USD file path. Defaults to the Isaac Sim asset path.
            position: Initial position of the robot.
            orientation: Initial orientation of the robot.
            policy_path: Path to the trained policy .pt file.
                Defaults to the bundled policy in the extension's data/policy/ directory.
            policy_config_path: Path to the policy environment config .yaml file.
                Defaults to the bundled config in the extension's data/policy/ directory.
        """
        assets_root_path = get_assets_root_path()
        if usd_path is None:
            usd_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        # Policy defaults — bundled with the extension in data/policy/
        if policy_path is None:
            extension_root = _get_extension_root()
            policy_path = os.path.join(extension_root, "data", "policy", "policy.pt")
        if policy_config_path is None:
            extension_root = _get_extension_root()
            policy_config_path = os.path.join(extension_root, "data", "policy", "env.yaml")

        self.load_policy(policy_path, policy_config_path)

        self._action_scale = 0.25
        self._previous_action = np.zeros(12)
        self._policy_counter = 0

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """Compute the observation vector for the policy.

        Args:
            command: The robot command (v_x, v_y, yaw_rate, pitch_rate, roll_rate), size 5.

        Returns:
            Observation vector of size 50 containing linear velocity, angular velocity,
            gravity direction, command, joint states, and previous action.
        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(50)
        # Base linear velocity (body frame)
        obs[:3] = lin_vel_b
        # Base angular velocity (body frame)
        obs[3:6] = ang_vel_b
        # Gravity vector (body frame)
        obs[6:9] = gravity_b
        # Command (v_x, v_y, pitch, yaw_rate, roll_rate)
        obs[9:14] = command
        # Joint positions (relative to default)
        current_joint_pos = self.robot.get_joint_positions()[:12]
        obs[14:26] = current_joint_pos - self.default_pos
        # Joint velocities
        current_joint_vel = self.robot.get_joint_velocities()[:12]
        obs[26:38] = current_joint_vel
        # Previous action
        obs[38:50] = self._previous_action

        return obs

    def forward(self, dt: float, command: np.ndarray) -> None:
        """Compute desired torques and apply them to the articulation.

        Args:
            dt: Timestep update in the world.
            command: The robot command (v_x, v_y, yaw_rate, pitch_rate, roll_rate).
        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        # Build the 12-element target positions for the Go2 leg joints only
        leg_positions = self.default_pos + (self.action * self._action_scale)
        action = ArticulationAction(joint_positions=leg_positions, joint_indices=np.arange(12))
        self.robot.apply_action(action)

        self._policy_counter += 1
