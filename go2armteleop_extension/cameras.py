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

"""Camera setup and viewport assignment for the Go2-Arm teleoperation scene.

This module creates two cameras (arm wrist camera and Go2 head camera) and
assigns them to the Isaac Sim viewport windows.
"""

from typing import Optional

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera
from omni.kit.viewport.utility import (
    get_active_viewport,
    get_viewport_from_window_name,
    get_active_viewport_and_window,
    get_active_viewport_window,
)

from scipy.spatial.transform import Rotation as R


class CameraConfig:
    """Configuration for a single camera in the scene."""

    def __init__(
        self,
        prim_path: str,
        parent_link: str,
        offset: np.ndarray,
        euler_deg: np.ndarray,
        resolution: tuple[int, int] = (512, 512),
        rotation_rate: float = 30.0,
    ):
        """Initialize camera configuration.

        Args:
            prim_path: USD prim path for the camera.
            parent_link: USD prim path of the parent link/body.
            offset: Translation offset from the parent link origin [x, y, z].
            euler_deg: Euler angles (yaw, pitch, roll) in degrees for camera orientation.
            resolution: Camera resolution as (width, height).
            rotation_rate: Camera update rate in Hz.
        """
        self.prim_path = prim_path
        self.parent_link = parent_link
        self.offset = offset
        self.euler_deg = euler_deg
        self.resolution = resolution
        self.rotation_rate = rotation_rate


def create_camera(config: CameraConfig) -> Camera:
    """Create a Camera object from a CameraConfig.

    Args:
        config: Camera configuration specifying prim path, offset, and orientation.

    Returns:
        Configured Camera instance ready for use.
    """
    camera_position = config.offset
    quat_scipy = R.from_euler("yxz", config.euler_deg, degrees=True).as_quat()
    camera_orientation = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])

    camera = Camera(
        prim_path=config.prim_path,
        position=camera_position,
        orientation=camera_orientation,
        resolution=config.resolution,
    )
    camera.set_focal_length(1.0)
    camera.set_clipping_range(near_distance=0.01, far_distance=10.0)
    camera.set_local_pose(
        translation=camera_position,
        orientation=camera_orientation,
        camera_axes="usd",
    )
    return camera


# Pre-defined camera configurations for the Go2-Arm scene
CAMERA_ARM = CameraConfig(
    prim_path="/World/open_manipulator_x/link5/Go2Camera",
    parent_link="link5",
    offset=np.array([0.04, 0.0, 0.06]),
    euler_deg=np.array([0.0, 73.0, -90]),
    resolution=(512, 512),
)

CAMERA_QUADRUPED = CameraConfig(
    prim_path="/World/Go2/Head_upper/Go2Camera",
    parent_link="Head_upper",
    offset=np.array([0.04, 0.0, 0.017]),
    euler_deg=np.array([0.0, 90.0, -90]),
    resolution=(512, 512),
)


def setup_viewports(
    viewport1_window: Optional[object] = None,
    viewport2_window: Optional[str] = None,
) -> None:
    """Assign cameras to Isaac Sim viewport windows.

    Args:
        viewport1_window: Optional viewport window object for Viewport 1.
            If None, attempts to get the default viewport automatically.
        viewport2_window: Optional name of the second viewport window
            (default: "Viewport 2").
    """
    viewport = get_viewport_from_window_name("Viewport")
    #print(f"Got viewport: {viewport}")
    # Set the viewport to the camera
    if viewport:
        viewport.set_active_camera("/World/open_manipulator_x/link5/Go2Camera")
    else:
        print("Failed to set Viewport 1")

    viewport2 = get_viewport_from_window_name("Viewport 2")
    #print(f"Got viewport: {viewport2}")
    # Set the viewport to the camera
    if viewport2:
        viewport2.set_active_camera("/World/Go2/Head_upper/Go2Camera")  
    else:
        print("Failed to set Viewport 2")
