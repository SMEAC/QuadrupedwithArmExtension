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

"""Scene object creation for the Go2-Arm teleoperation scene.

This module creates all scene objects: ground plane, Go2 robot with RL policy,
OpenManipulator-X arm with fixed joint, RTX LiDAR, tennis ball, and range sensor.
Camera objects are returned as raw configuration data to avoid circular imports.
"""

import os
import numpy as np
import omni
import omni.usd
import omni.kit.commands
import omni.physx.scripts.utils as physxUtils
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.prims import SingleArticulation
from isaacsim.sensors.rtx import LidarRtx
from isaacsim.sensors.physx import _range_sensor
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera
from pxr import UsdGeom, Sdf, Gf, UsdPhysics
from policy.go2withpitch import Go2withPitchFlatTerrainPolicy
from isaacsim.core.experimental.prims import RigidPrim



def _get_extension_root() -> str:
    """Return the root directory of the go2armteleop_extension package."""
    return os.path.dirname(os.path.abspath(__file__))


def _find_xformable_prim(prim):
    """Return the first valid leaf prim that has a ``xformOpOrder`` attribute.

    ``xformOpOrder`` is the stable attribute that signals a USDGeom Xform;
    it works across all USD versions we target, unlike ``IsXformable()``.
    """
    if prim and prim.IsValid():
        if prim.HasAttribute("xformOpOrder"):
            return prim
        for child in prim.GetChildren():
            child_xform = _find_xformable_prim(child)
            if child_xform is not None:
                return child_xform
        return None
    return None


# ---------------------------------------------------------------------------
# Individual scene object creators
# ---------------------------------------------------------------------------

def create_ground_plane(world) -> None:
    """Add the default ground plane to the scene.

    Args:
        world: The Isaac Sim world instance.
    """
    world.scene.add_default_ground_plane(
        z_position=0,
        name="default_ground_plane",
        prim_path="/World/defaultGroundPlane",
        static_friction=0.3,
        dynamic_friction=0.3,
        restitution=0.01,
    )


def create_go2_robot(
    world,
    prim_path: str = "/World/Go2",
    root_path: str = "/World/Go2/base",
    name: str = "Go2",
    position: np.ndarray | None = None,
    usd_path: str | None = None,
) -> Go2withPitchFlatTerrainPolicy:
    """Create the Go2 quadruped robot with RL policy.

    Args:
        world: The Isaac Sim world instance.
        prim_path: USD prim path for the robot.
        root_path: USD prim path for the articulation root.
        name: Name of the quadruped.
        position: Initial position [x, y, z].
        usd_path: USD file path for the Go2 robot model.

    Returns:
        The initialized Go2withPitchFlatTerrainPolicy instance.
    """
    if position is None:
        position = np.array([0, 0, 0.5])

    return Go2withPitchFlatTerrainPolicy(
        prim_path=prim_path,
        root_path=root_path,
        name=name,
        usd_path=usd_path,
        position=position,
    )


def create_open_manipulator_x(
    stage,
    arm_usd_path: str,
    arm_prim_path: str = "/World/open_manipulator_x",
    position: np.ndarray | None = None,
) -> SingleArticulation:
    """Load the OpenManipulator-X arm USD and create its articulation.

    Args:
        stage: The current USD stage.
        arm_usd_path: Path to the OpenManipulator-X USD file.
        arm_prim_path: USD prim path for the arm.
        position: Initial position [x, y, z] on the Go2 base.

    Returns:
        The SingleArticulation instance for the arm.
    """
    stage_utils.add_reference_to_stage(arm_usd_path, arm_prim_path)

    return SingleArticulation(
        prim_path=arm_prim_path,
        name="open_manipulator_x",
        position=position,
    )


def create_ball(
    stage,
    ball_prim_path: str = "/World/ball",
    position: np.ndarray | None = None,
) -> str:
    """Load the tennis ball USD into the scene.

    The USD file is resolved relative to the extension root directory
    via ``_get_extension_root()`` so the function works regardless of
    the working directory from which Isaac Sim is launched.

    Args:
        stage: The current USD stage.
        ball_prim_path: USD prim path for the ball.
        position: Initial position [x, y, z].

    Returns:
        The USD prim path of the ball.
    """
    if position is None:
        position = np.array([1.0, 1.0, 1.5])

    # Resolve the relative resource path against the extension root.
    _ball_usd_rel = "resource/Tennis_ball_01.usda"
    ball_usd_abs = os.path.join(_get_extension_root(), _ball_usd_rel)
    print(f"[Go2Arm] Placing Ball")
    stage_utils.add_reference_to_stage(ball_usd_abs, ball_prim_path)
    print(f"[Go2Arm] Ball placed")
    # Apply position to the referenced ball prim if possible.
    ball_prim = stage.GetPrimAtPath(ball_prim_path)
    if ball_prim and ball_prim.IsValid():
        try:
            xformable_prim = _find_xformable_prim(ball_prim)
            if xformable_prim is not None:
                xform = UsdGeom.XformCommonAPI(xformable_prim)
                xform.SetTranslate(Gf.Vec3d(*position))
                #linear_vel = np.array([0.5, 0.0, 0.0]) # Move 0.5 m/s along X-axis
                #xform.set_linear_velocity(linear_vel)
                print(f"[Go2Arm] Ball placed at {ball_prim_path}")
            else:
                print(f"[Go2Arm] Warning: no xformable prim found for ball at {ball_prim_path}")
        except Exception as e:
            print(f"[Go2Arm] Warning: failed to set ball position: {e}")
    else:
        print(f"[Go2Arm] Warning: ball prim not found at {ball_prim_path}")
    
    
    return ball_prim_path

def create_basket(
    stage,
    basket_prim_path: str = "/World/basket",
    position: np.ndarray | None = None,
) -> str:
    """Load the basket USD into the scene.

    The USD file is resolved relative to the extension root directory
    via ``_get_extension_root()`` so the function works regardless of
    the working directory from which Isaac Sim is launched.

    Args:
        stage: The current USD stage.
        basket_prim_path: USD prim path for the basket.
        position: Initial position [x, y, z].

    Returns:
        The USD prim path of the basket.
    """
    if position is None:
        position = np.array([1.0, 1.0, 1.5])

    # Load basket from NVIDIA OCP URL.
    basket_url = (
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Props/KLT_Bin/small_KLT.usd"
    )
    print(f"[Go2Arm] Placing Basket from {basket_url}")
    stage_utils.add_reference_to_stage(basket_url, basket_prim_path)
    print(f"[Go2Arm] Basket placed")
    # Apply position to the referenced basket prim if possible.
    basket_prim = stage.GetPrimAtPath(basket_prim_path)
    if basket_prim and basket_prim.IsValid():
        try:
            xformable_prim = _find_xformable_prim(basket_prim)
            if xformable_prim is not None:
                xform = UsdGeom.XformCommonAPI(xformable_prim)
                xform.SetTranslate(Gf.Vec3d(*position))
                #linear_vel = np.array([0.5, 0.0, 0.0]) # Move 0.5 m/s along X-axis
                #xform.set_linear_velocity(linear_vel)
                print(f"[Go2Arm] Basket placed at {basket_prim_path}")
            else:
                print(f"[Go2Arm] Warning: no xformable prim found for basket at {basket_prim_path}")
        except Exception as e:
            print(f"[Go2Arm] Warning: failed to set basket position: {e}")
    else:
        print(f"[Go2Arm] Warning: basket prim not found at {basket_prim_path}")
    
    
    return basket_prim_path

def create_gripper(
    stage,
    gripper_prim_path: str = "/World/gripper_left",
    scale: np.ndarray | None = None,
) -> str:
    """Load the gripper USD into the scene.

    The USD file is resolved relative to the extension root directory
    via ``_get_extension_root()`` so the function works regardless of
    the working directory from which Isaac Sim is launched.

    Args:
        stage: The current USD stage.
        gripper_prim_path: USD prim path for the gripper.
        scale: Initial scale [x, y, z].

    Returns:
        The USD prim path of the gripper.
    """
    if scale is None:
        scale = np.array([1.5, 1.5, 1.5])



    # Resolve the relative resource path against the extension root.
    _gripper_usd_rel = "resource/Gripper_Ball_v3.usda"
    gripper_usd_abs = os.path.join(_get_extension_root(), _gripper_usd_rel)
    print(f"[Go2Arm] Gripper placed")
    stage_utils.add_reference_to_stage(gripper_usd_abs, gripper_prim_path)
    print(f"[Go2Arm] Gripper placed")
    # Apply position to the referenced gripper prim if possible.
    gripper_prim = stage.GetPrimAtPath(gripper_prim_path)
    if gripper_prim and gripper_prim.IsValid():
        try:
            xformable_prim = _find_xformable_prim(gripper_prim)
            if xformable_prim is not None:
                xform = UsdGeom.XformCommonAPI(xformable_prim)
                xform.SetScale(Gf.Vec3f(*scale))
                
            else:
                print(f"[Go2Arm] Warning: at Scale  {gripper_prim_path}")
        except Exception as e:
            print(f"[Go2Arm] Warning: failed to set gripper position: {e}")
    else:
        print(f"[Go2Arm] Warning: gripper prim not found at {gripper_prim_path}")

    prim = RigidPrim(gripper_prim_path, masses=[0.001])

    '''
    child_prim_path = f"{gripper_prim_path}/node_/mesh_"
    child_prim = stage.GetPrimAtPath(child_prim_path)

    if child_prim.IsValid():
        # 3. Cast the prim to a UsdGeom.Xformable so we can modify its transform stack
        xformable = UsdGeom.Xformable(child_prim)
        
        # 4. Set the reset flag to True (this clears parent transform inheritance for physics)
        xformable.SetResetXformStack(True)
    '''    
    return gripper_prim_path

def create_fixed_joint(stage, body0_path: str, body1_path: str, offset: np.ndarray, rot: np.ndarray) -> None:
    """Create a fixed joint connecting two body prims.

    Args:
        stage: The current USD stage.
        body0_path: USD prim path of the parent body (Go2 base).
        body1_path: USD prim path of the child body (arm world).
        offset: Translation offset [x, y, z] for the joint.
        rot: Rotation offset [x, y, z] for the joint.
    """
    joint_prim = physxUtils.createJoint(
        stage, "Fixed",
        stage.GetPrimAtPath(body0_path),
        stage.GetPrimAtPath(body1_path),
    )
    if offset is None:
        offset = np.array([0.0, 0.0, 0.0])
    if rot is None:
        rot = np.array([0.0, 0.0, 0.0])

    from scipy.spatial.transform import Rotation as R

    quat_scipy = R.from_euler("xyz", rot, degrees=True).as_quat()
    orientation = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])

    if joint_prim is not None:
        physics_joint = UsdPhysics.Joint(joint_prim)
        physics_joint.GetLocalPos0Attr().Set(Gf.Vec3f(offset[0], offset[1], offset[2]))
        physics_joint.GetLocalRot0Attr().Set(Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3]))
        print(f"[Go2Arm] Fixed joint created between {body0_path} and {body1_path}")
    joint_path = joint_prim.GetPath()
    print(f"[Go2Arm] Fixed joint created at {joint_path}")

def create_rtx_lidar(
    prim_path: str = "/World/Go2/radar/lidar3D",
    translation: np.ndarray | None = None,
    orientation: np.ndarray | None = None,
) -> LidarRtx:
    """Create the RTX LiDAR sensor on the Go2 base.

    Args:
        prim_path: USD prim path for the LiDAR sensor.
        translation: Translation offset [x, y, z].
        orientation: Orientation quaternion [w, x, y, z].

    Returns:
        The configured LidarRtx instance.
    """
    if translation is None:
        translation = np.array([0.0, 0.0, 0.0])
    if orientation is None:
        orientation = np.array([0.9925462, 0.0, 0.1218693, 0.0])

    sensor_attributes = {"omni:sensor:Core:scanRateBaseHz": 60}
    return LidarRtx(
        prim_path=prim_path,
        translation=translation,
        orientation=orientation,
        config_file_name="Example_Rotary",
        **sensor_attributes,
    )


def create_range_sensor(
    lidar_path: str = "/Lidar",
    parent_path: str = "/World/Go2/radar",
    min_range: float = 0.4,
    max_range: float = 100.0,
) -> tuple[bool, object]:
    """Create a range sensor for the LiDAR.

    Args:
        lidar_path: USD prim path for the range sensor.
        parent_path: USD prim path of the parent object.
        min_range: Minimum detection range in meters.
        max_range: Maximum detection range in meters.

    Returns:
        Tuple of (success_flag, created_prim).
    """
    result, prim = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=lidar_path,
        parent=parent_path,
        min_range=min_range,
        max_range=max_range,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=4.0,
        rotation_rate=0.0,
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )
    return result, prim


def create_camera_obj(
    prim_path: str,
    parent_link: str,
    offset: np.ndarray,
    euler_deg: np.ndarray,
    resolution: tuple[int, int] = (512, 512),
) -> Camera:
    """Create a Camera USD prim and return the Camera object.

    This function is kept in scene.py to avoid circular imports with cameras.py.
    The main file will pass these Camera objects to cameras.setup_cameras_viewports().

    Args:
        prim_path: USD prim path for the camera.
        parent_link: USD prim path of the parent link/body.
        offset: Translation offset from the parent link origin [x, y, z].
        euler_deg: Euler angles (yaw, pitch, roll) in degrees for camera orientation.
        resolution: Camera resolution as (width, height).

    Returns:
        Configured Camera instance.
    """
    from scipy.spatial.transform import Rotation as R

    camera_position = offset
    quat_scipy = R.from_euler("yxz", euler_deg, degrees=True).as_quat()
    camera_orientation = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])

    camera = Camera(
        prim_path=prim_path,
        position=camera_position,
        orientation=camera_orientation,
        resolution=resolution,
    )
    camera.set_focal_length(1.0)
    camera.set_clipping_range(near_distance=0.01, far_distance=10.0)
    camera.set_local_pose(
        translation=camera_position,
        orientation=camera_orientation,
        camera_axes="usd",
    )
    return camera


# ---------------------------------------------------------------------------
# Camera configuration constants (used by create_camera_obj)
# ---------------------------------------------------------------------------

CAMERA_ARM_CONFIG = {
    "prim_path": "/World/open_manipulator_x/link5/Go2Camera",
    "parent_link": "link5",
    "offset": np.array([0.04, 0.0, 0.06]),
    "euler_deg": np.array([0.0, 73.0, -90]),
    "resolution": (512, 512),
}

CAMERA_QUADRUPED_CONFIG = {
    "prim_path": "/World/Go2/Head_upper/Go2Camera",
    "parent_link": "Head_upper",
    "offset": np.array([0.04, 0.0, 0.017]),
    "euler_deg": np.array([0.0, 90.0, -90]),
    "resolution": (512, 512),
}


# ---------------------------------------------------------------------------
# Top-level scene setup
# ---------------------------------------------------------------------------

def setup_scene(
    world,
    arm_usd_path: str | None = None,
    arm_prim_path: str | None = None,
    arm_position: np.ndarray | None = None,
) -> dict:
    """Create all scene objects for the Go2-Arm teleoperation scene.

    This is the top-level function that orchestrates creation of:
    - Ground plane
    - Go2 robot with RL policy
    - OpenManipulator-X arm with fixed joint
    - Tennis ball
    - Two cameras (arm wrist + Go2 head)
    - RTX LiDAR with range sensor

    Args:
        world: The Isaac Sim world instance.
        arm_usd_path: Path to the OpenManipulator-X USD file. Defaults to ARM_USD_PATH.
        arm_prim_path: USD prim path for the arm. Defaults to ARM_PRIM_PATH.
        arm_position: Position on the Go2 base where the arm is mounted [x, y, z].

    Returns:
        dict containing references to all created objects with keys:
        - 'go2': Go2withPitchFlatTerrainPolicy instance
        - 'arm': SingleArticulation instance
        - 'ball': str (USD prim path of the tennis ball)
        - 'camera_arm': Camera instance (arm wrist)
        - 'camera_quadruped': Camera instance (Go2 head)
        - 'lidar': LidarRtx instance
        - 'range_result': Range sensor creation result (bool)
        - 'range_prim': Range sensor USD prim
        - 'camera_arm_path': Camera arm prim path (str)
        - 'camera_quad_path': Camera quadruped prim path (str)
        - 'event_stream': Timeline event stream for play events
    """
    # Ground plane
    create_ground_plane(world)

    # Go2 robot with RL policy
    go2 = create_go2_robot(
        world=world,
        prim_path="/World/Go2",
        root_path="/World/Go2/base",
        name="Go2",
        position=np.array([0, 0, 0.5]),
        usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Unitree/Go2/go2.usd",
    )

    # Timeline event to reset physics callback on play
    timeline = omni.timeline.get_timeline_interface()
    event_stream = timeline.get_timeline_event_stream()

    # OpenManipulator-X arm
    stage = omni.usd.get_context().get_stage()

    arm_usd_path = arm_usd_path if arm_usd_path is not None else "/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd"
    arm_prim_path = arm_prim_path if arm_prim_path is not None else "/World/open_manipulator_x"
    arm_position = arm_position if arm_position is not None else np.array([1.5, 0.0, 0.0])
    arm = create_open_manipulator_x(stage, arm_usd_path, arm_prim_path, arm_position)

    # Fixed joint connecting arm to Go2 base
    create_fixed_joint(stage, "/World/Go2/base", 
                              "/World/open_manipulator_x/world",
                              offset=np.array([0.2, 0.0, 0.07]),
                              rot=np.array([0.0, 0.0, 0.0]))

    gripper_left_prim_path = create_gripper(stage, gripper_prim_path="/World/gripper_left", scale=np.array([1.5, 1.5, 1.5]))
    gripper_right_prim_path = create_gripper(stage, gripper_prim_path="/World/gripper_right", scale=np.array([1.5, 1.5, 1.5]))
    create_fixed_joint(stage, "/World/open_manipulator_x/gripper_left_link", 
                              "/World/gripper_left", 
                              offset=np.array([0.15, 0.045, -0.045]), 
                              rot=np.array([90.0, 0.0, -90.0]))
                              #rot=np.array([180.0, 90.0, 90.0]))

    create_fixed_joint(stage, "/World/open_manipulator_x/gripper_right_link", 
                              "/World/gripper_right", 
                              offset=np.array([0.15, -0.045, 0.045]), 
                              rot=np.array([-90.0, 0.0, 90.0]))
    # Tennis ball
    ball_prim_path = create_ball(stage, ball_prim_path="/World/ball", position=np.array([2.0, 1.0, 1.5]))

    # Objects
    basket_prim_path = create_basket(stage, basket_prim_path="/World/basket", position=np.array([2.0, -1.0, 0.5]))

    # Arm-mounted camera
    camera_arm = create_camera_obj(
        prim_path=CAMERA_ARM_CONFIG["prim_path"],
        parent_link=CAMERA_ARM_CONFIG["parent_link"],
        offset=CAMERA_ARM_CONFIG["offset"],
        euler_deg=CAMERA_ARM_CONFIG["euler_deg"],
        resolution=CAMERA_ARM_CONFIG["resolution"],
    )

    # Go2 head camera
    camera_quadruped = create_camera_obj(
        prim_path=CAMERA_QUADRUPED_CONFIG["prim_path"],
        parent_link=CAMERA_QUADRUPED_CONFIG["parent_link"],
        offset=CAMERA_QUADRUPED_CONFIG["offset"],
        euler_deg=CAMERA_QUADRUPED_CONFIG["euler_deg"],
        resolution=CAMERA_QUADRUPED_CONFIG["resolution"],
    )

    # RTX LiDAR
    lidar = create_rtx_lidar()

    # Range sensor for LiDAR
    range_result, range_prim = create_range_sensor()

    return {
        "go2": go2,
        "arm": arm,
        "ball": ball_prim_path,
        "camera_arm": camera_arm,
        "camera_quadruped": camera_quadruped,
        "lidar": lidar,
        "range_result": range_result,
        "range_prim": range_prim,
        "camera_arm_path": camera_arm.prim_path,
        "camera_quad_path": camera_quadruped.prim_path,
        "event_stream": event_stream,
    }
