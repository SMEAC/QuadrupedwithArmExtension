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
"""

import os
import sys

# Ensure the extension package directory is on sys.path so sibling subpackages
# (e.g. policy/) are importable.  Isaac Sim adds the search-path root to sys.path
# but does NOT automatically make subdirectories discoverable.
_package_root = os.path.dirname(os.path.abspath(__file__))
if _package_root not in sys.path:
    sys.path.insert(0, _package_root)

import carb
import numpy as np
import omni
import omni.appwindow
import omni.graph.core as og
import omni.physx.scripts.utils as physxUtils
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.prims import SingleArticulation
from isaacsim.examples.interactive.base_sample import BaseSample
from policy.go2withpitch import Go2withPitchFlatTerrainPolicy
from isaacsim.sensors.rtx import LidarRtx
from isaacsim.sensors.physx import _range_sensor
import isaacsim.sensors.physics
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera
from omni.kit.viewport.utility import (
    get_active_viewport,
    get_viewport_from_window_name,
    get_active_viewport_and_window,
    get_active_viewport_window,
)
from pxr import UsdGeom, Gf, UsdPhysics


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

        # OpenManipulator-X arm configuration
        self.arm_usd_path = "/home/gavin/isaacSimData/openManipulator/open_manipulator_x.usd"
        self.arm_prim_path = "/World/open_manipulator_x"
        self.arm_position = np.array([1.5, 0.0, 0.0])

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------
    def setup_scene(self) -> None:
        # Ground plane
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.3,
            dynamic_friction=0.3,
            restitution=0.01,
        )

        # Go2 robot with RL policy
        self.go2 = Go2withPitchFlatTerrainPolicy(
            prim_path="/World/Go2",
            root_path="/World/Go2/base",
            name="Go2",
            usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Unitree/Go2/go2.usd",
            position=np.array([0, 0, 0.5]),
        )

        # Timeline event to reset physics callback on play
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._timeline_timer_callback_fn
        )

        # ---- Load OpenManipulator-X arm ----
        stage_utils.add_reference_to_stage(self.arm_usd_path, self.arm_prim_path)
        self.arm = SingleArticulation(
            prim_path=self.arm_prim_path,
            name="open_manipulator_x",
            position=self.arm_position,
        )

        # ---- Attach arm to Go2 base with a fixed joint ----
        stage = omni.usd.get_context().get_stage()
        body0_path = "/World/Go2/base"
        body1_path = "/World/open_manipulator_x/world"

        joint_prim = physxUtils.createJoint(
            stage, "Fixed",
            stage.GetPrimAtPath(body0_path),
            stage.GetPrimAtPath(body1_path),
        )
        if joint_prim is not None:
            physics_joint = UsdPhysics.Joint(joint_prim)
            physics_joint.GetLocalPos0Attr().Set(Gf.Vec3f([0.2, 0.0, 0.07]))
            print(f"[Go2Arm] Fixed joint created between {body0_path} and {body1_path}")

        # ---- Arm-mounted camera ----
        cameraArm_position = np.array([0.04, 0.0, 0.06])
        from scipy.spatial.transform import Rotation as R
        quat_scipy = R.from_euler("yxz", np.array([0.0, 73.0, -90]), degrees=True).as_quat()
        cameraArm_orientation = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])
        cameraArm = Camera(
            prim_path="/World/open_manipulator_x/link5/Go2Camera",
            position=cameraArm_position,
            orientation=cameraArm_orientation,
            resolution=(512, 512),
        )
        cameraArm.set_focal_length(1.0)
        cameraArm.set_clipping_range(near_distance=0.01, far_distance=10)
        cameraArm.set_local_pose(translation=cameraArm_position, orientation=cameraArm_orientation, camera_axes="usd")

        # ---- Go2 head camera ----
        cameraQuadruped_position = np.array([0.04, 0.0, 0.017])
        quat_scipy = R.from_euler("yxz", np.array([0.0, 90, -90]), degrees=True).as_quat()
        cameraQuadruped_orientation = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])
        cameraQuadruped = Camera(
            prim_path="/World/Go2/Head_upper/Go2Camera",
            position=cameraQuadruped_position,
            orientation=cameraQuadruped_orientation,
            resolution=(512, 512),
        )
        cameraQuadruped.set_focal_length(1.0)
        cameraQuadruped.set_clipping_range(near_distance=0.01, far_distance=10)
        cameraQuadruped.set_local_pose(translation=cameraQuadruped_position, orientation=cameraQuadruped_orientation, camera_axes="usd")

        # ---- Set viewports to cameras ----
        viewport = get_viewport_from_window_name("Viewport")
        if viewport:
            viewport.set_active_camera("/World/open_manipulator_x/link5/Go2Camera")
        else:
            print("[Go2Arm] Failed to set Viewport 1 camera")

        viewport2 = get_viewport_from_window_name("Viewport 2")
        if viewport2:
            viewport2.set_active_camera("/World/Go2/Head_upper/Go2Camera")
        else:
            print("[Go2Arm] Failed to set Viewport 2 camera")

        # ---- RTX LiDAR ----
        sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 60}
        sensor = LidarRtx(
            prim_path="/World/Go2/radar/lidar3D",
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.9925462, 0.0, 0.1218693, 0.0]),
            config_file_name="Example_Rotary",
            **sensor_attributes,
        )

        lidarPath = "/Lidar"
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidarPath,
            parent="/World/Go2/radar",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
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

        # ---- OmniGraph: cmd_vel subscriber ----
        controller = og.Controller()
        keys = og.Controller.Keys

        controller.edit(
            {"graph_path": "/CMDVELGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ],
                keys.SET_VALUES: [
                    ("ros2_subscriber.inputs:messageName", "Twist"),
                    ("ros2_subscriber.inputs:messagePackage", "geometry_msgs"),
                    ("ros2_subscriber.inputs:topicName", "cmd_vel"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ros2_subscriber.inputs:execIn"),
                    ("Ros2Context.outputs:context", "ros2_subscriber.inputs:context"),
                ],
            },
        )

        # ---- OmniGraph: arm joint-state subscriber ----
        controller.edit(
            {"graph_path": "/ArmGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickArm", "omni.graph.action.OnPlaybackTick"),
                    ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ROS2ContextArm", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ROS2SubscriberArm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ToString", "omni.graph.nodes.ToString"),
                    ("print_text", "omni.graph.ui_nodes.PrintText"),
                ],
                keys.SET_VALUES: [
                    ("ROS2SubscriberArm.inputs:topicName", "joint_states"),
                    ("print_text.inputs:toScreen", True),
                    ("articulation_controller.inputs:targetPrim", "/World/open_manipulator_x/joints/joint1"),
                ],
                keys.CONNECT: [
                    ("ROS2ContextArm.outputs:context", "ROS2SubscriberArm.inputs:context"),
                    ("ROS2SubscriberArm.outputs:effortCommand", "articulation_controller.inputs:effortCommand"),
                    ("ROS2SubscriberArm.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
                    ("ROS2SubscriberArm.outputs:jointNames", "articulation_controller.inputs:jointNames"),
                    ("ROS2SubscriberArm.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("ROS2SubscriberArm.outputs:execOut", "articulation_controller.inputs:execIn"),
                    ("ROS2SubscriberArm.outputs:execOut", "print_text.inputs:execIn"),
                    ("OnPlaybackTickArm.outputs:tick", "ROS2SubscriberArm.inputs:execIn"),
                ],
            },
        )

        # ---- OmniGraph: LiDAR 3D ----
        controller.edit(
            {"graph_path": "/LIDARGraph3D", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickLaser", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("Ros2ContextLaser", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ros2_rtx_lidar_helper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.SET_VALUES: [
                    ("isaac_create_render_product.inputs:height", 640),
                    ("isaac_create_render_product.inputs:width", 480),
                    ("isaac_create_render_product.inputs:cameraPrim", "/World/Go2/radar/lidar3D"),
                    ("ros2_rtx_lidar_helper.inputs:type", "point_cloud"),
                    ("ros2_rtx_lidar_helper.inputs:frameId", "radar"),
                ],
                keys.CONNECT: [
                    ("Ros2ContextLaser.outputs:context", "ros2_rtx_lidar_helper.inputs:context"),
                    ("OnPlaybackTickLaser.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                    ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                    ("isaac_create_render_product.outputs:execOut", "ros2_rtx_lidar_helper.inputs:execIn"),
                    ("isaac_create_render_product.outputs:renderProductPath", "ros2_rtx_lidar_helper.inputs:renderProductPath"),
                ],
            },
        )

        # ---- OmniGraph: Odometry & TF ----
        controller.edit(
            {"graph_path": "/OdometryGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickOdo", "omni.graph.action.OnPlaybackTick"),
                    ("Ros2ContextOdo", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_read_simulation_time_odo", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_trans_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("isaac_compute_odometry_node", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("ros2_pub_odometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("ros2_pub_raw_tf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("ros2_pub_raw_tf_sim_lidar", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ros2_pub_trans_tree.inputs:targetPrims", [
                        "/World/Go2/base", "/World/Go2/RL_foot", "/World/Go2/RL_calf",
                        "/World/Go2/RL_hip", "/World/Go2/RL_thigh", "/World/Go2/FL_foot",
                        "/World/Go2/FL_calf", "/World/Go2/FL_hip", "/World/Go2/FL_thigh",
                        "/World/Go2/RR_foot", "/World/Go2/RR_calf", "/World/Go2/RR_hip",
                        "/World/Go2/RR_thigh", "/World/Go2/FR_foot", "/World/Go2/FR_calf",
                        "/World/Go2/FR_hip", "/World/Go2/FR_thigh", "/World/Go2/Head_upper",
                        "/World/Go2/Head_lower", "/World/Go2/imu", "/World/Go2/radar",
                        "/World/open_manipulator_x/link1", "/World/open_manipulator_x/link2",
                        "/World/open_manipulator_x/link3", "/World/open_manipulator_x/link4",
                        "/World/open_manipulator_x/link5", "/World/open_manipulator_x/gripper_left_link",
                        "/World/open_manipulator_x/gripper_right_link",
                    ]),
                    ("isaac_compute_odometry_node.inputs:chassisPrim", "/World/Go2/base"),
                    ("ros2_pub_raw_tf.inputs:childFrameId", "base"),
                    ("ros2_pub_raw_tf.inputs:parentFrameId", "odom"),
                    ("ros2_pub_raw_tf_sim_lidar.inputs:childFrameId", "sim_lidar"),
                    ("ros2_pub_raw_tf_sim_lidar.inputs:parentFrameId", "radar"),
                    ("ros2_pub_raw_tf_sim_lidar.inputs:staticPublisher", True),
                    ("ros2_pub_odometry.inputs:chassisFrameId", "base"),
                    ("ros2_pub_trans_tree.inputs:parentPrim", "/World/Go2/base"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTickOdo.outputs:tick", "ros2_pub_trans_tree.inputs:execIn"),
                    ("Ros2ContextOdo.outputs:context", "ros2_pub_trans_tree.inputs:context"),
                    ("isaac_read_simulation_time_odo.outputs:simulationTime", "ros2_pub_trans_tree.inputs:timeStamp"),
                    ("isaac_compute_odometry_node.outputs:execOut", "ros2_pub_odometry.inputs:execIn"),
                    ("OnPlaybackTickOdo.outputs:tick", "isaac_compute_odometry_node.inputs:execIn"),
                    ("isaac_compute_odometry_node.outputs:angularVelocity", "ros2_pub_odometry.inputs:angularVelocity"),
                    ("isaac_compute_odometry_node.outputs:linearVelocity", "ros2_pub_odometry.inputs:linearVelocity"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_pub_odometry.inputs:orientation"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_pub_odometry.inputs:position"),
                    ("isaac_read_simulation_time_odo.outputs:simulationTime", "ros2_pub_odometry.inputs:timeStamp"),
                    ("Ros2ContextOdo.outputs:context", "ros2_pub_odometry.inputs:context"),
                    ("OnPlaybackTickOdo.outputs:tick", "ros2_pub_raw_tf.inputs:execIn"),
                    ("isaac_compute_odometry_node.outputs:orientation", "ros2_pub_raw_tf.inputs:rotation"),
                    ("isaac_compute_odometry_node.outputs:position", "ros2_pub_raw_tf.inputs:translation"),
                    ("isaac_read_simulation_time_odo.outputs:simulationTime", "ros2_pub_raw_tf.inputs:timeStamp"),
                    ("OnPlaybackTickOdo.outputs:tick", "ros2_pub_raw_tf_sim_lidar.inputs:execIn"),
                    ("isaac_read_simulation_time_odo.outputs:simulationTime", "ros2_pub_raw_tf_sim_lidar.inputs:timeStamp"),
                    ("Ros2ContextOdo.outputs:context", "ros2_pub_raw_tf_sim_lidar.inputs:context"),
                ],
            },
        )

        # ---- OmniGraph: Arm camera ----
        controller.edit(
            {"graph_path": "/CameraArmGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickCameraArm", "omni.graph.action.OnPlaybackTick"),
                    ("Ros2ContextCameraArm", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_run_one_simulation_frame_cameraArm", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("isaac_create_render_product_cameraArm", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ros2_cameraArm_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("isaac_create_render_product_cameraArm.inputs:cameraPrim", "/World/open_manipulator_x/link5/Go2Camera"),
                    ("ros2_cameraArm_helper.inputs:frameId", "link5"),
                    ("ros2_cameraArm_helper.inputs:topicName", "wrist_camera"),
                    ("ros2_cameraArm_helper.inputs:type", "rgb"),
                    ("isaac_create_render_product_cameraArm.inputs:height", 512),
                    ("isaac_create_render_product_cameraArm.inputs:width", 512),
                ],
                keys.CONNECT: [
                    ("Ros2ContextCameraArm.outputs:context", "ros2_cameraArm_helper.inputs:context"),
                    ("OnPlaybackTickCameraArm.outputs:tick", "isaac_run_one_simulation_frame_cameraArm.inputs:execIn"),
                    ("isaac_run_one_simulation_frame_cameraArm.outputs:step", "isaac_create_render_product_cameraArm.inputs:execIn"),
                    ("isaac_create_render_product_cameraArm.outputs:execOut", "ros2_cameraArm_helper.inputs:execIn"),
                    ("isaac_create_render_product_cameraArm.outputs:renderProductPath", "ros2_cameraArm_helper.inputs:renderProductPath"),
                ],
            },
        )

        # ---- OmniGraph: Quadruped camera ----
        controller.edit(
            {"graph_path": "/CameraQuadrupedGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickCameraQuadruped", "omni.graph.action.OnPlaybackTick"),
                    ("Ros2ContextCameraQuadruped", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_run_one_simulation_frame_cameraQuadruped", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("isaac_create_render_product_cameraQuadruped", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ros2_cameraQuadruped_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
                    ("isaac_create_render_product_cameraQuadruped.inputs:cameraPrim", "/World/Go2/Head_upper/Go2Camera"),
                    ("ros2_cameraQuadruped_helper.inputs:frameId", "Head_upper"),
                    ("ros2_cameraQuadruped_helper.inputs:topicName", "rgb"),
                    ("ros2_cameraQuadruped_helper.inputs:type", "rgb"),
                    ("isaac_create_render_product_cameraQuadruped.inputs:height", 512),
                    ("isaac_create_render_product_cameraQuadruped.inputs:width", 512),
                ],
                keys.CONNECT: [
                    ("Ros2ContextCameraQuadruped.outputs:context", "ros2_cameraQuadruped_helper.inputs:context"),
                    ("OnPlaybackTickCameraQuadruped.outputs:tick", "isaac_run_one_simulation_frame_cameraQuadruped.inputs:execIn"),
                    ("isaac_run_one_simulation_frame_cameraQuadruped.outputs:step", "isaac_create_render_product_cameraQuadruped.inputs:execIn"),
                    ("isaac_create_render_product_cameraQuadruped.outputs:execOut", "ros2_cameraQuadruped_helper.inputs:execIn"),
                    ("isaac_create_render_product_cameraQuadruped.outputs:renderProductPath", "ros2_cameraQuadruped_helper.inputs:renderProductPath"),
                ],
            },
        )

        # ---- OmniGraph: Clock ----
        controller.edit(
            {"graph_path": "/SimTimeGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTickClock", "omni.graph.action.OnPlaybackTick"),
                    ("Ros2ContextClock", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_read_simulation_time_clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ros2_pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("Ros2ContextClock.outputs:context", "ros2_pub_clock.inputs:context"),
                    ("OnPlaybackTickClock.outputs:tick", "ros2_pub_clock.inputs:execIn"),
                    ("isaac_read_simulation_time_clock.outputs:simulationTime", "ros2_pub_clock.inputs:timeStamp"),
                ],
            },
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._physics_ready = False
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
            self._newCommand = [
                og.Controller.attribute("/CMDVELGraph/ros2_subscriber.outputs:linear:x").get(),
                og.Controller.attribute("/CMDVELGraph/ros2_subscriber.outputs:linear:y").get(),
                og.Controller.attribute("/CMDVELGraph/ros2_subscriber.outputs:angular:z").get(),
                og.Controller.attribute("/CMDVELGraph/ros2_subscriber.outputs:angular:y").get(),
                og.Controller.attribute("/CMDVELGraph/ros2_subscriber.outputs:angular:x").get(),
            ]
            self._merged_command = [x + y for x, y in zip(self._newCommand, self._base_command)]
            self.go2.forward(step_size, self._merged_command)
        else:
            self._physics_ready = True
            self.go2.initialize(physics_sim_view="/World/go2")
            print(f"[Go2Arm] Go2 articulation bodies: {self.go2.robot.get_articulation_body_count()}")
            self.go2.post_reset()
            self.go2.robot.set_joints_default_state(self.go2.default_pos)
            self.arm.initialize()

    def _sub_keyboard_event(self, event: any, *args: any, **kwargs: any) -> bool:
        """Keyboard event subscriber: accumulates on press, releases on key-up."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True

    def _timeline_timer_callback_fn(self, event: any) -> None:
        if self.go2:
            self._physics_ready = False
        if not self.get_world().physics_callback_exists("physics_step"):
            self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)

    def world_cleanup(self) -> None:
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")
