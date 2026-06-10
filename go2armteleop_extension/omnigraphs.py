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

"""OmniGraph creation for the Go2-Arm teleoperation scene.

This module creates all OmniGraphs used for ROS2 bridge integration:
- cmd_vel subscriber
- Arm joint-state subscriber
- LiDAR 3D point cloud
- Odometry and TF
- Arm and quadruped camera feeds
- Simulation clock
"""

import omni.graph.core as og
from isaacsim.sensors.rtx import LidarRtx
from isaacsim.sensors.physx import _range_sensor


def _create_cmdvel_graph(controller: og.Controller) -> og.Controller:
    """Create the cmd_vel ROS2 subscriber OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.

    Returns:
        The controller for chaining (same instance).
    """
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
    return controller


def _create_arm_graph(controller: og.Controller) -> og.Controller:
    """Create the OpenManipulator-X joint-state subscriber OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
    return controller


def _create_lidar_graph(controller: og.Controller) -> og.Controller:
    """Create the RTX LiDAR 3D point cloud OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
    return controller


def _create_odometry_graph(controller: og.Controller) -> og.Controller:
    """Create the odometry and TF tree OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
    return controller


def _create_camera_arm_graph(controller: og.Controller, camera_arm_prim: str) -> og.Controller:
    """Create the arm camera ROS2 publish OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.
        camera_arm_prim: USD prim path of the arm camera.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
                ("isaac_create_render_product_cameraArm.inputs:cameraPrim", camera_arm_prim),
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
    return controller


def _create_camera_quadruped_graph(controller: og.Controller, camera_quad_prim: str) -> og.Controller:
    """Create the Go2 head camera ROS2 publish OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.
        camera_quad_prim: USD prim path of the Go2 head camera.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
                ("isaac_create_render_product_cameraQuadruped.inputs:cameraPrim", camera_quad_prim),
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
    return controller


def _create_clock_graph(controller: og.Controller) -> og.Controller:
    """Create the simulation clock ROS2 publish OmniGraph.

    Args:
        controller: The omni.graph.Controller instance.

    Returns:
        The controller for chaining (same instance).
    """
    keys = og.Controller.Keys
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
    return controller


def setup_omnigraphs(
    controller: og.Controller,
    camera_arm: object,
    camera_quadruped: object,
) -> dict:
    """Create all ROS2 bridge OmniGraphs for the Go2-Arm scene.

    This function creates seven OmniGraphs:
    1. /CMDVELGraph — cmd_vel subscriber
    2. /ArmGraph — OpenManipulator-X joint-state subscriber
    3. /LIDARGraph3D — RTX LiDAR point cloud
    4. /OdometryGraph — odometry and TF trees
    5. /CameraArmGraph — arm wrist camera ROS2 feed
    6. /CameraQuadrupedGraph — Go2 head camera ROS2 feed
    7. /SimTimeGraph — simulation clock publisher

    Args:
        controller: The omni.graph.Controller instance.
        camera_arm: The arm wrist Camera object (used to get its prim path).
        camera_quadruped: The Go2 head Camera object (used to get its prim path).

    Returns:
        dict with metadata including graph output attribute paths
        for accessing cmd_vel data in the physics callback.
    """
    # Create all 7 graphs
    _create_cmdvel_graph(controller)
    _create_arm_graph(controller)
    _create_lidar_graph(controller)
    _create_odometry_graph(controller)
    _create_camera_arm_graph(controller, camera_arm.prim_path)
    _create_camera_quadruped_graph(controller, camera_quadruped.prim_path)
    _create_clock_graph(controller)

    # Return metadata for the physics callback to access cmd_vel outputs
    return {
        "cmdvel_linear_x": "/CMDVELGraph/ros2_subscriber.outputs:linear:x",
        "cmdvel_linear_y": "/CMDVELGraph/ros2_subscriber.outputs:linear:y",
        "cmdvel_angular_z": "/CMDVELGraph/ros2_subscriber.outputs:angular:z",
        "cmdvel_angular_y": "/CMDVELGraph/ros2_subscriber.outputs:angular:y",
        "cmdvel_angular_x": "/CMDVELGraph/ros2_subscriber.outputs:angular:x",
    }
