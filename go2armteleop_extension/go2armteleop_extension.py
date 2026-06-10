# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of a copy at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Omniverse extension wrapper for the Go2-Arm teleoperation sample."""

import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from go2armteleop import Go2ArmExample


class Go2ArmExampleExtension(omni.ext.IExt):
    """Extension entry point. Registers the Go2-Arm sample in the Isaac Sim Examples Browser."""

    def on_startup(self, ext_id: str):
        self.example_name = "Go2 Arm Teleop v1"
        self.category = "A"

        overview = ("This Example shows a Unitree Go2 Robot with OpenManipulator-X arm "
                    "running a flat-terrain policy trained in Isaac Lab.")
        overview += "\n\nKeyboard Input:"
        overview += "\n  Numpad 8 / Up    - Move Forward"
        overview += "\n  Numpad 2 / Down  - Move Reverse"
        overview += "\n  Numpad 4 / Left   - Strafe Left"
        overview += "\n  Numpad 6 / Right  - Strafe Right"
        overview += "\n  Numpad 7 / N      - Spin Counterclockwise"
        overview += "\n  Numpad 9 / M      - Spin Clockwise"
        overview += "\n  A / a             - Pitch Up"
        overview += "\n  Z / z             - Pitch Down"
        overview += "\n  C / c             - Roll Left"
        overview += "\n  X / x             - Roll Right"
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Quadruped + Arm: Unitree Go2 with OpenManipulator-X",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": Go2ArmExample(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

        return
