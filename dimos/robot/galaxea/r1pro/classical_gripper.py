# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""R1Pro physical finger sweep expressed in GraspGenX's +Z approach convention."""

from dimos.manipulation.grasping.grasp_gen_x.module import RigidTransform, SweepVolumeGripperConfig

# The sim's parallel jaws close on palm Y and approach along palm -Z. Both
# planning TCPs are 85 mm along palm -Z. Swapping X/Y and flipping Z is a proper
# rotation; it does not mirror a grasp or reverse which way the hand approaches.
R1PRO_GRASP_FRAME_TO_TCP: RigidTransform = (
    (0.0, 1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.085),
    (0.0, 0.0, 0.0, 1.0),
)

# Filled from the finger-pad geometry of grasping_sim, not another robot's jaw.
# Sweep volume means the region between the inner contact faces, not the entire
# gripper's collision bounds. Full hand/scene collision validation remains required.
R1PRO_GRIPPER_SWEEP = SweepVolumeGripperConfig(
    extents_open=(0.1, 0.018, 0.035),
    offset_open=(0.0, 0.0, 0.085),
    extents_half_open=(0.05, 0.018, 0.035),
    offset_half_open=(0.0, 0.0, 0.085),
    fingertip_depth=0.1025,
)
