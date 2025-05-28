# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import stack_joint_pos_instance_randomize_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets import OMY_CFG  # isort: skip


@configclass
class OMYCubeStackInstanceRandomizeEnvCfg(
    stack_joint_pos_instance_randomize_env_cfg.OMYCubeStackInstanceRandomizeEnvCfg
):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set OMY as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = OMY_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Reduce the number of environments due to camera resources
        self.scene.num_envs = 2

        # Set actions for the specific robot type (OMY)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint[1-6]"],
            body_name="link6",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )
