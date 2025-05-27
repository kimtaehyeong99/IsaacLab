# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`OMY_CFG`: The OMY arm without a gripper.

Reference: https://github.com/ros-industrial/universal_robot
"""

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

usd_path = os.path.join(
    os.path.dirname(__file__),
    "../aiworker_usd/open_manipulator_y.usd"
)
usd_path = os.path.abspath(usd_path)

OMY_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 1.62,
            "joint3": -1.14,
            "joint4": 0.0,
            "joint5": 2.67,
            "joint6": -1.54,
            "rh_l1": 0.0,
            "rh_l2": 0.0,
            "rh_r1_joint": 0.0,
            "rh_r2": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)
"""Configuration of OMY arm using implicit actuator models."""