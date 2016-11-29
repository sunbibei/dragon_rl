""" Hyperparameters for The Single Leg of Dragon HOPPING task with PI2."""
from __future__ import division

import os.path
from datetime import datetime
import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.agent_ros import AgentROS # About Agent Package

from gps.algorithm.algorithm_traj_opt_pi2 import AlgorithmTrajOptPi2
from gps.algorithm.cost.cost_dz import CostDZ
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_CONSTANT, RAMP_LINEAR, RAMP_QUADRATIC, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_pi2 import TrajOptPi2
from gps.algorithm.policy.lin_gauss_init import init_pd
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info

EE_POINTS = np.array([[0, 0, 0]])

SENSOR_DIMS = {
    JOINT_ANGLES: 2,
    JOINT_VELOCITIES: 2,
    END_EFFECTOR_POINTS: 3,
    END_EFFECTOR_POINT_VELOCITIES: 3,
    ACTION: 2
}

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/dragon_hopping_pi2_example/'

x0s = []
ee_tgts = []
reset_conditions = []

common = {
    'experiment_name': 'dragon_hopping_pr2_example' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 1,
}

# Silence Add: Set up each condition.
for i in xrange(common['conditions']):
    """
    ja_x0, ee_pos_x0, ee_rot_x0 = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'initial'
    )
    ja_aux, _, _ = load_pose_from_npz(
        common['target_filename'], 'auxiliary_arm', str(i), 'initial'
    )
    _, ee_pos_tgt, ee_rot_tgt = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'target'
    )
    """
    ja_x0 = np.array([[-0.51, 0.94]])
    ee_pos_x0 = np.array([[0.023098, 0, 0.21839]])
    ee_rot_x0 = np.array([[[0.5, 0, 0.866],\
        [0.866, 1, 0],\
        [0, 0, 1]]])
    ja_aux = np.array([0., 0.])
    ee_pos_tgt = np.array([[0.023098, 0, 0.21891968]])
    ee_rot_tgt = np.array([[[0.5, 0, 0.866],\
        [0.866, 1, 0],\
        [0, 0, 1]]])

    x0 = np.zeros(10)
    x0[:2] = ja_x0
    x0[2:(2 + 3)] = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_x0, ee_rot_x0).T
    )

    ee_tgt = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T
    )

    aux_x0 = np.zeros(2)
    aux_x0[:] = ja_aux

    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[0:2],
        },
        AUXILIARY_ARM: {
            'mode': JOINT_SPACE,
            'data': aux_x0,
        },
    }

    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)

agent = {
    'type': AgentROS,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': 100,
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [],
}

if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

"""
# Silence DEL: Need to match ROS Agent
agent = {
    'type': AgentBox2D,
    'target_state' : np.array([5, 20, 0]),
    "world" : PointMassWorld,
    'render' : False,
    'x0': np.array([0, 5, 0, 0, 0, 0]),
    'rk': 0,
    'dt': 0.05,
    'substeps': 1,
    'conditions': common['conditions'],
    'pos_body_idx': np.array([]),
    'pos_body_offset': np.array([]),
    'T': 100,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
    'obs_include': [],
    'smooth_noise_var': 3.0,
}
"""

algorithm = {
    'type': AlgorithmTrajOptPi2,
    'conditions': common['conditions'],
}

algorithm['init_traj_distr'] = {
    'type': init_pd,
    'init_var': 10.0,
    'pos_gains': 5.0,
    'dQ': SENSOR_DIMS[ACTION],
    'dt': agent['dt'],
    'T': agent['T'],
}

action_cost = {
    'type': CostAction,
    'wu': np.array([5e-5, 5e-5])
}

fk_cost = {
    'type': CostDZ,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'target_end_effector': np.zeros(EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 1,
    'l2': 0.0001,
    'ramp_option': RAMP_CONSTANT,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [fk_cost],
    'weights': [1.0],
}

algorithm['traj_opt'] = {
    'type': TrajOptPi2,
    'kl_threshold': 2.0,
    'covariance_damping': 2.0,
    'min_temperature': 0.001,
}

algorithm['policy_opt'] = {}

config = {
    'iterations': 20,
    'num_samples': 5,
    'common': common,
    'verbose_trials': 0,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
    'dQ': algorithm['init_traj_distr']['dQ'],
}

common['info'] = generate_experiment_info(config)