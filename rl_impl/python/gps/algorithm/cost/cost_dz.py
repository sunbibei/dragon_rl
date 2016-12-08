""" This file defines the forward kinematics cost function. """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_FK
from gps.algorithm.cost.cost import Cost
from gps.algorithm.cost.cost_utils import get_ramp_multiplier
from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS, TOOL_JOINT_ANGLES


class CostDZ(Cost):
    """
    Forward kinematics cost function. Used for costs involving the end
    effector position.
    """
    def __init__(self, hyperparams):
        config = copy.deepcopy(COST_FK)
        config.update(hyperparams)
        Cost.__init__(self, config)

    def eval(self, sample):
        """
        Evaluate forward kinematics (end-effector penalties) cost.
        Temporary note: This implements the 'joint' penalty type from
            the matlab code, with the velocity/velocity diff/etc.
            penalties removed. (use CostState instead)
        Args:
            sample: A single sample.
        """
        T = sample.T
        dX = sample.dX
        dU = sample.dU

        #wpm = get_ramp_multiplier(
        #    self._hyperparams['ramp_option'], T,
        #    wp_final_multiplier=self._hyperparams['wp_final_multiplier']
        #)
        #wp = self._hyperparams['wp'] * np.expand_dims(wpm, axis=-1)

        # Initialize terms.
        l = np.zeros(T)
        lu = np.zeros((T, dU))
        lx = np.zeros((T, dX))
        luu = np.zeros((T, dU, dU))
        lxx = np.zeros((T, dX, dX))
        lux = np.zeros((T, dU, dX))

        high = sample.get(TOOL_JOINT_ANGLES)
        ee_pt = sample.get(END_EFFECTOR_POINTS)
        ee_tgt = self._hyperparams['target_end_effector']
        dist = ee_pt - ee_tgt
        #tgt = pt.copy()
        #tgt[:, 2] = 0
        # print "pt: ", pt
        #tgt[:, 0] = 0
        #dist = pt - tgt
        # TODO: calc THE \delta x
        delta_x = np.sum(dist ** 2, axis=1)
        delta_z = high.max()
        alpha = self._hyperparams['alpha'];

        ell = alpha * delta_x - (1 - alpha) * np.abs(high)
        
        print "cost value: ", ell
        l = np.zeros(T)
        l[T-1] = -np.abs(delta_z)
        
        return l, lx, lu, lxx, luu, lux
