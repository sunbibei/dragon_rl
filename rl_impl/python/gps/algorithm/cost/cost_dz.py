""" This file defines the forward kinematics cost function. """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_FK
from gps.algorithm.cost.cost import Cost
from gps.algorithm.cost.cost_utils import get_ramp_multiplier
from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS


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

        # Initialize terms.
        l = np.zeros(T)
        lu = np.zeros((T, dU))
        lx = np.zeros((T, dX))
        luu = np.zeros((T, dU, dU))
        lxx = np.zeros((T, dX, dX))
        lux = np.zeros((T, dU, dX))

        pt = sample.get(END_EFFECTOR_POINTS)
        tgt = pt.copy()
        tgt[:, 2] = 0
        # print "pt: ", pt
        #tgt[:, 0] = 0
        dist = pt - tgt
        # TODO: calc THE \delta x
        delta_x = 0
        delta_z = dist.min()
        alpha = self._hyperparams['alpha'];

        ell = alpha * delta_x - (1 - alpha) * np.abs(delta_z)
        
        print "cost value: ", ell
        l = np.zeros(T)
        l[T-1] = ell
        
        return l, lx, lu, lxx, luu, lux
