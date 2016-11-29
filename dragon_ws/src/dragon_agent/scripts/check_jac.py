#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('gps_agent_lib')
import gps_agent_lib
from gps_agent_lib.msg import PositionCommand, TrialCommand, ControllerParams, LinGaussParams, SampleResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

import numpy as np

from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.agent.ros.ros_utils import policy_to_msg, msg_to_sample
from gps.proto.gps_pb2 import *

POS_COM_TOPIC = '/gps_controller_position_command'
TRIAL_COM_TOPIC = '/gps_controller_trial_command'
TEST_TOPIC = '/test_sub'
RESULT_TOPIC = '/gps_controller_report'

EE_SITES = np.array([[0,0,0]])

def listen(msg):
    print msg.__class__

def listen_report(msg):
    print msg.__class__
    sensor_data = msg.sensor_data

    eerot = sensor_data[END_EFFECTOR_ROTATIONS]
    eerot = np.array(eerot.data).reshape( eerot.shape)

    eejac = sensor_data[END_EFFECTOR_JACOBIANS]
    eejac = np.array(eejac.data).reshape(eejac.shape)

    eeptjac = sensor_data[END_EFFECTOR_POINT_JACOBIANS]
    eeptjac = np.array(eeptjac.data).reshape(eeptjac.shape)

    eerotjac = sensor_data[END_EFFECTOR_POINT_ROT_JACOBIANS]
    eerotjac = np.array(eerotjac.data).reshape(eerotjac.shape)

    checkjac, checkr = check_eept_jacobian(eejac[0], eerot[0])
    import pdb; pdb.set_trace();

def check_eept_jacobian(eejac, eerot, ee_sites=EE_SITES):
    n_sites = ee_sites.shape[0]
    n_actuator = eejac.shape[1]

    Jx = np.zeros((3*n_sites, 7))
    Jr = np.zeros((3*n_sites, 7))

    iq = slice(0,n_actuator)
    # Process each site.
    for i in range(n_sites):
        site_start = i*3
        site_end = (i+1)*3
        # Initialize.
        ovec = ee_sites[i]

        Jx[site_start:site_end, iq] = eejac[0:3,:]
        Jr[site_start:site_end, iq] = eejac[3:6,:]

        # Compute site Jacobian.
        ovec = eerot.dot(ovec)
        #"""
        Jx[site_start:site_end, iq] += \
            np.c_[Jr[site_start+1, iq].dot(ovec[2]) - Jr[site_start+2, iq].dot(ovec[1]) ,
             Jr[site_start+2, iq].dot(ovec[0]) - Jr[site_start, iq].dot(ovec[2]) ,
             Jr[site_start, iq].dot(ovec[1]) - Jr[site_start+1, iq].dot(ovec[0])].T
        #"""

        """
        Jx[site_start:site_end, iq] += \
            np.c_[Jr[site_start+1, iq]*ovec[2] - Jr[site_start+2, iq]*ovec[1] ,
                  Jr[site_start+2, iq]*ovec[0] - Jr[site_start  , iq]*ovec[2] ,
                  Jr[site_start  , iq]*ovec[1] - Jr[site_start+1, iq]*ovec[0]].T
        """
        #for k in range(n_actuator):
        #Jx[site_start , iq]  += Jr[site_start+1, iq]*ovec[2] - Jr[site_start+2, iq]*ovec[1]
        #Jx[site_start+1, iq] += Jr[site_start+2, iq]*ovec[0] - Jr[site_start  , iq]*ovec[2]
        #Jx[site_start+2, iq] += Jr[site_start  , iq]*ovec[1] - Jr[site_start+1, iq]*ovec[0]
        #"""
    return Jx, Jr

def get_lin_gauss_test(T=50):
    dX = 4
    x0 = np.zeros(dX)
    x0[0] = 1.0
    # lgpol = init_lqr({'x0' : x0, 'dU' : 2, 'dX' : dX, 'T' : T, 'dt' : 0.05})
    lgpol = init_pd({\
        'init_var' : 1.0, 'pos_gains' : 0.0,\
        'dQ' : 2, 'dt' : 0.05, 'x0' : x0,\
        'dU' : 2, 'dX' : 4, 'T' : T\
        })
    print 'T:', lgpol.T
    print 'dX:', lgpol.dX
    #Conver lgpol to message
    noise = np.zeros((T, 2))
    controller_params = policy_to_msg(lgpol, noise)
    return controller_params

def main():
    rospy.init_node('issue_com')
    pub = rospy.Publisher(TRIAL_COM_TOPIC, TrialCommand, queue_size=10)
    test_pub = rospy.Publisher(TEST_TOPIC, Empty, queue_size=10)
    sub = rospy.Subscriber(POS_COM_TOPIC, TrialCommand, listen)
    sub2 = rospy.Subscriber(RESULT_TOPIC, SampleResult, listen_report)
    #sub = rospy.Subscriber('/joint_states', JointState, listen)

    tc = TrialCommand()
    T = 1
    tc.controller = get_lin_gauss_test(T=T)
    tc.T = T
    tc.frequency = 20.0
    # NOTE: ordering of datatypes in state is determined by the order here
    tc.state_datatypes = [JOINT_ANGLES, JOINT_VELOCITIES]
    tc.obs_datatypes = tc.state_datatypes
    tc.ee_points = EE_SITES.reshape(EE_SITES.size).tolist()
    ee_points_tgt = np.array([[0,0,0.5]])
    tc.ee_points_tgt = ee_points_tgt.reshape(ee_points_tgt.size).tolist()

    r = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    pub.publish(pc)
    #    r.sleep()
    #    print 'published!'
    r.sleep()
    test_pub.publish(Empty())
    pub.publish(tc)
    rospy.spin()

print "Testing"
if __name__ == "__main__":
    main()